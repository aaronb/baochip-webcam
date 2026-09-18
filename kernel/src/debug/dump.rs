// SPDX-License-Identifier: Apache-2.0

//! Binary thread and server tables for the `DebugThreads` and `DebugServers` platform calls.
//!
//! The text calls (`DebugProcesses` and friends) print process states for a working console.
//! These are for a watchdog inside a process whose peers may be deadlocked, so they report
//! what a deadlock is made of: every thread's saved registers (a thread parked in a syscall
//! keeps the call number and its arguments in a0..a7, and a send is resolved to the server it
//! targets) and every server's queue fill. `webcam-tools/uvc_debug.py` decodes both layouts.
//!
//! Thread table, little-endian:
//! - header (20 bytes): `THD1`, records written, threads seen, name table offset, name table length
//! - records (52 bytes): pid u8, tid u8, flags u8 (bit 0 ready, bit 1 running), 0 u8, pc, ra, sp, a0..a7,
//!   target server index u16 (0xffff: none), target server pid u16
//! - name table: pid u8, length u8, name bytes, per process
//!
//! Server table, little-endian:
//! - header (8 bytes): `SRV1`, records written
//! - records (20 bytes): server index u16, pid u16, messages not yet received u16, slots in use u16, slots
//!   u16, 0 u16, mask of threads waiting to receive u32, first word of the SID u32

use xous_kernel::{CID, MemoryFlags, PID, SysCallResult};

use crate::arch::process::{MAX_THREAD, Process as ArchProcess};
use crate::platform::bao1x::PlatformCallAbi;
use crate::services::{ProcessState, SystemServices};

const PAGE: usize = xous_kernel::arch::PAGE_SIZE;
/// syscall numbers (`xous::SysCallNumber`) whose a1 is a connection ID
const SYSCALL_SEND_MESSAGE: usize = 16;
const SYSCALL_TRY_SEND_MESSAGE: usize = 24;

const THREAD_HEADER: usize = 20;
const THREAD_RECORD: usize = 52;
/// kept free at the end of the thread table for the process names
const NAME_TABLE_ROOM: usize = 640;
const SERVER_HEADER: usize = 8;
const SERVER_RECORD: usize = 20;

/// Writer for the caller's page while it is aliased at `USERSPACE_BUFFER`. That alias exists
/// only in the kernel's address space: write with the kernel's mapping active.
struct Page {
    pos: usize,
}

impl Page {
    fn put(&mut self, bytes: &[u8]) -> bool {
        if self.pos + bytes.len() > PAGE {
            return false;
        }
        unsafe {
            core::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                (xous_kernel::arch::USERSPACE_BUFFER + self.pos) as *mut u8,
                bytes.len(),
            )
        };
        self.pos += bytes.len();
        true
    }

    fn put_at(&mut self, at: usize, bytes: &[u8]) {
        let pos = self.pos;
        self.pos = at;
        self.put(bytes);
        self.pos = pos;
    }
}

fn put_words(rec: &mut [u8], at: usize, words: &[usize]) {
    for (i, w) in words.iter().enumerate() {
        rec[at + i * 4..at + i * 4 + 4].copy_from_slice(&(*w as u32).to_le_bytes());
    }
}

fn activate_kernel_map(ss: &SystemServices) {
    ss.get_process(PID::new(1).unwrap()).unwrap().mapping.activate().unwrap();
}

/// Fill the page at `page_vaddr` in the caller's address space with the table `op` names.
/// Returns the page address and the number of bytes written.
pub fn handle(pid: PID, op: PlatformCallAbi, page_vaddr: usize) -> SysCallResult {
    if page_vaddr & (PAGE - 1) != 0 {
        return Err(xous_kernel::Error::BadAddress);
    }
    let phys = crate::arch::mem::virt_to_phys(page_vaddr)?;
    if phys < utralib::HW_SRAM_MEM || phys >= utralib::HW_SRAM_MEM + utralib::HW_SRAM_MEM_LEN {
        return Err(xous_kernel::Error::BadAddress);
    }
    // alias the caller's page into the kernel, as the text debug calls in `syscall.rs` do
    SystemServices::with(activate_kernel_map);
    crate::mem::MemoryManager::with_mut(|mm| {
        crate::arch::mem::map_page_inner(
            mm,
            PID::new(1).unwrap(),
            phys,
            xous_kernel::arch::USERSPACE_BUFFER,
            MemoryFlags::R | MemoryFlags::W,
            false,
        )
        .unwrap();
    });
    unsafe { core::ptr::write_bytes(xous_kernel::arch::USERSPACE_BUFFER as *mut u8, 0, PAGE) };

    let used = match op {
        PlatformCallAbi::DebugThreads => threads(pid),
        PlatformCallAbi::DebugServers => servers(pid),
        _ => 0,
    };

    // both tables finish with the caller current and the kernel's mapping active
    crate::mem::MemoryManager::with_mut(|mm| {
        crate::arch::mem::unmap_page_inner(mm, xous_kernel::arch::USERSPACE_BUFFER).unwrap();
    });
    SystemServices::with(|ss| ss.get_process(pid).unwrap().mapping.activate().unwrap());
    Ok(xous_kernel::Result::Scalar5(page_vaddr, used, 0, 0, 0))
}

fn threads(caller: PID) -> usize {
    let mut page = Page { pos: THREAD_HEADER };
    let mut written = 0u32;
    let mut seen = 0u32;
    SystemServices::with(|ss| {
        for process in &ss.processes {
            if process.free() {
                continue;
            }
            let (ready, running) = match process.state() {
                ProcessState::Ready(x) | ProcessState::Exception(x) | ProcessState::BlockedException(x) => {
                    (x, None)
                }
                ProcessState::Running(x) => (x, Some(process.current_thread)),
                _ => (0, None),
            };
            let mut mapped = false;
            for tid in 0..MAX_THREAD {
                // thread contexts live in the process's own address space
                if !mapped {
                    process.activate().unwrap();
                    mapped = true;
                }
                let thread = ArchProcess::with_current(|p| *p.thread(tid));
                if thread.sepc == 0 && thread.registers[1] == 0 {
                    continue;
                }
                let a0 = thread.registers[9];
                let target = if a0 == SYSCALL_SEND_MESSAGE || a0 == SYSCALL_TRY_SEND_MESSAGE {
                    ss.sidx_from_cid(thread.registers[10] as CID)
                } else {
                    None
                };
                activate_kernel_map(ss);
                mapped = false;
                seen += 1;
                if page.pos + THREAD_RECORD > PAGE - NAME_TABLE_ROOM {
                    continue;
                }
                let mut flags = 0u8;
                if ready & (1 << tid) != 0 {
                    flags |= 1;
                }
                if running == Some(tid) {
                    flags |= 2;
                }
                let (target_sidx, target_pid) = target
                    .and_then(|sidx| {
                        ss.servers.get(sidx)?.as_ref().map(|s| (sidx as u16, s.pid.get() as u16))
                    })
                    .unwrap_or((0xffff, 0));
                let mut rec = [0u8; THREAD_RECORD];
                rec[0] = process.pid.get();
                rec[1] = tid as u8;
                rec[2] = flags;
                put_words(&mut rec, 4, &[thread.sepc, thread.registers[0], thread.registers[1]]);
                put_words(&mut rec, 16, &thread.registers[9..17]);
                rec[48..50].copy_from_slice(&target_sidx.to_le_bytes());
                rec[50..52].copy_from_slice(&target_pid.to_le_bytes());
                page.put(&rec);
                written += 1;
            }
        }
        ss.get_process(caller).unwrap().activate().unwrap();
        activate_kernel_map(ss);

        let names_at = page.pos;
        for process in &ss.processes {
            if process.free() {
                continue;
            }
            let name = ss.process_name(process.pid).unwrap_or("").as_bytes();
            let name = &name[..name.len().min(30)];
            if page.pos + 2 + name.len() > PAGE {
                break;
            }
            page.put(&[process.pid.get(), name.len() as u8]);
            page.put(name);
        }
        let names_len = page.pos - names_at;
        page.put_at(0, b"THD1");
        page.put_at(4, &written.to_le_bytes());
        page.put_at(8, &seen.to_le_bytes());
        page.put_at(12, &(names_at as u32).to_le_bytes());
        page.put_at(16, &(names_len as u32).to_le_bytes());
    });
    page.pos
}

fn servers(caller: PID) -> usize {
    let mut page = Page { pos: SERVER_HEADER };
    let mut written = 0u32;
    SystemServices::with(|ss| {
        for (sidx, server) in ss.servers.iter().enumerate() {
            let Some(server) = server else {
                continue;
            };
            let Ok(owner) = ss.get_process(server.pid) else {
                continue;
            };
            // the queue lives in the owning process's address space
            owner.activate().unwrap();
            let (pending, used, capacity, waiting) = server.debug_queue_stats();
            activate_kernel_map(ss);
            let mut rec = [0u8; SERVER_RECORD];
            rec[0..2].copy_from_slice(&(sidx as u16).to_le_bytes());
            rec[2..4].copy_from_slice(&(server.pid.get() as u16).to_le_bytes());
            rec[4..6].copy_from_slice(&(pending as u16).to_le_bytes());
            rec[6..8].copy_from_slice(&(used as u16).to_le_bytes());
            rec[8..10].copy_from_slice(&(capacity as u16).to_le_bytes());
            rec[12..16].copy_from_slice(&(waiting as u32).to_le_bytes());
            rec[16..20].copy_from_slice(&server.sid.to_array()[0].to_le_bytes());
            if !page.put(&rec) {
                break;
            }
            written += 1;
        }
        ss.get_process(caller).unwrap().activate().unwrap();
        activate_kernel_map(ss);
        page.put_at(0, b"SRV1");
        page.put_at(4, &written.to_le_bytes());
    });
    page.pos
}
