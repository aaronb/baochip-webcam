//! Minimal USB Video Class (UVC 1.0) function: uncompressed UYVY in a few fixed sizes, streamed
//! over a bulk IN endpoint.
//!
//! Data path:
//!
//! 1. `bao-video` captures image data and lends it to this service in chunks of up to `UVC_CHUNK_PAYLOADS`
//!    payloads (`Opcode::UvcSendFrame`). A chunk may be a whole frame (the small mode) or a part of one (the
//!    large modes, whose frames are captured in bands over several sensor frames). Flags mark the chunk that
//!    starts and the one that ends a frame.
//! 2. The main loop copies the chunk into an IFRAM staging buffer laid out as payload slots, each a complete
//!    UVC payload: a 2-byte header followed by the image bytes (see `stage_chunk`).
//! 3. A software interrupt kicks the streamer. In interrupt context, one bulk transfer descriptor (TD) per
//!    slot is enqueued; each completion chains the next slot. Every payload is shorter than a multiple of the
//!    max packet size (or gets a zero-length packet appended), so it ends with a short packet, which is how
//!    the host delimits bulk payloads.
//!
//! Stream lifecycle: the host negotiates with PROBE/COMMIT control requests. This core completes
//! the data stage of those (OUT) requests in hardware without telling the stack, so the class
//! learns about them from the SETUP packet the driver records (`poll`) and reads the host's
//! proposal out of the EP0 buffer afterwards. COMMIT starts the stream; the observer (bao-video)
//! is told which mode was chosen. A CLEAR_FEATURE(ENDPOINT_HALT) on the streaming endpoint (what
//! Linux and Windows send to stop a bulk video stream) or a bus reset stops it.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use bao1x_hal::usb::driver::{CRG_IN, CRG_INT_TARGET, CRG_XFER_AZP, CorigineUsb};
use num_traits::ToPrimitive;
use usb_device::Result;
use usb_device::UsbDirection;
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, Request, RequestType};

use crate::api::{Opcode, UVC_CHUNK_PAYLOADS, UVC_MAX_PAYLOAD_DATA, UVC_MODES, UvcMode};

/// Endpoint number used for the video streaming bulk IN endpoint. This is assigned explicitly
/// because the HAL's automatic endpoint allocator only handles the IN/OUT pair pattern used by the
/// HID and CDC classes. With the FIDO device removed (see `hw.rs`), EP1 holds the keyboard, EP2/EP3
/// hold the CDC serial endpoints, and EP4 IN is free.
pub const UVC_EP_NUM: usize = 4;
/// High-speed bulk max packet size
pub const UVC_MPS: u16 = 512;

/// Payload header: bHeaderLength + bmHeaderInfo only (no PTS/SCR)
pub const PAYLOAD_HDR: usize = 2;
/// Distance between payload slots in the staging buffer; fits the largest payload, word-aligned
pub const PAYLOAD_STRIDE: usize = (PAYLOAD_HDR + UVC_MAX_PAYLOAD_DATA + 3) & !3;
/// Size of the IFRAM staging buffer: one chunk of payloads
pub const STAGING_BYTES: usize = UVC_CHUNK_PAYLOADS * PAYLOAD_STRIDE;

const _: () = assert!(STAGING_BYTES <= 10 * 4096);

// USB video class codes
const USB_CLASS_VIDEO: u8 = 0x0E;
const SC_VIDEOCONTROL: u8 = 0x01;
const SC_VIDEOSTREAMING: u8 = 0x02;
const SC_VIDEO_INTERFACE_COLLECTION: u8 = 0x03;
const CS_INTERFACE: u8 = 0x24;
const VC_HEADER: u8 = 0x01;
const VC_INPUT_TERMINAL: u8 = 0x02;
const VC_OUTPUT_TERMINAL: u8 = 0x03;
const VS_INPUT_HEADER: u8 = 0x01;
const VS_FORMAT_UNCOMPRESSED: u8 = 0x04;
const VS_FRAME_UNCOMPRESSED: u8 = 0x05;
const VS_PROBE_CONTROL: u8 = 0x01;
const VS_COMMIT_CONTROL: u8 = 0x02;
const ITT_CAMERA: u16 = 0x0201;
const TT_STREAMING: u16 = 0x0101;
const INPUT_TERMINAL_ID: u8 = 1;
const OUTPUT_TERMINAL_ID: u8 = 2;
// class-specific request codes
const SET_CUR: u8 = 0x01;
const GET_CUR: u8 = 0x81;
const GET_MIN: u8 = 0x82;
const GET_MAX: u8 = 0x83;
const GET_RES: u8 = 0x84;
const GET_LEN: u8 = 0x85;
const GET_INFO: u8 = 0x86;
const GET_DEF: u8 = 0x87;
/// UVC 1.0 probe/commit control length
const PROBE_LEN: usize = 26;

/// GUID for the UYVY pixel format (first byte of each pixel pair is chroma, then luma). The
/// camera DMA lands the sensor's 8-bit stream with the first byte of each pair in the high half of
/// a 16-bit word, so luma sits at odd byte addresses: exactly UYVY byte order.
const UYVY_GUID: [u8; 16] =
    [b'U', b'Y', b'V', b'Y', 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71];

/// Vendor request (device-to-host, recipient device) that returns `DebugCounters`: wValue 0 for
/// the original 32-byte report, 1 for `DebugCounters::extended_report`. Vendor requests are
/// answered in interrupt context, so they work even when the service's main loop is wedged.
pub const VENDOR_REQ_DEBUG: u8 = 0x51;
/// Vendor request for the kernel thread and server tables (`DebugDump`). wValue `DUMP_REQUEST`
/// asks the watchdog thread for a new capture; otherwise wValue is the table (0 threads,
/// 1 servers) and wIndex the byte offset. Every reply starts with the capture's generation, 0
/// while a capture is being written.
pub const VENDOR_REQ_DUMP: u8 = 0x52;
pub const DUMP_REQUEST: u16 = 0xffff;
/// Vendor request that injects a fault, for testing hang fixes. wValue `INJECT_STALL` holds the
/// main loop for wIndex ms and then logs a line, `INJECT_STALL_QUIET` holds it without logging,
/// and `INJECT_FLOOD` logs wIndex lines from a helper thread. Replies 1 if the request was taken.
pub const VENDOR_REQ_INJECT: u8 = 0x53;
pub const INJECT_STALL: u16 = 1;
pub const INJECT_STALL_QUIET: u16 = 2;
pub const INJECT_FLOOD: u16 = 3;

/// `DebugCounters::phase`: what the main loop is doing. Anything but idle for seconds is a hang.
pub const PHASE_IDLE: u32 = 0;
pub const PHASE_HANDLING: u32 = 1;
pub const PHASE_INJECT_KEY: u32 = 2;
pub const PHASE_LOG: u32 = 3;
pub const PHASE_NAMES: u32 = 4;
pub const PHASE_SERIAL_WRITE: u32 = 5;
pub const PHASE_SLEEP: u32 = 6;
pub const PHASE_LOG_HOOK: u32 = 7;
pub const PHASE_BUS_RESET: u32 = 8;

/// Liveness counters readable over USB with `VENDOR_REQ_DEBUG`. Written by the main loop, the
/// interrupt handler and the watchdog thread; read in interrupt context.
#[derive(Default)]
pub struct DebugCounters {
    /// incremented once per main-loop iteration
    pub main_ticks: AtomicU32,
    /// opcode of the message the main loop most recently received
    pub last_opcode: AtomicU32,
    /// serial listener mode: 0 none, 1 ascii, 2 binary, 3 console
    pub listen_mode: AtomicU32,
    /// number of `UvcKick` software interrupts serviced
    pub kicks: AtomicU32,
    /// number of bulk IN completions seen on the video endpoint
    pub completions: AtomicU32,
    /// number of VS_COMMIT requests accepted
    pub commits: AtomicU32,
    /// `PHASE_*`
    pub phase: AtomicU32,
    /// the last opcodes the main loop received, in a ring
    opcodes: [AtomicU32; 8],
    opcode_pos: AtomicU32,
    /// ms since `main_ticks` last moved, as the watchdog thread saw it
    pub still_ms: AtomicU32,
    /// watchdog thread iterations; if this stops, so has the watchdog
    pub watchdog_beats: AtomicU32,
    /// notifications refused because the receiving queue was full
    pub drop_frame_done: AtomicU32,
    pub drop_stream_change: AtomicU32,
    pub drop_serial_rx: AtomicU32,
    pub drop_observer: AtomicU32,
    pub drop_inject: AtomicU32,
    /// console keys dropped because the keyboard server's queue was full
    pub drop_keys: AtomicU32,
    /// frames offered while another was already waiting
    pub refused_frames: AtomicU32,
    pub bad_chunks: AtomicU32,
    pub double_locks: AtomicU32,
    pub stream_changes: AtomicU32,
    /// lines `INJECT_FLOOD` still has to log
    flood_lines: AtomicU32,
}

impl DebugCounters {
    /// Main loop: note a received message
    pub fn record_opcode(&self, id: u32) {
        let pos = self.opcode_pos.fetch_add(1, Ordering::SeqCst) as usize;
        self.opcodes[pos % self.opcodes.len()].store(id, Ordering::SeqCst);
        self.last_opcode.store(id, Ordering::SeqCst);
    }

    /// `VENDOR_REQ_DEBUG` wValue 1: phase, ms since the main loop moved, watchdog beats, the
    /// last 8 opcodes oldest first (u16), the drop counters (u16: frame done, stream change,
    /// serial rx, observer, inject, refused frames, bad chunks, double locks), then u32 keys
    /// dropped, stream changes, and the dump generation.
    fn extended_report(&self, dump_generation: u32) -> [u8; 56] {
        let mut d = [0u8; 56];
        let load = |a: &AtomicU32| a.load(Ordering::SeqCst);
        d[0..4].copy_from_slice(&load(&self.phase).to_le_bytes());
        d[4..8].copy_from_slice(&load(&self.still_ms).to_le_bytes());
        d[8..12].copy_from_slice(&load(&self.watchdog_beats).to_le_bytes());
        let pos = load(&self.opcode_pos) as usize;
        for i in 0..self.opcodes.len() {
            let op = load(&self.opcodes[(pos + i) % self.opcodes.len()]) as u16;
            d[12 + 2 * i..14 + 2 * i].copy_from_slice(&op.to_le_bytes());
        }
        let small = [
            &self.drop_frame_done,
            &self.drop_stream_change,
            &self.drop_serial_rx,
            &self.drop_observer,
            &self.drop_inject,
            &self.refused_frames,
            &self.bad_chunks,
            &self.double_locks,
        ];
        for (i, a) in small.iter().enumerate() {
            d[28 + 2 * i..30 + 2 * i].copy_from_slice(&(load(a).min(0xffff) as u16).to_le_bytes());
        }
        d[44..48].copy_from_slice(&load(&self.drop_keys).to_le_bytes());
        d[48..52].copy_from_slice(&load(&self.stream_changes).to_le_bytes());
        d[52..56].copy_from_slice(&dump_generation.to_le_bytes());
        d
    }
}

const PAGE: usize = 4096;
/// kernel `PlatformCallAbi::DebugThreads` and `DebugServers` (`debug-proc` kernels only)
const KERNEL_DEBUG_THREADS: usize = 4;
const KERNEL_DEBUG_SERVERS: usize = 5;
const DUMP_TABLES: usize = 2;
/// `VENDOR_REQ_DUMP` reply size: kept under the 64-byte EP0 packet so it is a single short packet
const DUMP_REPLY: usize = 60;
/// how often the watchdog thread looks at the main loop
const WATCH_MS: usize = 250;
/// a main loop inside a handler for this long gets a dump without being asked
const AUTO_DUMP_MS: u32 = 5000;

#[repr(C, align(4096))]
struct DebugPage([u8; PAGE]);

/// Kernel thread and server tables (layouts in the kernel's `debug::dump`), captured by the
/// watchdog thread because the kernel calls can't be made from interrupt context, and read out
/// in pieces with `VENDOR_REQ_DUMP`.
pub struct DebugDump {
    /// capture number, 0 while a capture is being written
    generation: AtomicU32,
    captures: AtomicU32,
    requested: AtomicBool,
    /// `DUMP_TABLES` pages, allocated once for the life of the process
    tables: *mut u8,
}

// Safety: the watchdog thread is the only writer, and a reader discards any copy that a capture
// overlapped (see `read`).
unsafe impl Send for DebugDump {}
unsafe impl Sync for DebugDump {}

impl DebugDump {
    pub fn new() -> Self {
        Self {
            generation: AtomicU32::new(0),
            captures: AtomicU32::new(0),
            requested: AtomicBool::new(false),
            tables: Box::leak(vec![0u8; DUMP_TABLES * PAGE].into_boxed_slice()).as_mut_ptr(),
        }
    }

    pub fn generation(&self) -> u32 { self.generation.load(Ordering::SeqCst) }

    /// Watchdog thread: take both tables from the kernel
    fn capture(&self, scratch: &mut DebugPage) {
        let number = self.captures.fetch_add(1, Ordering::SeqCst) + 1;
        self.generation.store(0, Ordering::SeqCst);
        for (i, op) in [KERNEL_DEBUG_THREADS, KERNEL_DEBUG_SERVERS].iter().enumerate() {
            scratch.0.fill(0);
            let taken = xous::rsyscall(xous::SysCall::PlatformSpecific(
                *op,
                scratch.0.as_mut_ptr() as usize,
                0,
                0,
                0,
                0,
                0,
            ))
            .is_ok();
            unsafe {
                let dst = self.tables.add(i * PAGE);
                if taken {
                    core::ptr::copy_nonoverlapping(scratch.0.as_ptr(), dst, PAGE);
                } else {
                    core::ptr::write_bytes(dst, 0, PAGE);
                }
            }
        }
        self.generation.store(number, Ordering::SeqCst);
    }

    /// Interrupt context: one `VENDOR_REQ_DUMP` reply. Returns its length.
    fn read(&self, table: usize, offset: usize, out: &mut [u8; DUMP_REPLY]) -> usize {
        let generation = self.generation();
        let mut len = 4;
        if generation != 0 && table < DUMP_TABLES && offset < PAGE {
            let n = (PAGE - offset).min(DUMP_REPLY - 4);
            unsafe {
                core::ptr::copy_nonoverlapping(
                    self.tables.add(table * PAGE + offset),
                    out[4..].as_mut_ptr(),
                    n,
                )
            };
            len += n;
        }
        // a capture that began during the copy makes it worthless
        let generation = if self.generation() == generation { generation } else { 0 };
        out[..4].copy_from_slice(&generation.to_le_bytes());
        len
    }
}

/// Start the watchdog thread, which takes a `DebugDump` when asked or when the main loop has sat
/// in one phase of a handler for `AUTO_DUMP_MS`, and the thread that serves `INJECT_FLOOD`. The
/// watchdog must not log: the log server can be part of a hang.
pub fn start_debug_threads(dbg: Arc<DebugCounters>, dump: Arc<DebugDump>) {
    std::thread::spawn({
        let dbg = dbg.clone();
        move || {
            let tt = ticktimer::Ticktimer::new().unwrap();
            let mut scratch = Box::new(DebugPage([0; PAGE]));
            let mut last = (dbg.main_ticks.load(Ordering::SeqCst), dbg.phase.load(Ordering::SeqCst));
            let mut still_ms = 0u32;
            let mut auto_taken = false;
            loop {
                tt.sleep_ms(WATCH_MS).ok();
                dbg.watchdog_beats.fetch_add(1, Ordering::SeqCst);
                // a handler that moves on to another phase (a long sleep ending in a stuck log
                // call) gets a fresh dump of the phase it ends up stuck in
                let now = (dbg.main_ticks.load(Ordering::SeqCst), dbg.phase.load(Ordering::SeqCst));
                if now != last {
                    last = now;
                    still_ms = 0;
                    auto_taken = false;
                } else {
                    still_ms = still_ms.saturating_add(WATCH_MS as u32);
                }
                dbg.still_ms.store(still_ms, Ordering::SeqCst);
                let stuck = still_ms >= AUTO_DUMP_MS && now.1 != PHASE_IDLE;
                if dump.requested.swap(false, Ordering::SeqCst) || (stuck && !auto_taken) {
                    auto_taken |= stuck;
                    dump.capture(&mut scratch);
                }
            }
        }
    });
    std::thread::spawn(move || {
        let tt = ticktimer::Ticktimer::new().unwrap();
        loop {
            tt.sleep_ms(100).ok();
            let lines = dbg.flood_lines.swap(0, Ordering::SeqCst);
            for i in 0..lines {
                log::info!("debug flood {}/{}", i + 1, lines);
            }
        }
    });
}

/// Payload header bits
const HDR_EOH: u8 = 0x80;
const HDR_EOF: u8 = 0x02;
const HDR_FID: u8 = 0x01;

/// What the streamer is transmitting: one chunk of payloads staged by the main loop.
#[derive(Clone, Copy, Default)]
struct Chunk {
    /// number of payload slots in use
    payloads: usize,
    /// byte length of each payload but the last (header included)
    payload_len: usize,
    /// byte length of the last payload (header included)
    last_len: usize,
    /// this chunk ends a frame
    eof: bool,
}

pub struct UvcClass<'a, B: UsbBus> {
    vc_if: InterfaceNumber,
    vs_if: InterfaceNumber,
    ep_in: EndpointIn<'a, B>,
    hw: Arc<Mutex<CorigineUsb>>,
    conn: xous::CID,
    /// physical address of the staging buffer
    staging_phys: usize,
    /// host has committed a stream
    streaming: AtomicBool,
    /// a staged chunk is being transmitted; the staging buffer must not be touched
    frame_active: AtomicBool,
    /// completed frames (chunks with the end-of-frame flag)
    frames_sent: AtomicU32,
    /// chunks transmitted
    chunks_sent: AtomicU32,
    /// mode index (0-based) the host selected with its last PROBE; used by COMMIT
    selected: AtomicUsize,
    /// a PROBE SET_CUR was seen and its data stage not yet inspected
    probe_pending: bool,
    /// a TD is enqueued and its completion hasn't been seen yet
    in_flight: bool,
    /// the next completion belongs to a TD from a stream that has since been stopped
    discard_completion: bool,
    /// index of the payload slot currently in flight
    cursor: usize,
    chunk: Chunk,
    pub dbg: Arc<DebugCounters>,
    pub dump: Arc<DebugDump>,
}

impl<'a, B: UsbBus> UvcClass<'a, B> {
    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        hw: Arc<Mutex<CorigineUsb>>,
        conn: xous::CID,
        staging_phys: usize,
    ) -> Self {
        let vc_if = alloc.interface();
        let vs_if = alloc.interface();
        let ep_in = alloc
            .alloc(
                Some(EndpointAddress::from_parts(UVC_EP_NUM, UsbDirection::In)),
                EndpointType::Bulk,
                UVC_MPS,
                0,
            )
            .expect("couldn't allocate UVC bulk endpoint");
        Self {
            vc_if,
            vs_if,
            ep_in,
            hw,
            conn,
            staging_phys,
            streaming: AtomicBool::new(false),
            frame_active: AtomicBool::new(false),
            frames_sent: AtomicU32::new(0),
            chunks_sent: AtomicU32::new(0),
            selected: AtomicUsize::new(0),
            probe_pending: false,
            in_flight: false,
            discard_completion: false,
            cursor: 0,
            chunk: Chunk::default(),
            dbg: Arc::new(DebugCounters::default()),
            dump: Arc::new(DebugDump::new()),
        }
    }

    pub fn is_streaming(&self) -> bool { self.streaming.load(Ordering::SeqCst) }

    pub fn frame_busy(&self) -> bool { self.frame_active.load(Ordering::SeqCst) }

    pub fn frames_sent(&self) -> u32 { self.frames_sent.load(Ordering::SeqCst) }

    /// Mode index (0-based into `UVC_MODES`) of the current or last committed stream
    pub fn selected_mode(&self) -> usize { self.selected.load(Ordering::SeqCst) }

    /// Describe the chunk that has just been staged, then follow with a `UvcKick` software IRQ.
    pub fn set_chunk(&mut self, payloads: usize, payload_len: usize, last_len: usize, eof: bool) {
        self.chunk = Chunk { payloads, payload_len, last_len, eof };
        self.cursor = 0;
        self.frame_active.store(true, Ordering::SeqCst);
    }

    /// Forget any in-progress transfer state. Used after the hardware has been re-initialized.
    pub fn reset_state(&mut self) {
        self.in_flight = false;
        self.discard_completion = false;
        self.cursor = 0;
        self.frame_active.store(false, Ordering::SeqCst);
        if self.streaming.swap(false, Ordering::SeqCst) {
            self.notify_stream_state(0);
        }
    }

    /// The probe/commit control we report for every GET_* request, for the selected mode.
    fn probe_data(&self) -> [u8; PROBE_LEN] {
        let idx = self.selected_mode();
        let mode = &UVC_MODES[idx];
        let mut d = [0u8; PROBE_LEN];
        d[0..2].copy_from_slice(&1u16.to_le_bytes()); // bmHint: dwFrameInterval is fixed
        d[2] = 1; // bFormatIndex
        d[3] = (idx + 1) as u8; // bFrameIndex
        d[4..8].copy_from_slice(&mode.interval.to_le_bytes());
        // 8..18: wKeyFrameRate, wPFrameRate, wCompQuality, wCompWindowSize, wDelay - all zero
        d[18..22].copy_from_slice(&(mode.frame_bytes() as u32).to_le_bytes()); // dwMaxVideoFrameSize
        d[22..26].copy_from_slice(&((PAYLOAD_HDR + mode.payload_data()) as u32).to_le_bytes()); // dwMaxPayloadTransferSize
        d
    }

    /// Read the host's PROBE proposal out of the EP0 buffer (the hardware completed the data
    /// stage there) and adopt its frame index if it names one of ours.
    fn adopt_probe_proposal(&mut self) {
        if !self.probe_pending {
            return;
        }
        self.probe_pending = false;
        let Ok(hw) = self.hw.try_lock() else {
            return;
        };
        let buf = hw.ep0_buf.load(Ordering::SeqCst) as usize;
        if buf == 0 {
            return;
        }
        let data = unsafe { core::slice::from_raw_parts(buf as *const u8, PROBE_LEN) };
        let format = data[2];
        let frame = data[3] as usize;
        if format == 1 && frame >= 1 && frame <= UVC_MODES.len() {
            self.selected.store(frame - 1, Ordering::SeqCst);
        }
    }

    fn notify_stream_state(&self, state: usize) {
        if xous::try_send_message(
            self.conn,
            xous::Message::new_scalar(
                Opcode::IrqUvcStreamChange.to_usize().unwrap(),
                state,
                self.selected_mode(),
                0,
                0,
            ),
        )
        .is_err()
        {
            self.dbg.drop_stream_change.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn notify_chunk_done(&self) {
        if xous::try_send_message(
            self.conn,
            xous::Message::new_scalar(Opcode::IrqUvcFrameDone.to_usize().unwrap(), 0, 0, 0, 0),
        )
        .is_err()
        {
            self.dbg.drop_frame_done.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn start_stream(&mut self) {
        // Abandon any chunk in progress. A TD left in the hardware ring by a previous stream may
        // still complete when the host reads again (then its completion is discarded), or may be
        // gone entirely if the bus was reset in between. Either way, don't let it block the new
        // stream: a discarded completion re-kicks, and at worst one payload is sent twice.
        self.cursor = 0;
        self.frame_active.store(false, Ordering::SeqCst);
        if self.in_flight {
            self.discard_completion = true;
            self.in_flight = false;
        }
        self.streaming.store(true, Ordering::SeqCst);
        self.notify_stream_state(1);
    }

    fn stop_stream(&mut self) {
        if self.streaming.swap(false, Ordering::SeqCst) {
            self.cursor = 0;
            self.frame_active.store(false, Ordering::SeqCst);
            if self.in_flight {
                self.discard_completion = true;
            }
            self.notify_stream_state(0);
        }
    }

    /// Interrupt context: start transmitting the staged chunk, if nothing is in flight.
    pub fn kick(&mut self) {
        if self.staging_phys == 0 || !self.is_streaming() || self.in_flight || !self.frame_busy() {
            return;
        }
        self.enqueue_payload();
    }

    /// Interrupt context: enqueue the payload at `self.cursor`.
    fn enqueue_payload(&mut self) {
        let slot = self.staging_phys + self.cursor * PAYLOAD_STRIDE;
        let len =
            if self.cursor + 1 == self.chunk.payloads { self.chunk.last_len } else { self.chunk.payload_len };
        let flags = if len % (UVC_MPS as usize) == 0 { CRG_XFER_AZP } else { 0 };
        match self.hw.try_lock() {
            Ok(mut hw) => {
                let pei = CorigineUsb::pei(UVC_EP_NUM as u8, CRG_IN);
                let _ = hw.app_ptr[pei - 2].take();
                hw.bulk_xfer(UVC_EP_NUM as u8, CRG_IN, slot, len, CRG_INT_TARGET, flags);
                self.in_flight = true;
            }
            Err(_) => {
                crate::println!("UVC: hw lock busy, dropping chunk");
                self.cursor = 0;
                self.frame_active.store(false, Ordering::SeqCst);
                self.notify_chunk_done();
            }
        }
    }

    /// Snapshot of the debug counters and stream state, as returned by `VENDOR_REQ_DEBUG`.
    fn debug_report(&self) -> [u8; 32] {
        let mut d = [0u8; 32];
        d[0..4].copy_from_slice(&self.dbg.main_ticks.load(Ordering::SeqCst).to_le_bytes());
        d[4..8].copy_from_slice(&self.dbg.last_opcode.load(Ordering::SeqCst).to_le_bytes());
        d[8..12].copy_from_slice(&self.dbg.listen_mode.load(Ordering::SeqCst).to_le_bytes());
        d[12..16].copy_from_slice(&self.dbg.kicks.load(Ordering::SeqCst).to_le_bytes());
        d[16..20].copy_from_slice(&self.dbg.completions.load(Ordering::SeqCst).to_le_bytes());
        d[20..24].copy_from_slice(&self.dbg.commits.load(Ordering::SeqCst).to_le_bytes());
        d[24..28].copy_from_slice(&self.frames_sent().to_le_bytes());
        d[28] = self.is_streaming() as u8;
        d[29] = self.frame_busy() as u8;
        d[30] = self.in_flight as u8
            | ((self.discard_completion as u8) << 1)
            | (((self.staging_phys != 0) as u8) << 2)
            | ((self.selected_mode() as u8) << 4);
        d[31] = self.cursor as u8;
        d
    }
}

impl<'a, B: UsbBus> UsbClass<B> for UvcClass<'a, B> {
    fn get_configuration_descriptors(&self, w: &mut DescriptorWriter) -> Result<()> {
        let vs_if: u8 = self.vs_if.into();
        let ep_addr: u8 = self.ep_in.address().into();

        w.iad(self.vc_if, 2, USB_CLASS_VIDEO, SC_VIDEO_INTERFACE_COLLECTION, 0)?;

        // ---- VideoControl interface: header, camera input terminal, streaming output terminal
        w.interface(self.vc_if, USB_CLASS_VIDEO, SC_VIDEOCONTROL, 0)?;
        const VC_TOTAL_LEN: u16 = 13 + 18 + 9;
        let clock: u32 = 1_000_000;
        let mut hdr = [0u8; 11];
        hdr[0] = VC_HEADER;
        hdr[1..3].copy_from_slice(&0x0100u16.to_le_bytes()); // bcdUVC 1.00
        hdr[3..5].copy_from_slice(&VC_TOTAL_LEN.to_le_bytes());
        hdr[5..9].copy_from_slice(&clock.to_le_bytes());
        hdr[9] = 1; // bInCollection
        hdr[10] = vs_if; // baInterfaceNr
        w.write(CS_INTERFACE, &hdr)?;

        let mut it = [0u8; 16];
        it[0] = VC_INPUT_TERMINAL;
        it[1] = INPUT_TERMINAL_ID;
        it[2..4].copy_from_slice(&ITT_CAMERA.to_le_bytes());
        it[4] = 0; // bAssocTerminal
        it[5] = 0; // iTerminal
        // 6..12: objective focal length min/max, ocular focal length: unspecified
        it[12] = 3; // bControlSize
        // 13..16: bmControls: no camera controls
        w.write(CS_INTERFACE, &it)?;

        let mut ot = [0u8; 7];
        ot[0] = VC_OUTPUT_TERMINAL;
        ot[1] = OUTPUT_TERMINAL_ID;
        ot[2..4].copy_from_slice(&TT_STREAMING.to_le_bytes());
        ot[4] = 0; // bAssocTerminal
        ot[5] = INPUT_TERMINAL_ID; // bSourceID
        ot[6] = 0; // iTerminal
        w.write(CS_INTERFACE, &ot)?;

        // ---- VideoStreaming interface: input header, one uncompressed format, one frame
        // descriptor per mode
        w.interface(self.vs_if, USB_CLASS_VIDEO, SC_VIDEOSTREAMING, 0)?;
        let vs_total_len: u16 = 14 + 27 + 30 * UVC_MODES.len() as u16;
        let mut ih = [0u8; 12];
        ih[0] = VS_INPUT_HEADER;
        ih[1] = 1; // bNumFormats
        ih[2..4].copy_from_slice(&vs_total_len.to_le_bytes());
        ih[4] = ep_addr;
        ih[5] = 0; // bmInfo
        ih[6] = OUTPUT_TERMINAL_ID; // bTerminalLink
        ih[7] = 0; // bStillCaptureMethod
        ih[8] = 0; // bTriggerSupport
        ih[9] = 0; // bTriggerUsage
        ih[10] = 1; // bControlSize
        ih[11] = 0; // bmaControls
        w.write(CS_INTERFACE, &ih)?;

        let mut fmt = [0u8; 25];
        fmt[0] = VS_FORMAT_UNCOMPRESSED;
        fmt[1] = 1; // bFormatIndex
        fmt[2] = UVC_MODES.len() as u8; // bNumFrameDescriptors
        fmt[3..19].copy_from_slice(&UYVY_GUID);
        fmt[19] = 16; // bBitsPerPixel
        fmt[20] = 1; // bDefaultFrameIndex
        // 21..25: aspect ratio X/Y, interlace flags, copy protect: zero
        w.write(CS_INTERFACE, &fmt)?;

        for (i, mode) in UVC_MODES.iter().enumerate() {
            let bit_rate: u32 = (mode.frame_bytes() as u64 * 8 * 10_000_000 / mode.interval as u64) as u32;
            let mut frm = [0u8; 28];
            frm[0] = VS_FRAME_UNCOMPRESSED;
            frm[1] = (i + 1) as u8; // bFrameIndex
            frm[2] = 0; // bmCapabilities
            frm[3..5].copy_from_slice(&(mode.width as u16).to_le_bytes());
            frm[5..7].copy_from_slice(&(mode.height as u16).to_le_bytes());
            frm[7..11].copy_from_slice(&bit_rate.to_le_bytes()); // dwMinBitRate
            frm[11..15].copy_from_slice(&bit_rate.to_le_bytes()); // dwMaxBitRate
            frm[15..19].copy_from_slice(&(mode.frame_bytes() as u32).to_le_bytes()); // dwMaxVideoFrameBufferSize
            frm[19..23].copy_from_slice(&mode.interval.to_le_bytes()); // dwDefaultFrameInterval
            frm[23] = 1; // bFrameIntervalType: one discrete interval
            frm[24..28].copy_from_slice(&mode.interval.to_le_bytes());
            w.write(CS_INTERFACE, &frm)?;
        }

        w.endpoint(&self.ep_in)?;
        Ok(())
    }

    fn reset(&mut self) { self.reset_state(); }

    /// Class-specific OUT requests with a data stage never reach `control_out` on this core: the
    /// driver completes their data and status stages in hardware and records the SETUP packet.
    /// Pick that up here; `poll` runs on every USB event, so this follows the SETUP closely.
    fn poll(&mut self) {
        let setup = match self.hw.try_lock() {
            Ok(mut hw) => hw.last_class_out_setup.take(),
            Err(_) => None,
        };
        let Some(s) = setup else {
            return;
        };
        let vs_if: u8 = self.vs_if.into();
        // bmRequestType, bRequest, wValue lo/hi, wIndex lo/hi, wLength
        if s[0] != 0x21 || s[1] != SET_CUR || s[4] != vs_if {
            return;
        }
        match s[3] {
            VS_PROBE_CONTROL => {
                // the proposal lands in the EP0 buffer shortly after; read it on the next GET
                self.probe_pending = true;
            }
            VS_COMMIT_CONTROL => {
                // COMMIT carries what the host last read back from PROBE
                self.adopt_probe_proposal();
                self.dbg.commits.fetch_add(1, Ordering::SeqCst);
                self.start_stream();
            }
            _ => {}
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = *xfer.request();
        if req.request_type == RequestType::Vendor && req.recipient == Recipient::Device {
            let len = req.length as usize;
            match req.request {
                VENDOR_REQ_DEBUG if req.value == 1 => {
                    let data = self.dbg.extended_report(self.dump.generation());
                    xfer.accept_with(&data[..len.min(data.len())]).ok();
                    return;
                }
                VENDOR_REQ_DEBUG => {
                    let data = self.debug_report();
                    xfer.accept_with(&data[..len.min(data.len())]).ok();
                    return;
                }
                VENDOR_REQ_DUMP => {
                    let mut out = [0u8; DUMP_REPLY];
                    let n = if req.value == DUMP_REQUEST {
                        self.dump.requested.store(true, Ordering::SeqCst);
                        out[..4].copy_from_slice(&self.dump.generation().to_le_bytes());
                        4
                    } else {
                        self.dump.read(req.value as usize, req.index as usize, &mut out)
                    };
                    xfer.accept_with(&out[..n.min(len)]).ok();
                    return;
                }
                VENDOR_REQ_INJECT => {
                    let taken = match req.value {
                        INJECT_STALL | INJECT_STALL_QUIET => xous::try_send_message(
                            self.conn,
                            xous::Message::new_scalar(
                                Opcode::IrqDebugStall.to_usize().unwrap(),
                                req.index as usize,
                                (req.value == INJECT_STALL_QUIET) as usize,
                                0,
                                0,
                            ),
                        )
                        .is_ok(),
                        INJECT_FLOOD => {
                            self.dbg.flood_lines.store(req.index as u32, Ordering::SeqCst);
                            true
                        }
                        _ => false,
                    };
                    if !taken {
                        self.dbg.drop_inject.fetch_add(1, Ordering::SeqCst);
                    }
                    xfer.accept_with(&[taken as u8][..len.min(1)]).ok();
                    return;
                }
                _ => {}
            }
        }
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return;
        }
        let vs_if: u8 = self.vs_if.into();
        if (req.index & 0xff) as u8 != vs_if {
            // requests to the VideoControl interface are not supported: we declare no controls
            return;
        }
        let selector = (req.value >> 8) as u8;
        let len = req.length as usize;
        match (selector, req.request) {
            (VS_PROBE_CONTROL | VS_COMMIT_CONTROL, GET_CUR | GET_MIN | GET_MAX | GET_DEF) => {
                if req.request == GET_CUR {
                    self.adopt_probe_proposal();
                }
                let data = self.probe_data();
                xfer.accept_with(&data[..len.min(data.len())]).ok();
            }
            (VS_PROBE_CONTROL | VS_COMMIT_CONTROL, GET_RES) => {
                let data = [0u8; PROBE_LEN];
                xfer.accept_with(&data[..len.min(data.len())]).ok();
            }
            (VS_PROBE_CONTROL | VS_COMMIT_CONTROL, GET_LEN) => {
                let data = (PROBE_LEN as u16).to_le_bytes();
                xfer.accept_with(&data[..len.min(data.len())]).ok();
            }
            (VS_PROBE_CONTROL | VS_COMMIT_CONTROL, GET_INFO) => {
                // supports GET and SET
                xfer.accept_with(&[0x03][..len.min(1)]).ok();
            }
            _ => {
                xfer.reject().ok();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = *xfer.request();
        match req.request_type {
            RequestType::Class => {
                if req.recipient != Recipient::Interface {
                    return;
                }
                let vs_if: u8 = self.vs_if.into();
                if (req.index & 0xff) as u8 != vs_if {
                    return;
                }
                // Not reached on this core for requests with a data stage (see `poll`); kept for
                // completeness should the driver ever deliver them.
                let selector = (req.value >> 8) as u8;
                match (selector, req.request) {
                    (VS_PROBE_CONTROL, SET_CUR) => {
                        xfer.accept().ok();
                    }
                    (VS_COMMIT_CONTROL, SET_CUR) => {
                        self.dbg.commits.fetch_add(1, Ordering::SeqCst);
                        self.start_stream();
                        xfer.accept().ok();
                    }
                    _ => {
                        xfer.reject().ok();
                    }
                }
            }
            RequestType::Standard => {
                // Hosts stop a bulk video stream with CLEAR_FEATURE(ENDPOINT_HALT) on the streaming
                // endpoint. Observe it, but leave the request for the standard handler to complete.
                let ep_addr: u8 = self.ep_in.address().into();
                if req.recipient == Recipient::Endpoint
                    && req.request == Request::CLEAR_FEATURE
                    && req.value == Request::FEATURE_ENDPOINT_HALT
                    && (req.index as u8) == ep_addr
                {
                    self.stop_stream();
                }
            }
            _ => {}
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr != self.ep_in.address() {
            return;
        }
        self.in_flight = false;
        self.dbg.completions.fetch_add(1, Ordering::SeqCst);
        if let Ok(mut hw) = self.hw.try_lock() {
            let pei = CorigineUsb::pei(UVC_EP_NUM as u8, CRG_IN);
            let _ = hw.app_ptr[pei - 2].take();
        }
        if self.discard_completion {
            self.discard_completion = false;
            // a new chunk may already be waiting
            self.kick();
            return;
        }
        if !self.frame_busy() {
            return;
        }
        self.cursor += 1;
        if self.cursor < self.chunk.payloads {
            self.enqueue_payload();
        } else {
            self.cursor = 0;
            self.chunks_sent.fetch_add(1, Ordering::SeqCst);
            if self.chunk.eof {
                self.frames_sent.fetch_add(1, Ordering::SeqCst);
            }
            self.frame_active.store(false, Ordering::SeqCst);
            self.notify_chunk_done();
        }
    }
}

/// Copy a chunk of raw UYVY image data into the staging buffer, writing a payload header in
/// front of every slot. Returns (payloads, payload_len, last_len) for `UvcClass::set_chunk`.
/// `fid` is the frame ID bit for this frame; `eof` marks the chunk that ends the frame.
///
/// The staging buffer is IFRAM (uncached, word-wide bus); `copy_from_slice` on aligned slices
/// compiles to a word copy, which is what we want.
pub fn stage_chunk(
    staging: &mut [u8],
    data: &[u8],
    payload_data: usize,
    fid: u8,
    eof: bool,
) -> (usize, usize, usize) {
    debug_assert!(staging.len() >= STAGING_BYTES);
    let payload_data = payload_data.clamp(1, UVC_MAX_PAYLOAD_DATA);
    let payloads = ((data.len() + payload_data - 1) / payload_data).clamp(1, UVC_CHUNK_PAYLOADS);
    let mut last_len = PAYLOAD_HDR;
    for i in 0..payloads {
        let start = i * payload_data;
        let end = (start + payload_data).min(data.len());
        let n = end - start;
        let slot = &mut staging[i * PAYLOAD_STRIDE..i * PAYLOAD_STRIDE + PAYLOAD_HDR + n];
        let last = i == payloads - 1;
        slot[0] = PAYLOAD_HDR as u8;
        slot[1] = HDR_EOH | (fid & HDR_FID) | if last && eof { HDR_EOF } else { 0 };
        slot[PAYLOAD_HDR..].copy_from_slice(&data[start..end]);
        if last {
            last_len = PAYLOAD_HDR + n;
        }
    }
    (payloads, PAYLOAD_HDR + payload_data, last_len)
}
