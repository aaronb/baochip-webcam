mod api;
mod debug;
#[cfg(target_os = "xous")]
mod hw;
#[cfg(not(target_os = "xous"))]
mod main_hosted;
mod mappings;
#[cfg(all(target_os = "xous", feature = "uvc"))]
mod uvc;

#[derive(num_derive::FromPrimitive, num_derive::ToPrimitive, Debug)]
#[cfg(target_os = "xous")]
enum TimeoutOp {
    Pump,
    InvalidCall,
    Quit,
}

#[derive(Debug)]
#[cfg(target_os = "xous")]
enum SerialListenMode {
    // this just causes data incoming to be printed to the debug log; it is the default
    NoListener,
    // this assumes there will be a CR/LF character to delimit lines (the `char` arg), and
    // will buffer data until two conditions are met: 1) a listener is hooked and 2) a CR/LF is received.
    // This will "infinitely" buffer incoming characters if no listener is hooked.
    AsciiListener(Option<char>),
    // this will simply buffer the data until the `usize` argument is met and passes it back to
    // hooked listener. If this mode is set and there is no listener, it will buffer data "indefinitely"
    // (e.g. until local heap is exhausted and the system panics)
    BinaryListener,
    // this will take any serial input and pass it on as if one was typing at the console
    ConsoleListener,
}

/// Evaluate `$body` with the main loop's debug phase (`uvc::PHASE_*`) set to `$phase`, so a hang
/// inside it shows up in `uvc_debug.py`. Without `uvc` there are no debug counters.
#[cfg(all(target_os = "xous", feature = "uvc"))]
macro_rules! phase {
    ($dbg:expr, $phase:expr, $body:expr) => {{
        let previous = $dbg.phase.swap($phase, core::sync::atomic::Ordering::SeqCst);
        let result = $body;
        $dbg.phase.store(previous, core::sync::atomic::Ordering::SeqCst);
        result
    }};
}
#[cfg(all(target_os = "xous", not(feature = "uvc")))]
macro_rules! phase {
    ($dbg:expr, $phase:expr, $body:expr) => {{ $body }};
}

fn main() -> ! {
    #[cfg(target_os = "xous")]
    main_hw();
    #[cfg(not(target_os = "xous"))]
    main_hosted::main_hosted();
}

#[cfg(target_os = "xous")]
pub(crate) fn main_hw() -> ! {
    use core::convert::TryFrom;
    use core::num::NonZeroU8;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    use std::collections::VecDeque;
    // Install a local panic handler
    #[cfg(feature = "debug-print-usb")]
    use std::panic;
    use std::sync::Arc;

    use api::*;
    use bao1x_api::IoGpio;
    use bao1x_api::keyboard::KeyMap;
    #[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
    use bao1x_hal::axp2101::VbusIrq;
    use bao1x_hal::usb::driver::{CorigineUsb, CorigineWrapper};
    use hw::{Bao1xUsb, UsbIrqReq};
    use num_traits::*;
    use packed_struct::PackedStructSlice;
    use usb_device::class_prelude::*;
    use utralib::{AtomicCsr, utra};
    use xous::msg_scalar_unpack;
    use xous_ipc::Buffer;
    #[cfg(not(feature = "uvc"))]
    use xous_usb_hid::device::fido::RawFidoReport;
    use xous_usb_hid::page::Keyboard;

    #[cfg(feature = "usbd-debug")]
    bao1x_hal::claim_duart();

    #[cfg(feature = "debug-print-usb")]
    panic::set_hook(Box::new(|info| {
        crate::println!("{}", info);
    }));

    log_server::init_wait().unwrap();
    log::set_max_level(log::LevelFilter::Info);
    log::info!("my PID is {}", xous::process::id());

    let xns = xous_names::XousNames::new().unwrap();
    let usbdev_sid = xns.register_name(api::SERVER_NAME_USB_DEVICE, None).expect("can't register server");
    let cid = xous::connect(usbdev_sid).expect("couldn't create suspend callback connection");
    log::trace!("registered with NS -- {:?}", usbdev_sid);
    let tt = ticktimer::Ticktimer::new().unwrap();

    let serial_number = std::env::vars()
        .find(|(key, _value)| key == "PUBLIC_SERIAL")
        .map(|(_key, value)| value)
        .expect("Missing PUBLIC_SERIAL in environment");

    let native_kbd = bao1x_api::keyboard::Keyboard::new(&xns).expect("couldn't connect to keyboard service");

    // IFRAM staging buffer for UVC payloads. The USB core can only DMA out of IFRAM. This has to
    // live for the lifetime of the process, so it is bound here in `main`'s scope.
    // If the allocation fails, USB still comes up; the video function just refuses frames.
    #[cfg(feature = "uvc")]
    let mut uvc_staging = unsafe { bao1x_hal::ifram::IframRange::request(crate::uvc::STAGING_BYTES, None) };
    #[cfg(feature = "uvc")]
    let uvc_phys = match uvc_staging.as_ref() {
        Some(range) => {
            let phys = range.phys_range.as_ptr() as usize;
            log::info!("UVC staging buffer at {:x}, {} bytes", phys, crate::uvc::STAGING_BYTES);
            phys
        }
        None => {
            log::error!("couldn't allocate IFRAM staging buffer for UVC; video streaming disabled");
            0
        }
    };
    #[cfg(not(feature = "uvc"))]
    let uvc_phys = 0usize;

    let usb_mapping = xous::syscall::map_memory(
        xous::MemoryAddress::new(bao1x_hal::usb::utra::CORIGINE_USB_BASE),
        None,
        bao1x_hal::usb::utra::CORIGINE_USB_LEN,
        xous::MemoryFlags::R | xous::MemoryFlags::W,
    )
    .expect("couldn't reserve register pages");
    let ifram_range = xous::syscall::map_memory(
        xous::MemoryAddress::new(bao1x_hal::usb::driver::CRG_UDC_MEMBASE),
        xous::MemoryAddress::new(bao1x_hal::usb::driver::CRG_UDC_MEMBASE), /* make P & V addresses
                                                                            * line up */
        bao1x_hal::usb::driver::CRG_IFRAM_PAGES * 0x1000,
        xous::MemoryFlags::R | xous::MemoryFlags::W,
    )
    .expect("couldn't allocate IFRAM pages");
    assert!(
        bao1x_hal::usb::driver::CRG_UDC_TOTAL_MEM_LEN <= bao1x_hal::usb::driver::CRG_IFRAM_PAGES * 0x1000
    );
    log::debug!(
        "total memory len: {:x}, allocated: {:x}",
        bao1x_hal::usb::driver::CRG_UDC_TOTAL_MEM_LEN,
        bao1x_hal::usb::driver::CRG_IFRAM_PAGES * 0x1000
    );
    let irq_range = xous::syscall::map_memory(
        xous::MemoryAddress::new(utra::irqarray1::HW_IRQARRAY1_BASE),
        None,
        0x1000,
        xous::MemoryFlags::R | xous::MemoryFlags::W,
    )
    .expect("couldn't allocate IRQ1 pages");
    let usb = AtomicCsr::new(usb_mapping.as_ptr() as *mut u32);
    let irq_csr = AtomicCsr::new(irq_range.as_ptr() as *mut u32);
    log::debug!("IRQ1 csr: {:x} -> {:x}", utra::irqarray1::HW_IRQARRAY1_BASE, unsafe {
        irq_csr.base() as usize
    });

    let mut corigine_usb =
        unsafe { CorigineUsb::new(ifram_range.as_ptr() as usize, usb.clone(), irq_csr.clone()) };
    corigine_usb.reset(); // initial reset of the core; we want some time to pass before doing the next items

    // safety: this is only safe because we will actually claim the IRQ after all the initializations are
    // done, and we promise not to enable interrupts until that time, either.
    unsafe {
        corigine_usb.irq_claimed();
        log::debug!("claimed irq");
    }
    let cw = CorigineWrapper::new(corigine_usb);
    let usb_alloc = UsbBusAllocator::new(cw.clone());

    // Notes:
    //  - Most drivers would `Box()` the hardware management structure to make sure the compiler doesn't move
    //    its location. However, we can't do this here because we are trying to maintain compatibility with
    //    another crate that implements the USB stack which can't handle Box'd structures.
    //  - It is safe to call `.init()` repeatedly because within `init()` we have an atomic bool that tracks
    //    if the interrupt handler has been hooked, and ignores further requests to hook it.
    let mut cu =
        Box::new(Bao1xUsb::new(usb.clone(), irq_csr.clone(), cid, cw, &usb_alloc, &serial_number, uvc_phys));
    cu.init();
    #[cfg(feature = "uvc")]
    let dbg = cu.uvc.dbg.clone();
    #[cfg(feature = "uvc")]
    crate::uvc::start_debug_threads(dbg.clone(), cu.uvc.dump.clone());

    // Serial driver variables
    let mut serial_listener: Option<xous::MessageEnvelope> = None;
    let mut serial_listen_mode: SerialListenMode = SerialListenMode::NoListener;
    let mut serial_buf = Vec::<u8>::new();
    let mut serial_rx_trigger = false; // when true, the condition was met to pass data to the listener (but the listener was not yet installed)

    // under the theory that PIDs cannot be forged.
    // also if someone commandeers a process, all bets are off within that process (this is a general
    // statement)
    let mut fido_listener_pid: Option<NonZeroU8> = None;
    let mut fido_listener: Option<xous::MessageEnvelope> = None;
    let mut fido_rx_queue: VecDeque<[u8; 64]> = VecDeque::new();

    let mut autotype_delay_ms = 30;

    // event observer connection
    let mut observer_conn: Option<xous::CID> = None;
    let mut observer_op: Option<usize> = None;

    // UVC state: who to tell about stream start/stop, a frame waiting for the staging buffer to
    // free up, and the frame ID bit that alternates between frames.
    #[cfg(feature = "uvc")]
    let mut uvc_observer: Option<(xous::CID, usize)> = None;
    #[cfg(feature = "uvc")]
    let mut uvc_pending: Option<xous::MessageEnvelope> = None;
    #[cfg(feature = "uvc")]
    let mut uvc_fid: u8 = 0;

    // manage FIDO Rx timeouts -- not tested yet
    let to_server = xous::create_server().unwrap();
    let to_conn = xous::connect(to_server).unwrap();
    // we don't have AtomicU64 on this platform, so we suffer from a rollover condition in timeouts once every
    // 46 days this manifests as a timeout that happens to be scheduled on the rollover being rounded to a
    // max limit timeout of 5 seconds, and/or an immediate timeout happening during the 5 seconds before
    // the 46-day limit
    let target_time_lsb = Arc::new(AtomicU32::new(0));
    let to_run = Arc::new(AtomicBool::new(false));
    const MAX_TIMEOUT_LIMIT_MS: u32 = 5000;
    const POLL_INTERVAL_MS: u64 = 50;
    std::thread::spawn({
        let cid = cid;
        let to_conn = to_conn;
        let target_time_lsb = target_time_lsb.clone();
        let to_run = to_run.clone();
        move || {
            let tt = ticktimer::Ticktimer::new().unwrap();
            let mut msg_opt = None;
            let mut return_type = 0;
            let mut next_wake = tt.elapsed_ms();
            loop {
                xous::reply_and_receive_next_legacy(to_server, &mut msg_opt, &mut return_type).unwrap();
                let msg = msg_opt.as_mut().unwrap();
                // loop only consumes CPU time when a timeout is active. Once it has timed out,
                // it will wait for a new pump call.
                let now = tt.elapsed_ms();
                let opcode =
                    num_traits::FromPrimitive::from_usize(msg.body.id()).unwrap_or(TimeoutOp::InvalidCall);
                log::debug!("Timeout thread: {:?}", opcode);
                match opcode {
                    TimeoutOp::Pump => {
                        if to_run.load(Ordering::SeqCst) {
                            let tt_lsb = target_time_lsb.load(Ordering::SeqCst);
                            if tt_lsb >= (now as u32) || (now as u32) - tt_lsb > MAX_TIMEOUT_LIMIT_MS
                            // limits rollover case
                            {
                                xous::try_send_message(
                                    cid,
                                    xous::Message::new_scalar(
                                        Opcode::U2fRxTimeout.to_usize().unwrap(),
                                        0,
                                        0,
                                        0,
                                        0,
                                    ),
                                )
                                .ok();
                                // no need to set to_run to `false` because a Pump message isn't initiated;
                                // the loop de-facto stops from a lack of new Pump messages
                            } else {
                                if next_wake <= now {
                                    next_wake = now + POLL_INTERVAL_MS;
                                    tt.sleep_ms(POLL_INTERVAL_MS as usize).ok();
                                    xous::try_send_message(
                                        to_conn,
                                        xous::Message::new_scalar(
                                            TimeoutOp::Pump.to_usize().unwrap(),
                                            0,
                                            0,
                                            0,
                                            0,
                                        ),
                                    )
                                    .ok();
                                } else {
                                    // don't issue more wakeups if we already have a wakeup scheduled
                                }
                            }
                        }
                    }
                    TimeoutOp::Quit => {
                        if let Some(scalar) = msg.body.scalar_message_mut() {
                            scalar.id = 0;
                            scalar.arg1 = 1;
                            break;
                        }
                    }
                    TimeoutOp::InvalidCall => {
                        log::error!(
                            "Unknown opcode received in FIDO Rx timeout handler: {:?}",
                            msg.body.id()
                        );
                    }
                }
            }
            xous::destroy_server(to_server).unwrap();
        }
    });

    let iox = bao1x_api::IoxHal::new();
    #[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
    let mut i2c = bao1x_hal::i2c::I2c::new();
    #[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
    let pmic = {
        log::info!("Registering PMIC handler to detect USB plug/unplug events");
        bao1x_hal::board::setup_pmic_irq(
            &iox,
            api::SERVER_NAME_USB_DEVICE,
            Opcode::PmicIrq.to_usize().unwrap(),
        );
        let mut pmic = bao1x_hal::axp2101::Axp2101::new(&mut i2c).expect("couldn't open PMIC");
        pmic.setup_vbus_irq(&mut i2c, bao1x_hal::axp2101::VbusIrq::Remove).expect("couldn't setup IRQ");
        pmic
    };

    let (se0_port, se0_pin) = bao1x_hal::board::setup_usb_pins(&iox);
    iox.set_gpio_pin_dir(se0_port, se0_pin, bao1x_api::IoxDir::Input); // release SE0 state, allowing for enumeration
    // NOTE: if SE0 is required, the KPC has to be un-configured to allow the SE0 I/O to actually be driven

    let mut key_map = KeyMap::Qwerty;

    log::debug!("Entering main loop");

    let mut msg_opt = None;
    loop {
        #[cfg(feature = "uvc")]
        dbg.phase.store(crate::uvc::PHASE_IDLE, Ordering::SeqCst);
        xous::reply_and_receive_next(usbdev_sid, &mut msg_opt).expect("Error fetching next message");
        let msg = msg_opt.as_mut().unwrap();
        let opcode = num_traits::FromPrimitive::from_usize(msg.body.id()).unwrap_or(Opcode::InvalidCall);
        log::debug!("{:?}", opcode);
        #[cfg(feature = "uvc")]
        {
            dbg.phase.store(crate::uvc::PHASE_HANDLING, Ordering::SeqCst);
            dbg.main_ticks.fetch_add(1, Ordering::SeqCst);
            dbg.record_opcode(msg.body.id() as u32);
            dbg.listen_mode.store(
                match serial_listen_mode {
                    SerialListenMode::NoListener => 0,
                    SerialListenMode::AsciiListener(_) => 1,
                    SerialListenMode::BinaryListener => 2,
                    SerialListenMode::ConsoleListener => 3,
                },
                Ordering::SeqCst,
            );
        }
        // Before the message: the parked chunk goes first, and a `UvcSendFrame` arriving now
        // then finds the staging buffer busy and takes its place in `uvc_pending`.
        #[cfg(feature = "uvc")]
        uvc_resume_pending(&mut uvc_pending, &mut cu, uvc_staging.as_mut(), &mut uvc_fid, &dbg);
        if cu.double_lock_detected() {
            // UVC builds count it for `uvc_debug.py` rather than log from whatever message it
            // shows up on, which is often a burst of mirrored log lines
            #[cfg(feature = "uvc")]
            dbg.double_locks.fetch_add(1, Ordering::SeqCst);
            #[cfg(not(feature = "uvc"))]
            log::warn!(
                "Double lock error detected in USB stack. Meditations: services/usb-bao1x/src/hw.rs@226 (composite_handler inner loop) and consider adding more IRQ enable/disable similar to libs/bao1x-hal/src/usb/driver.rs@2549 (write impl)"
            );
        }
        match opcode {
            #[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
            Opcode::PmicIrq => match pmic.get_vbus_irq_status(&mut i2c).unwrap() {
                VbusIrq::Insert => {
                    log::error!("VBUS insert reported by PMIC, but we didn't ask for the event!");
                }
                VbusIrq::Remove => {
                    log::info!("VBUS removed. Resetting stack.");
                    cu.unplug();
                }
                VbusIrq::InsertAndRemove => {
                    panic!("Unexpected report from vbus_irq status");
                }
                VbusIrq::None => {
                    // log::warn!("Received an interrupt but no actual event reported");
                }
            },
            #[cfg(all(feature = "board-baosec", feature = "oem-baosec-lite"))]
            Opcode::PmicIrq => {
                use bao1x_hal::axp2101::VbusIrq;
                if let Some(scalar) = msg.body.scalar_message_mut() {
                    let vbus_irq = VbusIrq::from(scalar.arg1);
                    if vbus_irq == VbusIrq::Remove {
                        cu.unplug();
                    }
                }
            }
            Opcode::U2fRxDeferred => {
                // notify the event listener, if any
                if observer_conn.is_some() && observer_op.is_some() {
                    xous::try_send_message(
                        observer_conn.unwrap(),
                        xous::Message::new_scalar(observer_op.unwrap(), 0, 0, 0, 0),
                    )
                    .ok();
                }

                if fido_listener_pid.is_none() {
                    fido_listener_pid = msg.sender.pid();
                }
                if fido_listener.is_some() {
                    log::error!(
                        "Double-listener request detected. There should only ever by one registered listener at a time."
                    );
                    log::error!(
                        "This will cause an upstream server to misbehave, but not panicing so the problem can be debugged."
                    );
                    // the receiver will get a response with the `code` field still in the `RxWait` state to
                    // indicate the problem
                }
                if fido_listener_pid == msg.sender.pid() {
                    // preferentially pull from the rx queue if it has elements
                    if let Some(data) = fido_rx_queue.pop_front() {
                        log::debug!(
                            "no deferral: ret queued data: {:?} queue len: {}",
                            &data[..8],
                            fido_rx_queue.len() + 1
                        );
                        let mut response = unsafe {
                            Buffer::from_memory_message_mut(msg.body.memory_message_mut().unwrap())
                        };
                        let mut buf = response.to_original::<U2fMsgIpc, _>().unwrap();
                        assert_eq!(buf.code, U2fCode::RxWait, "Expected U2fcode::RxWait in wrapper");
                        buf.data.copy_from_slice(&data);
                        buf.code = U2fCode::RxAck;
                        response.replace(buf).unwrap();
                    } else {
                        log::trace!("registering deferred listener");
                        {
                            // not tested
                            let spec = unsafe {
                                Buffer::from_memory_message_mut(msg.body.memory_message_mut().unwrap())
                            };
                            if let Some(to) = spec.to_original::<U2fMsgIpc, _>().unwrap().timeout_ms {
                                let target = tt.elapsed_ms() + to;
                                target_time_lsb.store(target as u32, Ordering::SeqCst); // this will keep updating the target time later and later
                                // run must always be set *after* target time is updated, because there is
                                // always a chance we timed out and checked
                                // target time between these two steps
                                to_run.store(true, Ordering::SeqCst);
                                xous::try_send_message(
                                    to_conn,
                                    xous::Message::new_scalar(
                                        TimeoutOp::Pump.to_usize().unwrap(),
                                        0,
                                        0,
                                        0,
                                        0,
                                    ),
                                )
                                .ok();
                            }
                        };
                        fido_listener = msg_opt.take();
                    }
                } else {
                    log::warn!(
                        "U2F interface capability is locked on first use; additional servers are ignored: {:?}",
                        msg.sender
                    );
                    let mut buffer =
                        unsafe { Buffer::from_memory_message_mut(msg.body.memory_message_mut().unwrap()) };
                    let mut u2f_ipc = buffer.to_original::<U2fMsgIpc, _>().unwrap();
                    u2f_ipc.code = U2fCode::Denied;
                    buffer.replace(u2f_ipc).unwrap();
                }
            }
            // Note: not tested
            Opcode::U2fRxTimeout => {
                // put the lock in its own scope so we release it as soon as we've taken the message
                let maybe_listener = { fido_listener.take() };
                if let Some(mut listener) = maybe_listener {
                    let mut response = unsafe {
                        Buffer::from_memory_message_mut(listener.body.memory_message_mut().unwrap())
                    };
                    let mut buf = response.to_original::<U2fMsgIpc, _>().unwrap();
                    assert_eq!(buf.code, U2fCode::RxWait, "Expected U2fcode::RxWait in wrapper");
                    buf.code = U2fCode::RxTimeout;
                    response.replace(buf).unwrap();
                }
            }
            Opcode::IrqFidoRx => {
                if let Some(raw_report) = cu.hid_packet.pop_front() {
                    let u2f_report = HIDReport(raw_report);
                    if let Some(mut listener) = fido_listener.take() {
                        let mut response = unsafe {
                            Buffer::from_memory_message_mut(listener.body.memory_message_mut().unwrap())
                        };
                        let mut deferred_buf = response.to_original::<U2fMsgIpc, _>().unwrap();

                        deferred_buf.data.copy_from_slice(&u2f_report.0);
                        log::trace!("ret deferred data {:x?}", &u2f_report.0[..8]);
                        deferred_buf.code = U2fCode::RxAck;
                        response.replace(deferred_buf).unwrap();
                    } else {
                        crate::println!("Got U2F packet, but no server to respond...queuing.");
                        fido_rx_queue.push_back(u2f_report.0);
                    }
                } else {
                    // I *think* this is harmless, can remove this later on if protocol is robust
                    log::warn!("got IrqFidoRx but no data");
                }
            }
            Opcode::U2fTx => {
                if fido_listener_pid.is_none() {
                    fido_listener_pid = msg.sender.pid();
                }
                let mut buffer =
                    unsafe { Buffer::from_memory_message_mut(msg.body.memory_message_mut().unwrap()) };
                let mut u2f_ipc = buffer.to_original::<U2fMsgIpc, _>().unwrap();
                if cu.device.state() != usb_device::device::UsbDeviceState::Configured {
                    log::warn!("U2fTx: HANGUP");
                    u2f_ipc.code = U2fCode::Hangup;
                    buffer.replace(u2f_ipc).unwrap();
                    continue;
                }
                #[cfg(feature = "uvc")]
                {
                    // the FIDO transport is not part of the composite in UVC builds
                    u2f_ipc.code = U2fCode::Hangup;
                }
                #[cfg(not(feature = "uvc"))]
                if fido_listener_pid == msg.sender.pid() {
                    let mut u2f_msg = RawFidoReport::default();
                    assert_eq!(u2f_ipc.code, U2fCode::Tx, "Expected U2fCode::Tx in wrapper");
                    u2f_msg.packet.copy_from_slice(&u2f_ipc.data);
                    {
                        // scope this so the borrow is released before the IRQ is tripped
                        cu.fido_tx_queue.borrow_mut().push_back(u2f_msg);
                    }
                    cu.sw_irq(UsbIrqReq::FidoTx);
                    // ensure that the interrupt has happened, because the interrupt can create
                    // concurrency issues on the fido_tx_queue if we've re-entered this function
                    // before the interrupt has processed
                    while !cu.irq_serviced.load(Ordering::SeqCst) {
                        xous::yield_slice();
                    }
                    // clear the serviced flag here - this is OK because this is the only other thread
                    // that can access the variable.
                    cu.irq_serviced.store(false, Ordering::SeqCst);
                    log::debug!("enqueued U2F packet {:x?}", u2f_ipc.data);
                    u2f_ipc.code = U2fCode::TxAck;
                } else {
                    u2f_ipc.code = U2fCode::Denied;
                }
                buffer.replace(u2f_ipc).unwrap();
            }
            Opcode::SendKeyCode => {
                if let Some(scalar) = msg.body.scalar_message_mut() {
                    if cu.device.state() != usb_device::device::UsbDeviceState::Configured {
                        continue;
                    }

                    let code0 = scalar.arg1;
                    let code1 = scalar.arg2;
                    let code2 = scalar.arg3;
                    let autoup = scalar.arg4;
                    if code0 != 0 {
                        cu.kbd_tx_queue.borrow_mut().push_back(match key_map {
                            KeyMap::Dvorak => mappings::char_to_hid_code_dvorak(code0 as u8 as char)[0],
                            _ => mappings::char_to_hid_code_us101(code0 as u8 as char)[0],
                        });
                    }
                    if code1 != 0 {
                        cu.kbd_tx_queue.borrow_mut().push_back(match key_map {
                            KeyMap::Dvorak => mappings::char_to_hid_code_dvorak(code1 as u8 as char)[0],
                            _ => mappings::char_to_hid_code_us101(code1 as u8 as char)[0],
                        });
                    }
                    if code2 != 0 {
                        cu.kbd_tx_queue.borrow_mut().push_back(match key_map {
                            KeyMap::Dvorak => mappings::char_to_hid_code_dvorak(code2 as u8 as char)[0],
                            _ => mappings::char_to_hid_code_us101(code2 as u8 as char)[0],
                        });
                    }
                    let auto_up = if autoup == 1 { true } else { false };
                    // kbd_tx_queue borrow_mut() should be out of scope before the IRQ is fired
                    cu.sw_irq(UsbIrqReq::KbdTx);
                    tt.sleep_ms(autotype_delay_ms).ok();
                    if auto_up {
                        {
                            // ensure borrow_mut() is scoped out before IRQ is fired
                            cu.kbd_tx_queue.borrow_mut().push_back(Keyboard::NoEventIndicated);
                        }
                        cu.sw_irq(UsbIrqReq::KbdTx);
                        tt.sleep_ms(autotype_delay_ms).ok();
                    }
                    scalar.arg1 = 0;
                }
            }
            Opcode::SendString => {
                let mut buffer =
                    unsafe { Buffer::from_memory_message_mut(msg.body.memory_message_mut().unwrap()) };
                let mut usb_send = buffer.to_original::<api::UsbString, _>().unwrap();
                let mut sent = 0;
                if cu.device.state() != usb_device::device::UsbDeviceState::Configured {
                    log::warn!("Aborting send, no USB configured");
                    usb_send.sent = Some(0);
                    buffer.replace(usb_send).unwrap();
                    continue;
                }

                // check keymap on every call because we may need to toggle this for e.g. plugging
                // into a new host with a different map
                for ch in usb_send.s.as_str().chars() {
                    // ASSUME: user's keyboard type matches the preference on their Precursor device.
                    let codes = match key_map {
                        KeyMap::Dvorak => mappings::char_to_hid_code_dvorak(ch),
                        _ => mappings::char_to_hid_code_us101(ch),
                    };
                    for code in codes {
                        cu.kbd_tx_queue.borrow_mut().push_back(code);
                    }
                    // key down
                    cu.sw_irq(UsbIrqReq::KbdTx);
                    tt.sleep_ms(autotype_delay_ms).ok();
                    // key up
                    {
                        // ensure that borrow_mut() is scoped out before IRQ is fired
                        cu.kbd_tx_queue.borrow_mut().push_back(Keyboard::NoEventIndicated);
                    }
                    cu.sw_irq(UsbIrqReq::KbdTx);
                    tt.sleep_ms(autotype_delay_ms).ok();

                    sent += 1;
                }
                usb_send.sent = Some(sent as _);
                buffer.replace(usb_send).unwrap();
            }
            Opcode::RegisterUsbObserver => {
                let buffer = unsafe { Buffer::from_memory_message(msg.body.memory_message().unwrap()) };
                let ur = buffer.as_flat::<UsbListenerRegistration, _>().unwrap();
                if observer_conn.is_none() {
                    match phase!(
                        dbg,
                        crate::uvc::PHASE_NAMES,
                        xns.request_connection_blocking(ur.server_name.as_str())
                    ) {
                        Ok(cid) => {
                            observer_conn = Some(cid);
                            observer_op = Some(<u32 as From<u32>>::from(ur.listener_op_id.into()) as usize);
                        }
                        Err(e) => {
                            log::error!("couldn't connect to observer: {:?}", e);
                            observer_conn = None;
                            observer_op = None;
                        }
                    }
                }
            }
            Opcode::SetAutotypeRate => msg_scalar_unpack!(msg, rate, _, _, _, {
                // limit rate to 0.5s delay. Even then, this will probably cause repeated characters because
                // it also adjusts keydown delays
                let checked_rate = if rate > 500 { 500 } else { rate };
                // there is no limit on the minimum rate. good luck if you set it to 0!
                autotype_delay_ms = checked_rate;
            }),
            Opcode::SetLogLevel => msg_scalar_unpack!(msg, level_code, _, _, _, {
                let level = LogLevel::try_from(level_code).unwrap_or(LogLevel::Info);
                match level {
                    LogLevel::Trace => log::set_max_level(log::LevelFilter::Trace),
                    LogLevel::Info => log::set_max_level(log::LevelFilter::Info),
                    LogLevel::Debug => log::set_max_level(log::LevelFilter::Debug),
                    LogLevel::Warn => log::set_max_level(log::LevelFilter::Warn),
                    LogLevel::Err => log::set_max_level(log::LevelFilter::Error),
                }
            }),
            Opcode::IrqSerialRx => {
                if let Some(scalar) = msg.body.scalar_message() {
                    let valid_bytes = scalar.arg1;
                    serial_buf.extend_from_slice(&cu.serial_rx[..valid_bytes]);
                    match serial_listen_mode {
                        SerialListenMode::NoListener => {
                            match std::str::from_utf8(&serial_buf) {
                                Ok(s) => log::info!("No listener ascii: {}", s),
                                Err(_) => {
                                    log::info!("No listener binary: {:x?}", &serial_buf);
                                }
                            }
                            serial_buf.clear();
                        }
                        SerialListenMode::ConsoleListener => {
                            match std::str::from_utf8(&serial_buf) {
                                Ok(s) => {
                                    // Don't wait on the keyboard server: it logs, and log lines
                                    // come back into this loop's queue through the USB console
                                    // mirror. Keys that don't fit are dropped with the rest of
                                    // the batch.
                                    let mut dropped = 0;
                                    for c in s.chars() {
                                        if dropped > 0 || native_kbd.try_inject_key(c).is_err() {
                                            dropped += 1;
                                        }
                                    }
                                    #[cfg(feature = "uvc")]
                                    dbg.drop_keys.fetch_add(dropped, Ordering::SeqCst);
                                    #[cfg(not(feature = "uvc"))]
                                    let _ = dropped;
                                }
                                Err(_) => {
                                    log::info!("Non UTF-8 received on console: {:x?}", &serial_buf);
                                }
                            }
                            serial_buf.clear();
                        }
                        SerialListenMode::AsciiListener(maybe_delimiter) => {
                            if let Some(delimiter) = maybe_delimiter {
                                if !delimiter.is_ascii() {
                                    log::warn!(
                                        "Chosen ASCII delimiter {} is not ASCII. Serial receive will not function properly.",
                                        delimiter
                                    );
                                }
                                if !serial_rx_trigger {
                                    // once true, sticks as true
                                    serial_rx_trigger =
                                        serial_buf.iter().find(|&&c| c == (delimiter as u8)).is_some();
                                }
                            } else {
                                serial_rx_trigger = true;
                            }
                            // now see if we should pass it back to the listener (if it is hooked)
                            if serial_rx_trigger && serial_listener.is_some() {
                                let mut rx_msg = serial_listener.take().unwrap();
                                let mut response = unsafe {
                                    Buffer::from_memory_message_mut(rx_msg.body.memory_message_mut().unwrap())
                                };
                                let mut buf = response.to_original::<UsbSerialAscii, _>().unwrap();
                                use std::fmt::Write; // is this really the best way to do it? probably not.
                                write!(buf.s, "{}", std::string::String::from_utf8_lossy(&serial_buf)).ok();

                                response.replace(buf).unwrap();
                                // the rx_msg will drop and respond to the listener
                                serial_rx_trigger = false;
                                serial_buf.clear();
                            }
                        }
                        SerialListenMode::BinaryListener => {
                            match serial_listener.take() {
                                Some(mut rx_msg) => {
                                    let mut response = unsafe {
                                        Buffer::from_memory_message_mut(
                                            rx_msg.body.memory_message_mut().unwrap(),
                                        )
                                    };
                                    let mut buf = response.to_original::<UsbSerialBinary, _>().unwrap();
                                    let n = serial_buf.len().min(SERIAL_BINARY_BUFLEN);
                                    let at_most_one_page: Vec<u8> = serial_buf.drain(..n).collect();
                                    buf.d.extend_from_slice(&at_most_one_page);
                                    response.replace(buf).unwrap();
                                    // the rx_msg will drop and respond to the listener
                                }
                                None => {
                                    // do nothing, keep queuing data...
                                }
                            }
                        }
                    }
                }
            }
            Opcode::SerialHookAscii => {
                let maybe_delimiter = {
                    let buffer = unsafe { Buffer::from_memory_message(msg.body.memory_message().unwrap()) };
                    let data = buffer.to_original::<UsbSerialAscii, _>().unwrap();
                    data.delimiter
                };
                serial_listen_mode = SerialListenMode::AsciiListener(maybe_delimiter);
                serial_listener = msg_opt.take();
            }
            Opcode::SerialHookBinary => {
                serial_listen_mode = SerialListenMode::BinaryListener;
                serial_listener = msg_opt.take();
            }
            Opcode::SerialHookConsole => msg_scalar_unpack!(msg, _, _, _, _, {
                let log_conn = xous::connect(xous::SID::from_bytes(b"xous-log-server ").unwrap()).unwrap();
                match phase!(
                    dbg,
                    crate::uvc::PHASE_LOG_HOOK,
                    xous::send_message(
                        log_conn,
                        xous::Message::new_blocking_scalar(
                            log_server::api::Opcode::TryHookUsbMirror.to_usize().unwrap(),
                            0,
                            0,
                            0,
                            0,
                        ),
                    )
                ) {
                    Ok(xous::Result::Scalar1(result)) => {
                        if result == 1 {
                            serial_listen_mode = SerialListenMode::ConsoleListener;
                            // unhook any previous pending listener
                            serial_listener.take();
                        } else {
                            log::error!("Error trying to connect USB console.");
                        }
                    }
                    _ => {
                        log::error!("Could not connect USB console");
                    }
                }
            }),
            Opcode::SerialClearHooks => {
                let log_conn = xous::connect(xous::SID::from_bytes(b"xous-log-server ").unwrap()).unwrap();
                // it is never harmful to double-unhook this
                phase!(
                    dbg,
                    crate::uvc::PHASE_LOG_HOOK,
                    xous::send_message(
                        log_conn,
                        xous::Message::new_blocking_scalar(
                            log_server::api::Opcode::UnhookUsbMirror.to_usize().unwrap(),
                            0,
                            0,
                            0,
                            0,
                        ),
                    )
                    .ok()
                );

                serial_listen_mode = SerialListenMode::NoListener;
                serial_listener.take();
            }
            Opcode::SerialFlush => msg_scalar_unpack!(msg, _, _, _, _, {
                if cu.device.state() != usb_device::device::UsbDeviceState::Configured {
                    continue;
                }
                // The interrupt handler may access the same usbd-serial state through
                // UsbDevice::poll(), so the flush must use the IRQ-safe wrapper.
                phase!(dbg, crate::uvc::PHASE_SERIAL_WRITE, cu.serial_flush_irq_safe().ok());
                // this tries to return any data that's pending within the main loop's buffers
                match serial_listen_mode {
                    SerialListenMode::BinaryListener => {
                        match serial_listener.take() {
                            Some(mut rx_msg) => {
                                let mut response = unsafe {
                                    Buffer::from_memory_message_mut(rx_msg.body.memory_message_mut().unwrap())
                                };
                                let mut buf = response.to_original::<UsbSerialBinary, _>().unwrap();
                                let chars_avail = serial_buf.len().min(SERIAL_BINARY_BUFLEN);
                                buf.d.copy_from_slice(serial_buf.drain(..chars_avail).as_slice());
                                response.replace(buf).unwrap();
                                // the rx_msg will drop and respond to the listener
                            }
                            None => {
                                // do nothing, keep queuing data...
                            }
                        }
                    }
                    SerialListenMode::AsciiListener(_) => {
                        match serial_listener.take() {
                            Some(mut rx_msg) => {
                                let mut response = unsafe {
                                    Buffer::from_memory_message_mut(rx_msg.body.memory_message_mut().unwrap())
                                };
                                let mut buf = response.to_original::<UsbSerialAscii, _>().unwrap();
                                use std::fmt::Write; // is this really the best way to do it? probably not.
                                write!(buf.s, "{}", std::string::String::from_utf8_lossy(&serial_buf)).ok();

                                response.replace(buf).unwrap();
                                // the rx_msg will drop and respond to the listener
                                serial_rx_trigger = false;
                            }
                            None => {} // do nothing
                        }
                    }
                    _ => {}
                }
            }),
            Opcode::SerialSendData => {
                if cu.device.state() != usb_device::device::UsbDeviceState::Configured {
                    continue;
                }
                let buffer = unsafe { Buffer::from_memory_message(msg.body.memory_message().unwrap()) };
                let data = buffer.to_original::<UsbSerialBinary, _>().unwrap();

                // This is a best-effort, non-blocking path. Stop at the first short
                // write or error because only a contiguous prefix can be accepted.
                for chunk in data.d.chunks(crate::hw::SERIAL_MAX_PACKET_SIZE) {
                    match phase!(dbg, crate::uvc::PHASE_SERIAL_WRITE, cu.serial_write_irq_safe(chunk)) {
                        Ok(sent) if sent == chunk.len() => {}
                        Ok(_) | Err(_) => break,
                    }
                }
            }
            Opcode::SerialSendDataBlocking => {
                let mut buffer =
                    unsafe { Buffer::from_memory_message_mut(msg.body.memory_message_mut().unwrap()) };

                let mut request = buffer.to_original::<UsbSerialSend, _>().unwrap();

                if cu.device.state() != usb_device::device::UsbDeviceState::Configured {
                    // The IPC request itself was handled successfully, but no bytes
                    // can be accepted while the USB device is not configured.
                    request.sent = Some(0);
                    buffer.replace(request).unwrap();
                    continue;
                }

                let mut total_sent = 0usize;

                // usbd-serial has a smaller internal transmit buffer than the IPC
                // payload, so submit the request in USB-sized chunks.
                //
                // Only a contiguous prefix is reported as accepted. Stop at the
                // first short write or error and let the caller retry the remainder.
                for chunk in request.d.chunks(crate::hw::SERIAL_MAX_PACKET_SIZE) {
                    match phase!(dbg, crate::uvc::PHASE_SERIAL_WRITE, cu.serial_write_irq_safe(chunk)) {
                        Ok(sent) => {
                            total_sent += sent;

                            if sent < chunk.len() {
                                break;
                            }
                        }
                        Err(_) => break,
                    }
                }

                request.sent = Some(total_sent as u32);
                buffer.replace(request).unwrap();
            }
            Opcode::LogString => {
                // the logger API is "best effort" only. Because retries and response codes can cause problems
                // in the logger API, if anything goes wrong, we prefer to discard characters rather than get
                // the whole subsystem stuck in some awful recursive error handling hell.
                if let Some(mem_msg) = msg.body.memory_message() {
                    let buffer = unsafe { Buffer::from_memory_message(mem_msg) };
                    match buffer.to_original::<api::UsbString, _>() {
                        Ok(usb_send) => {
                            for chunk in
                                usb_send.s.as_bytes().chunks(bao1x_hal::usb::driver::CRG_UDC_APP_BUFSIZE)
                            {
                                phase!(
                                    dbg,
                                    crate::uvc::PHASE_SERIAL_WRITE,
                                    cu.serial_write_irq_safe(&chunk).ok()
                                );
                            }
                        }
                        _ => {} // silent errors
                    }
                }
            }
            #[cfg(feature = "uvc")]
            Opcode::RegisterUvcObserver => {
                let buffer = unsafe { Buffer::from_memory_message(msg.body.memory_message().unwrap()) };
                let ur = buffer.as_flat::<UsbListenerRegistration, _>().unwrap();
                match phase!(
                    dbg,
                    crate::uvc::PHASE_NAMES,
                    xns.request_connection_blocking(ur.server_name.as_str())
                ) {
                    Ok(cid) => {
                        let op = <u32 as From<u32>>::from(ur.listener_op_id.into()) as usize;
                        log::info!("UVC observer registered: {} op {}", ur.server_name.as_str(), op);
                        uvc_observer = Some((cid, op));
                    }
                    Err(e) => {
                        log::error!("couldn't connect to UVC observer: {:?}", e);
                    }
                }
            }
            #[cfg(feature = "uvc")]
            Opcode::UvcSendFrame => {
                if !cu.uvc.is_streaming() || uvc_staging.is_none() {
                    if let Some(mem) = msg.body.memory_message_mut() {
                        mem.valid = xous::MemorySize::new(UVC_RESULT_NOT_STREAMING);
                    }
                } else if cu.uvc.frame_busy() {
                    // the previous frame is still going out; hold the caller until it completes
                    if uvc_pending.is_some() {
                        // counted, not logged: the stream paths of this loop don't log
                        dbg.refused_frames.fetch_add(1, Ordering::SeqCst);
                        if let Some(mem) = msg.body.memory_message_mut() {
                            mem.valid = xous::MemorySize::new(UVC_RESULT_NOT_STREAMING);
                        }
                    } else {
                        uvc_pending = msg_opt.take();
                    }
                } else if let Some(staging) = uvc_staging.as_mut() {
                    if let Some((n, len, last, eof)) =
                        uvc_stage_chunk(msg, staging.as_slice_mut::<u8>(), &mut uvc_fid, &dbg)
                    {
                        cu.uvc.set_chunk(n, len, last, eof);
                        cu.sw_irq(UsbIrqReq::UvcKick);
                    }
                }
            }
            #[cfg(feature = "uvc")]
            Opcode::IrqUvcFrameDone => {
                // the parked chunk, if any, was resumed before the message was dispatched
            }
            #[cfg(feature = "uvc")]
            Opcode::IrqUvcStreamChange => msg_scalar_unpack!(msg, state, mode, _, _, {
                // counted, not logged (bao-video logs the camera starting and stopping). A parked
                // chunk was already dealt with before dispatch: the state change abandoned any
                // frame in progress, so it was staged into the new stream or refused.
                dbg.stream_changes.fetch_add(1, Ordering::SeqCst);
                if let Some((cid, op)) = uvc_observer {
                    // arg2 = 0: from the USB service (not the console); arg3 = mode index
                    if xous::try_send_message(cid, xous::Message::new_scalar(op, state, 0, mode, 0)).is_err()
                    {
                        dbg.drop_observer.fetch_add(1, Ordering::SeqCst);
                    }
                }
            }),
            #[cfg(feature = "uvc")]
            Opcode::IrqDebugStall => msg_scalar_unpack!(msg, ms, quiet, _, _, {
                // `uvc_debug.py inject stall`: hold this loop so its queue fills up behind it, then
                // log a line (unless quiet), which is where a log server waiting on that queue
                // deadlocks with it
                phase!(dbg, crate::uvc::PHASE_SLEEP, tt.sleep_ms(ms).ok());
                if quiet == 0 {
                    phase!(dbg, crate::uvc::PHASE_LOG, log::info!("debug: main loop held for {} ms", ms));
                }
            }),
            Opcode::UsbBusReset => {
                phase!(dbg, crate::uvc::PHASE_LOG, log::warn!("USB bus reset requested"));
                #[cfg(feature = "uvc")]
                {
                    cu.uvc.reset_state();
                    // the stream is down now, so a parked chunk is refused
                    uvc_resume_pending(&mut uvc_pending, &mut cu, uvc_staging.as_mut(), &mut uvc_fid, &dbg);
                    if let Some((cid, op)) = uvc_observer {
                        if xous::try_send_message(cid, xous::Message::new_scalar(op, 0, 0, 0, 0)).is_err() {
                            dbg.drop_observer.fetch_add(1, Ordering::SeqCst);
                        }
                    }
                }
                phase!(
                    dbg,
                    crate::uvc::PHASE_BUS_RESET,
                    usb_device::bus::UsbBus::force_reset(cu.device.bus()).ok()
                );
            }
            Opcode::UvcStatus => {
                if let Some(scalar) = msg.body.scalar_message_mut() {
                    #[cfg(feature = "uvc")]
                    {
                        scalar.arg1 = if cu.uvc.is_streaming() { 1 } else { 0 };
                        scalar.arg2 = cu.uvc.frames_sent() as usize;
                    }
                    #[cfg(not(feature = "uvc"))]
                    {
                        scalar.arg1 = 0;
                        scalar.arg2 = 0;
                    }
                }
            }
            #[cfg(not(feature = "uvc"))]
            Opcode::UvcSendFrame => {
                if let Some(mem) = msg.body.memory_message_mut() {
                    // `None` reads back as `UvcFrameResult::Unsupported`
                    mem.valid = None;
                }
            }
            Opcode::LinkStatus => {
                if let Some(scalar) = msg.body.scalar_message_mut() {
                    // to get the raw device state:
                    // cu.device.bus().core().get_device_state()
                    scalar.arg1 = cu.device.state() as usize;
                }
            }
            Opcode::GetLedState => {
                if let Some(scalar) = msg.body.scalar_message_mut() {
                    let mut code = [0u8; 1];
                    cu.led_state.pack_to_slice(&mut code).unwrap();
                    scalar.arg1 = code[0] as usize;
                }
            }
            Opcode::SetKeyMap => {
                if let Some(scalar) = msg.body.scalar_message_mut() {
                    key_map = KeyMap::from(scalar.arg1);
                }
            }
            Opcode::InvalidCall => {
                log::warn!("Illegal opcode received {:?}", msg);
            }
            Opcode::Quit => {
                log::warn!("Quit received, goodbye world!");
                break;
            }
            _ => {
                log::warn!(
                    "Opcode {:?} not implemented for this version of the USB stack: {:?}",
                    opcode,
                    msg
                );
            }
        }
    }
    // clean up our program
    log::warn!("main loop exit, destroying servers");
    xns.unregister_server(usbdev_sid).unwrap();
    xous::destroy_server(usbdev_sid).unwrap();
    log::info!("quitting");
    xous::terminate_process(0)
}

/// Copy the chunk carried by a `UvcSendFrame` lend into the IFRAM staging buffer and set the
/// reply code. Returns the chunk description for `UvcClass::set_chunk` if a chunk was staged.
#[cfg(all(target_os = "xous", feature = "uvc"))]
fn uvc_stage_chunk(
    env: &mut xous::MessageEnvelope,
    staging: &mut [u8],
    fid: &mut u8,
    dbg: &crate::uvc::DebugCounters,
) -> Option<(usize, usize, usize, bool)> {
    let mem = env.body.memory_message_mut()?;
    let len = mem.valid.map(|v| v.get()).unwrap_or(0);
    let info = mem.offset.map(|v| v.get()).unwrap_or(0);
    let first = info & api::UVC_CHUNK_FIRST != 0;
    let last = info & api::UVC_CHUNK_LAST != 0;
    let payload_data = info >> 2;
    let data = unsafe { mem.buf.as_slice::<u8>() };
    if len == 0
        || len > data.len()
        || len > api::UVC_CHUNK_MAX_BYTES
        || payload_data == 0
        || payload_data > api::UVC_MAX_PAYLOAD_DATA
    {
        dbg.bad_chunks.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
        mem.valid = xous::MemorySize::new(api::UVC_RESULT_BAD_FRAME);
        return None;
    }
    if first {
        *fid ^= 1;
    }
    let (n, plen, last_len) = crate::uvc::stage_chunk(staging, &data[..len], payload_data, *fid, last);
    mem.valid = xous::MemorySize::new(api::UVC_RESULT_SENT);
    Some((n, plen, last_len, last))
}

/// Deal with a chunk parked in `pending` while the staging buffer was busy: stage it now if the
/// host is still streaming and the buffer is free, reply that the stream is gone if it stopped,
/// and leave it parked while its predecessor is still going out. Runs once per main loop
/// message rather than only on `IrqUvcFrameDone`: the IRQ handler sends that with
/// `try_send_message`, which fails when this server's queue is full (a burst of mirrored log
/// lines does it), and a lost notification left the parked lend unanswered, which blocked
/// bao-video, and every Gfx caller behind it, until the host closed the stream.
#[cfg(all(target_os = "xous", feature = "uvc"))]
fn uvc_resume_pending(
    pending: &mut Option<xous::MessageEnvelope>,
    cu: &mut hw::Bao1xUsb,
    staging: Option<&mut bao1x_hal::ifram::IframRange>,
    fid: &mut u8,
    dbg: &crate::uvc::DebugCounters,
) {
    if pending.is_none() || (cu.uvc.is_streaming() && cu.uvc.frame_busy()) {
        return;
    }
    let Some(mut env) = pending.take() else {
        return;
    };
    match staging {
        Some(staging) if cu.uvc.is_streaming() => {
            if let Some((n, len, last, eof)) =
                uvc_stage_chunk(&mut env, staging.as_slice_mut::<u8>(), fid, dbg)
            {
                cu.uvc.set_chunk(n, len, last, eof);
                cu.sw_irq(hw::UsbIrqReq::UvcKick);
            }
        }
        _ => {
            if let Some(mem) = env.body.memory_message_mut() {
                mem.valid = xous::MemorySize::new(api::UVC_RESULT_NOT_STREAMING);
            }
        }
    }
    // `env` drops here, which replies to the waiting frame source
}
