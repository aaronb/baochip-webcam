// Note: the log server relies on this name not changing in order to hook the serial port for logging output.
// changing this name shouldn't lead to a crash, but it will lead to the USB driver being undiscoverable by
// the log crate.
pub(crate) const SERVER_NAME_USB_DEVICE: &'static str = "_Xous USB device driver_";

#[derive(num_derive::FromPrimitive, num_derive::ToPrimitive, Debug)]
pub enum Opcode {
    /// Returns the link status
    LinkStatus = 0,
    /// Send a keyboard code
    SendKeyCode = 1,
    /// "Type" a string to the keyboard. This API is relied upon by the log crate.
    SendString = 2,
    /// Get the current LED state
    GetLedState = 3,
    /// Switch to a specified device core
    SwitchCores = 4,
    /// Makes sure a given core is selected
    EnsureCore = 5,
    /// Check which core is connected
    WhichCore = 6,
    /// Restrict the debug core
    RestrictDebugAccess = 7,
    /// Retrieve restriction state
    IsRestricted = 8,
    /// Set-and-check of USB debug restriction
    DebugUsbOp = 9,
    /// Set autotype rate
    SetAutotypeRate = 10,
    /// Register a USB event observer
    RegisterUsbObserver = 11,
    /// Modify log level
    SetLogLevel = 12,

    /// Set the host-side key mapping (this translates data into e.g. dvorak or qwerty maps)
    SetKeyMap = 16,

    /// Send a U2F message
    U2fTx = 128,
    /// Blocks the caller, waiting for a U2F message
    U2fRxDeferred = 129,
    /// A bump from the timeout process to check if U2fRx has timed out
    U2fRxTimeout = 130,

    /// Query if the HID driver was able to start
    IsSocCompatible = 256,

    /// Hook serial ASCII listener
    SerialHookAscii = 512,
    /// Hook serial binary listener
    SerialHookBinary = 513,
    /// Flush any serial buffers
    SerialFlush = 514,
    /// Hook eager serial sender for TRNG output. This will not succeed if hooked for console mode already.
    SerialHookTrngSender = 515,
    /// Hook serial to the console input
    SerialHookConsole = 516,
    /// Clear any hooks
    SerialClearHooks = 517,
    /// TRNG send poll
    SerialTrngPoll = 518,
    /// Send serial data, without waiting for a response.
    SerialSendData = 519,
    /// Submit serial data and return the number of bytes accepted by the
    /// USB CDC transmit buffer.
    ///
    /// This blocks until the USB service processes the IPC request. It does
    /// not wait for the host to receive the data.
    SerialSendDataBlocking = 520,

    /// Interrupt-context USB stack messages
    IrqFidoRx = 768,
    IrqSerialRx = 769,
    /// UVC streamer finished transmitting the staged frame
    #[cfg(feature = "uvc")]
    IrqUvcFrameDone = 770,
    /// UVC stream started (arg1 = 1) or stopped (arg1 = 0) by the host
    #[cfg(feature = "uvc")]
    IrqUvcStreamChange = 771,

    /// Lend a chunk of a raw UYVY frame for transmission over UVC. `valid` carries the chunk
    /// length in and a `UVC_RESULT_*` code out; `offset` carries `UVC_CHUNK_*` flags and the
    /// payload data size (see `UVC_CHUNK_FIRST`). Ignored (with an error reply) without `uvc`.
    UvcSendFrame = 1100,
    /// Register a server to be notified when the host starts (arg1 = 1) or stops (arg1 = 0) the
    /// video stream.
    RegisterUvcObserver = 1101,
    /// Query UVC stream state: returns (streaming, frames_sent)
    UvcStatus = 1102,
    /// Drop off the bus and re-enumerate: the device core is reset and restarted, so the host
    /// sees an unplug and a fresh plug-in. A recovery action for a wedged link; it also
    /// restarts the USB serial console. Blocking scalar, replies once the core is back up.
    UsbBusReset = 1103,

    #[cfg(feature = "mass-storage")]
    SetBlockDevice = 1024,
    #[cfg(feature = "mass-storage")]
    SetBlockDeviceSID = 1025,
    #[cfg(feature = "mass-storage")]
    ResetBlockDevice = 1026,

    /// Platform-specific messages for callback handlers
    #[cfg(feature = "bao1x")]
    PmicIrq = 1536,

    // HIDv2
    /// Read a HID report
    HIDReadReport = 1027,

    /// Write a HID report
    HIDWriteReport = 1028,

    /// Set the HID descriptor to be pushed to the USB host
    HIDSetDescriptor = 1029,

    /// Unset HID descriptor and reset HIDv2 state
    HIDUnsetDescriptor = 1030,

    /// Handle the USB interrupt
    UsbIrqHandler = 2048,
    /// Suspend/resume callback
    #[cfg(any(feature = "renode", feature = "precursor", feature = "hosted"))]
    SuspendResume = 2049,
    /// Exits the server
    Quit = 4096,
    /// Invalid opcode gutter
    InvalidCall = 4097,

    /// API used by the logging crate. The number is hard-coded; don't change it.
    LogString = 8192,
}

// The log crate depends on this API not changing.
#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Clone)]
pub struct UsbString {
    pub s: String,
    pub sent: Option<u32>,
}

#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Copy, Clone)]
pub struct U2fMsgIpc {
    /// All U2F protocol messages are 64 bytes
    pub data: [u8; 64],
    /// Encodes the state of the message
    pub code: U2fCode,
    /// Specifies an optional timeout
    pub timeout_ms: Option<u64>,
}

#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Copy, Clone, Eq, PartialEq)]
pub enum U2fCode {
    Tx,
    TxAck,
    RxWait,
    RxAck,
    RxTimeout,
    Hangup,
    Denied,
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[repr(usize)]
#[allow(dead_code)]
pub enum UsbDeviceType {
    Debug = 0,
    FidoKbd = 1,
    Fido = 2,
    MassStorage = 3,
    Serial = 4,
    HIDv2 = 5,
}
use std::convert::TryFrom;

impl TryFrom<usize> for UsbDeviceType {
    type Error = &'static str;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(UsbDeviceType::Debug),
            1 => Ok(UsbDeviceType::FidoKbd),
            2 => Ok(UsbDeviceType::Fido),
            3 => Ok(UsbDeviceType::MassStorage),
            4 => Ok(UsbDeviceType::Serial),
            5 => Ok(UsbDeviceType::HIDv2),
            _ => Err("Invalid UsbDeviceType specifier"),
        }
    }
}

#[allow(dead_code)]
pub const SERIAL_BINARY_BUFLEN: usize = 3840; // save 256 bytes on the page for Rkyv overhead
#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Clone)]
pub struct UsbSerialAscii {
    pub s: String,
    pub delimiter: Option<char>,
}

#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
pub struct UsbSerialBinary {
    pub d: Vec<u8>,
}

#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
pub struct UsbSerialSend {
    /// Data submitted to the USB CDC transmit buffer.
    pub d: Vec<u8>,

    /// Length of the contiguous prefix accepted by the service.
    ///
    /// `None` means that the service did not return a valid response.
    /// `Some(0)` may indicate that USB is not configured or that the
    /// transmit buffer could not accept data.
    pub sent: Option<u32>,
}

pub const MAX_HID_REPORT_DESCRIPTOR_LEN: usize = 1024;

#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Copy, Clone)]
pub struct HIDReportDescriptorMessage {
    pub descriptor: [u8; MAX_HID_REPORT_DESCRIPTOR_LEN],
    pub len: usize,
}

#[derive(Copy, Clone, Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[repr(C, align(8))]
pub struct HIDReport(pub [u8; 64]);

impl Default for HIDReport {
    fn default() -> Self { return Self([0u8; 64]); }
}

#[derive(Debug, Default, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Copy, Clone)]
pub struct HIDReportMessage {
    pub data: Option<HIDReport>,
}

/// this structure is used to register a USB listener.
#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Clone)]
pub(crate) struct UsbListenerRegistration {
    pub server_name: String,
    pub listener_op_id: usize,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(usize)]
#[allow(dead_code)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Err = 4,
}

#[allow(dead_code)]
impl TryFrom<usize> for LogLevel {
    type Error = &'static str;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(LogLevel::Trace),
            1 => Ok(LogLevel::Debug),
            2 => Ok(LogLevel::Info),
            3 => Ok(LogLevel::Warn),
            4 => Ok(LogLevel::Err),
            _ => Err("Invalid LogLevel"),
        }
    }
}

// ---- UVC (USB video class) ----
/// A video mode offered to the host: one UVC frame descriptor each. Pixel format is always
/// UYVY (2 bytes per pixel). The sensor reads a centred `(width + line_pad) * ratio` by
/// `(height + 1) * ratio` window and sub-samples it by `ratio`; the pad covers the sensor's dark
/// last columns (the frame copy takes the first `width` samples of each line), and the extra
/// line covers the unreliable last captured line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UvcMode {
    pub width: usize,
    pub height: usize,
    /// sensor sub-sampling ratio: even, or odd up to 7 (read in groups of twice the ratio,
    /// keeping one Bayer quad of each; see `Gc2145::set_resolution`); 1 is not coherent
    pub ratio: u16,
    /// extra columns captured per line
    pub line_pad: usize,
    /// frame interval advertised to the host, in 100 ns units
    pub interval: u32,
    /// image rows per UVC payload; payload data = rows * width * 2 bytes (at most 4800)
    pub payload_rows: usize,
    /// image rows per DMA transfer: a frame is captured as a chain of transfers through a ring
    /// of `UVC_RING_DEPTH` slots of this size in the camera's IFRAM, and each completed
    /// transfer is handed to the USB service as one chunk. A multiple of `payload_rows`, at most
    /// `UVC_CHUNK_PAYLOADS * payload_rows`.
    pub slot_rows: usize,
}

#[allow(dead_code)] // the library target only needs the table
impl UvcMode {
    pub const fn frame_bytes(&self) -> usize { self.width * self.height * 2 }

    pub const fn payload_data(&self) -> usize { self.payload_rows * self.width * 2 }

    /// Pixels per captured line: the padded width
    pub const fn line_px(&self) -> usize { self.width + self.line_pad }

    /// DMA transfers per frame
    pub const fn transfers(&self) -> usize { (self.height + self.slot_rows - 1) / self.slot_rows }
}

/// The modes, in UVC frame-descriptor order (bFrameIndex = index + 1). Index 0 is the default
/// frame, which is what hosts open unless told otherwise, so the largest picture comes first.
pub const UVC_MODES: [UvcMode; 3] = [
    // 1556x1154 window, 1/2, sensor ~11 fps
    UvcMode {
        width: 768,
        height: 576,
        ratio: 2,
        line_pad: 10,
        interval: 833_333,
        payload_rows: 3,
        slot_rows: 24,
    },
    // 1568x1152 window, 1/4: nearly the full field of view, sensor ~11 fps
    UvcMode {
        width: 384,
        height: 288,
        ratio: 4,
        line_pad: 8,
        interval: 833_333,
        payload_rows: 6,
        slot_rows: 48,
    },
    // 640x480 sensor window (the QR scanner's), 1/4: the low-latency mode, sensor ~37 fps
    UvcMode {
        width: 160,
        height: 120,
        ratio: 4,
        line_pad: 24,
        interval: 333_333,
        payload_rows: 15,
        slot_rows: 60,
    },
    // A full-resolution (ratio 1) mode is deliberately absent: the sensor's ratio-1 output is
    // not coherent through this camera DMA (see the note in Gc2145::init_window).
];

/// Largest payload data size across the modes; sets the staging slot size
pub const UVC_MAX_PAYLOAD_DATA: usize = 4800;
/// Frames are handed to the USB service in chunks of at most this many payloads
pub const UVC_CHUNK_PAYLOADS: usize = 8;
/// Largest chunk in bytes (fits a 10-page buffer)
pub const UVC_CHUNK_MAX_BYTES: usize = UVC_CHUNK_PAYLOADS * UVC_MAX_PAYLOAD_DATA;
/// Slots in the capture ring (see `UvcMode::slot_rows`). The DMA channel holds two transfers,
/// so with three slots the consumer has two slot times to copy one out before it is needed
/// again, and the next frame's first two slots are never the one still being copied.
#[allow(dead_code)] // the library target only needs the table
pub const UVC_RING_DEPTH: usize = 3;

/// Flags carried in the `offset` field of a `UvcSendFrame` lend, alongside the payload data
/// size: `offset = flags | payload_data << 2`.
pub const UVC_CHUNK_FIRST: usize = 1;
pub const UVC_CHUNK_LAST: usize = 2;

/// Result codes returned in the `valid` field of a `UvcSendFrame` lend
pub const UVC_RESULT_SENT: usize = 1;
pub const UVC_RESULT_NOT_STREAMING: usize = 2;
pub const UVC_RESULT_BAD_FRAME: usize = 3;

#[allow(dead_code)] // used by the client library side of this crate
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum UvcFrameResult {
    /// The frame was accepted and transmitted to the host
    Sent,
    /// The host has not started (or has stopped) the stream; the frame was discarded
    NotStreaming,
    /// The lent buffer was too short
    BadFrame,
    /// The USB service was built without UVC support
    Unsupported,
}
