use std::hash::{Hash, Hasher};

//////////////// IPC APIs
#[cfg_attr(feature = "derive-rkyv", derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize))]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]

pub struct Gid {
    /// a 128-bit random identifier for graphical objects
    gid: [u32; 4],
}
impl Gid {
    pub fn new(id: [u32; 4]) -> Self { Gid { gid: id } }

    pub fn gid(&self) -> [u32; 4] { self.gid }

    pub fn dummy() -> Self { Gid { gid: [0xdead, 0xbeef, 0xdead, 0xbeef] } }
}
impl Hash for Gid {
    fn hash<H>(&self, state: &mut H)
    where
        H: Hasher,
    {
        Hash::hash(&self.gid[..], state);
    }
}

pub const SERVER_NAME_GFX: &str = "_Graphics_";

#[derive(Debug, num_derive::FromPrimitive, num_derive::ToPrimitive)]
pub enum GfxOpcode {
    /// Flush the buffer to the screen
    Flush,

    /// Clear the buffer to "light" colored pixels
    Clear,

    /// Draw a line at the specified area
    Line, //(Line),

    /// Draw a rectangle or square at the specified coordinates
    Rectangle, //(Rectangle),

    /// Draw a rounded rectangle
    RoundedRectangle, //(RoundedRectangle),

    /// Paint a Bitmap Tile
    #[cfg(feature = "ditherpunk")]
    Tile,

    /// Draw a circle with a specified radius
    Circle, //(Circle),

    /// Retrieve the X and Y dimensions of the screen
    ScreenSize,

    /// gets info about the current glyph to assist with layout
    QueryGlyphProps, //(GlyphStyle),

    /// draws a textview
    DrawTextView, //(TextView),

    /// draws an object that requires clipping
    DrawClipObject, //(ClipObject),
    DrawClipObjectList,

    /// draws the sleep screen; assumes requests are vetted by GAM/xous-names
    DrawSleepScreen,

    /// permanently turns on the Devboot mark
    Devboot,

    /// bulk read for signature verifications
    BulkReadFonts,
    RestartBulkRead,

    /// sling the framebuffer into and out of the suspend/resume area, abusing this
    /// to help accelerate redraws between modal swaps.
    Stash,
    Pop,

    /// generates a test pattern
    TestPattern,

    /// SuspendResume callback
    #[cfg(not(feature = "bao1x"))]
    SuspendResume,

    /// draw the boot logo (for continuity as apps initialize)
    DrawBootLogo,

    /// Handle Camera IRQs
    CamIrq,

    /// V2 API for claiming ownership of screen for modal operation
    AcquireModal,
    ReleaseModal,
    /// V2 API for fast drawing of multiple objects
    UnclippedObjectList,
    /// V2 API for getting filtered keyboard events
    FilteredKeyboardListener,
    RenderQr,

    #[cfg(feature = "board-baosec")]
    AcquireQr,
    KeyPress,
    #[cfg(feature = "board-baosec")]
    PowerDown,
    #[cfg(feature = "board-baosec")]
    PowerUp,
    #[cfg(feature = "board-baosec")]
    /// This call is specific and highly optimized to the display on Baosec
    BaosecBitmap,
    #[cfg(feature = "board-baosec")]
    /// This call is specific and highly optimized to the display on Baosec
    BaosecBitmapDiffuse,
    #[cfg(feature = "board-baosec")]
    Brightness,
    #[cfg(feature = "board-baosec")]
    FlipScreen,
    /// This is used to toggle DryRun mode. The purpose of this mode is to
    /// warm up UI routines from swap memory to improve UI latency.
    #[cfg(feature = "board-baosec")]
    DryRun,
    /// Start (arg1 = 1) or stop (arg1 = 0) webcam capture: frames are forwarded to the USB
    /// video class function. Sent by the USB service's stream observer callback (arg2 = 0), and
    /// by the console (arg2 = 1, "pinned": a pinned camera ignores the observer's stop and only
    /// `webcam off` releases it). If sent as a blocking scalar, arg1 of the reply is the
    /// resulting active state.
    #[cfg(feature = "board-baosec")]
    WebcamControl,
    /// Blocking scalar; returns (active, frames captured, frames sent to USB, frames dropped)
    #[cfg(feature = "board-baosec")]
    WebcamStatus,
    /// Internal: checks that frames are arriving after a webcam start; restarts the camera if not
    #[cfg(feature = "board-baosec")]
    WebcamWatchdog,
    /// Exposure, white balance and preview control (blocking scalar). arg1 selects:
    /// 0 = auto exposure, 1 = lock exposure at the current values, 2 = manual exposure with
    /// arg2 = exposure lines, arg3 = pre-gain, arg4 = post-gain (4.4 fixed point);
    /// 3 = manual white balance, arg2..arg4 = R, G, B gains; 7 = white balance mode, arg2 = 0
    /// for the sensor's own engine, 2 for a one-shot grey-world calibration that ends in manual
    /// gains; 6 = preview, arg2 = 0 off, 1 full frame, 2 centre crop; 8 = rotate the picture a
    /// half turn, arg2 = 0 upright, 1 rotated (the sensor's readout direction, so the preview
    /// and the USB stream both turn); 4 and 5 are tuning knobs.
    /// Settings persist across capture sessions. Reply arg1 = 1 on success.
    #[cfg(feature = "board-baosec")]
    WebcamExposure,
    /// Blocking scalar; returns arg1 = exposure mode (0 auto, 1 locked, 2 manual) | white
    /// balance mode << 4 (0 sensor auto, 1 manual, 2 calibrating) | preview view << 8 |
    /// rotated << 12, arg2 = exposure lines, arg3 = pre-gain << 8 | post-gain, arg4 = AWB R << 16 | G << 8 |
    /// B. Live values while the camera is on, otherwise the last seen.
    #[cfg(feature = "board-baosec")]
    WebcamExposureStatus,

    /// Gutter for invalid calls
    InvalidCall,

    Quit,
}

#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Clone)]
pub struct TokenClaim {
    pub token: Option<[u32; 4]>,
    pub name: String,
}

/// the buffer length of this equal to the internal length passed by the
/// engine-sha512 implementation times 2 (a small amount of overhead is required
/// out of an even 4096 page for bookkeeping). We could make this a neat power of 2,
/// but then you'd end up doing an extra memory message for the overhead bits that
/// are left over.
#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Copy, Clone)]
pub struct BulkRead {
    pub buf: [u8; 7936],
    pub from_offset: u32,
    pub len: u32, // used to return the length read out of the font map
}
impl BulkRead {
    pub fn default() -> BulkRead { BulkRead { buf: [0; 7936], from_offset: 0, len: 7936 } }
}

#[derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
pub struct QrAcquisition {
    pub content: Option<String>,
    pub meta: Option<String>,
}

#[derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
pub struct QrRender {
    pub width: usize,
    pub top_left: crate::minigfx::Point,
    pub modules: Vec<bool>,
}

// this structure is used to register a keyboard listener.
#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Clone)]
pub struct KeyboardRegistration {
    pub server_name: String,
    pub listener_op_id: usize,
}

#[cfg(feature = "board-baosec")]
#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Clone)]
pub struct BaosecBitmap {
    // exactly enough for a 128x128 black and white display = 2048 bytes
    pub bits: [u32; 512],
    // top left corner of the bitmap
    pub top_left: crate::minigfx::Point,
    // bounding box of the bitmap - if we want only a portion of the bitmap to be drawn
    pub bounding_box: crate::minigfx::Rectangle,
}

/// Exposure setting for the webcam, see `GfxOpcode::WebcamExposure`
#[cfg(feature = "board-baosec")]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum WebcamExposureMode {
    Auto,
    /// freeze whatever the automatic engines have converged on
    Lock,
    /// explicit exposure (line units) and pre/post gains (4.4 fixed point, 0x40 = 1.0)
    Manual {
        exposure: u16,
        pregain: u8,
        postgain: u8,
    },
}

/// Exposure state reported by `GfxOpcode::WebcamExposureStatus`
#[cfg(feature = "board-baosec")]
#[derive(Debug, Copy, Clone, Default)]
pub struct WebcamExposureStatus {
    /// 0 auto, 1 locked, 2 manual
    pub mode: u8,
    /// 0 sensor's automatic engine, 1 manual gains, 2 grey-world calibration in progress
    pub wb_mode: u8,
    /// OLED preview: 0 off (the UI's own screen shows), 1 full frame, 2 centre crop
    pub preview: u8,
    /// the picture is rotated a half turn (badge hung upside down)
    pub rotate: bool,
    pub exposure: u16,
    pub pregain: u8,
    pub postgain: u8,
    pub awb: [u8; 3],
}

/// Webcam capture statistics, see `GfxOpcode::WebcamStatus`
#[cfg(feature = "board-baosec")]
#[derive(Debug, Copy, Clone, Default)]
pub struct WebcamStatus {
    /// camera is powered and capturing
    pub active: bool,
    /// frames captured since the camera was last started
    pub captured: usize,
    /// frames accepted and transmitted by the USB service
    pub sent: usize,
    /// frames discarded because the host was not streaming
    pub dropped: usize,
}
