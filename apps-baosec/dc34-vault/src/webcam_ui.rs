//! Webcam appliance UI, `uvc` builds only. The badge's buttons and menu drive the exposure,
//! white balance and OLED preview of the USB webcam; the vault's own features sit idle.
//!
//! Buttons: the centre button toggles the exposure lock, up and down nudge the exposure
//! (switching to manual), left and right cycle the OLED view (status page, full-frame preview,
//! 1:1 centre crop), select opens the menu. Entering a preview view starts the camera if no
//! host has it open, and leaving for the status page stops it again unless a host is
//! streaming; the menu can also keep the camera on. The menu can rotate the picture a half
//! turn for a badge hung upside down, and save the current settings as the boot default,
//! which is applied before the host ever opens the camera.

use core::fmt::Write as _;
use std::io::{Read, Write};

use blitstr2::GlyphStyle;
use dc34_api::DC34_DICT;
use ux_api::minigfx::*;
use ux_api::service::api::{Gid, WebcamExposureMode, WebcamExposureStatus};
use ux_api::service::gfx::Gfx;

/// PDDB key (in `DC34_DICT`) holding the saved webcam settings
const DC34_WEBCAM: &str = "webcam";
const SETTINGS_VERSION: u8 = 2;
const SETTINGS_LEN: usize = 12;
/// version 1 had no rotation byte
const SETTINGS_LEN_V1: usize = 11;

/// Menu actions, carried as the scalar payload of `VaultOp::WebcamMenu`
pub const MENU_EXPOSURE_AUTO: usize = 0;
pub const MENU_EXPOSURE_LOCK: usize = 1;
pub const MENU_EXPOSURE_MANUAL: usize = 2;
pub const MENU_GAIN_UP: usize = 3;
pub const MENU_GAIN_DOWN: usize = 4;
pub const MENU_WB_AUTO: usize = 5;
pub const MENU_WB_CALIBRATE: usize = 6;
pub const MENU_VIEW_FULL: usize = 7;
pub const MENU_VIEW_ZOOM: usize = 8;
pub const MENU_VIEW_STATUS: usize = 9;
pub const MENU_SAVE: usize = 10;
pub const MENU_USB_RESET: usize = 11;
pub const MENU_ROTATE: usize = 12;
pub const MENU_CAMERA: usize = 13;

/// OLED views: the vault's status page (preview off), the full-frame preview, the centre crop
const VIEW_STATUS: u8 = 0;
const VIEW_FULL: u8 = 1;
const VIEW_ZOOM: u8 = 2;

/// how long a button's feedback line stays on the status page
const NOTICE_MS: u64 = 3000;
/// the status page's counters change every frame; redraw it no more often than this
const REDRAW_MS: u64 = 1000;

/// Saved settings. Locked and calibrating states are stored as their manual equivalents so a
/// reboot reproduces the picture rather than re-running the automatics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Settings {
    /// 0 auto, 2 manual
    exp_mode: u8,
    exposure: u16,
    pregain: u8,
    postgain: u8,
    /// 0 sensor auto, 1 manual gains
    wb_mode: u8,
    wb: [u8; 3],
    view: u8,
    /// picture rotated a half turn
    rotate: bool,
}

impl Settings {
    fn from_status(e: &WebcamExposureStatus, view: u8) -> Settings {
        Settings {
            exp_mode: if e.mode == 0 { 0 } else { 2 },
            exposure: e.exposure,
            pregain: e.pregain,
            postgain: e.postgain,
            wb_mode: if e.wb_mode == 0 { 0 } else { 1 },
            wb: e.awb,
            view,
            rotate: e.rotate,
        }
    }

    fn to_bytes(&self) -> [u8; SETTINGS_LEN] {
        [
            SETTINGS_VERSION,
            self.exp_mode,
            self.exposure as u8,
            (self.exposure >> 8) as u8,
            self.pregain,
            self.postgain,
            self.wb_mode,
            self.wb[0],
            self.wb[1],
            self.wb[2],
            self.view,
            self.rotate as u8,
        ]
    }

    fn from_bytes(b: &[u8]) -> Option<Settings> {
        let v1 = b.len() == SETTINGS_LEN_V1 && b[0] == 1;
        if !v1 && (b.len() != SETTINGS_LEN || b[0] != SETTINGS_VERSION) {
            return None;
        }
        Some(Settings {
            exp_mode: b[1],
            exposure: u16::from_le_bytes([b[2], b[3]]),
            pregain: b[4],
            postgain: b[5],
            wb_mode: b[6],
            wb: [b[7], b[8], b[9]],
            view: b[10].min(VIEW_ZOOM),
            rotate: !v1 && b[11] != 0,
        })
    }
}

pub struct WebcamUi {
    gfx: Gfx,
    /// the buttons swap sides with the picture rotated
    kbd: bao1x_api::keyboard::Keyboard,
    usb: usb_bao1x::UsbHid,
    tt: ticktimer_server::Ticktimer,
    view: u8,
    /// the view to return to when the menu closes: the menu is only visible with the preview off
    menu_saved_view: Option<u8>,
    /// the status page as last drawn, so unchanged pages are not flushed again
    last_page: String,
    last_draw_ms: u64,
    /// feedback line for the last button or menu action, with the time it was set
    notice: Option<(String, u64)>,
    /// the camera was started from here (preview or menu) rather than by a host
    local_camera: bool,
}

impl WebcamUi {
    pub fn new(xns: &xous_names::XousNames) -> Self {
        WebcamUi {
            gfx: Gfx::new(xns).unwrap(),
            kbd: bao1x_api::keyboard::Keyboard::new(xns).unwrap(),
            usb: usb_bao1x::UsbHid::new(),
            tt: ticktimer_server::Ticktimer::new().unwrap(),
            view: VIEW_FULL,
            menu_saved_view: None,
            last_page: String::new(),
            last_draw_ms: 0,
            notice: None,
            local_camera: false,
        }
    }

    /// Apply the saved settings, or the defaults if none are saved. Call once the PDDB is
    /// mounted; the video server keeps the settings and applies them whenever the camera starts.
    pub fn load_settings(&mut self, pddb: &pddb::Pddb) {
        let mut buf = [0u8; SETTINGS_LEN];
        let n = match pddb.get(DC34_DICT, DC34_WEBCAM, None, false, false, None, None::<fn()>) {
            Ok(mut key) => key.read(&mut buf).unwrap_or(0),
            Err(_) => 0,
        };
        match Settings::from_bytes(&buf[..n]) {
            Some(s) => {
                log::info!("webcam settings: {:?}", s);
                let mode = match s.exp_mode {
                    2 => WebcamExposureMode::Manual {
                        exposure: s.exposure,
                        pregain: s.pregain,
                        postgain: s.postgain,
                    },
                    _ => WebcamExposureMode::Auto,
                };
                self.gfx.webcam_exposure(mode).ok();
                if s.wb_mode == 1 {
                    self.gfx.webcam_white_balance(s.wb).ok();
                } else {
                    self.gfx.webcam_white_balance_auto().ok();
                }
                self.set_rotate(s.rotate);
                self.set_view(s.view);
            }
            None => {
                log::info!("webcam settings: none saved, using defaults");
                self.set_rotate(false);
                self.set_view(VIEW_FULL);
            }
        }
    }

    /// Rotate the picture a half turn (badge hung upside down): the sensor readout, the panel
    /// orientation for the UI's own screens, and the button directions. Setting it once also
    /// stops the accelerometer from flipping the screen on its own.
    fn set_rotate(&mut self, on: bool) {
        self.gfx.webcam_rotate(on).ok();
        self.kbd.flip_orientation(on);
    }

    /// Start the camera for local use (the video server keeps it on through host stream stops).
    fn camera_on(&mut self) {
        match self.gfx.webcam_control_mode(true, 0) {
            Ok(true) => self.local_camera = true,
            _ => self.notice("Camera: start failed"),
        }
    }

    /// End local use of the camera; the video server keeps it on if a host is streaming.
    fn camera_off(&mut self) {
        self.local_camera = false;
        self.gfx.webcam_control(false).ok();
    }

    fn save_settings(&mut self) {
        let e = match self.gfx.webcam_exposure_status() {
            Ok(e) => e,
            Err(_) => {
                self.notice("Save failed");
                return;
            }
        };
        let s = Settings::from_status(&e, self.view);
        let pddb = pddb::Pddb::new();
        let saved = match pddb.get(DC34_DICT, DC34_WEBCAM, None, true, true, Some(SETTINGS_LEN), None::<fn()>)
        {
            Ok(mut key) => key.write(&s.to_bytes()).is_ok(),
            Err(e) => {
                log::error!("webcam settings: PDDB error {:?}", e);
                false
            }
        };
        if saved {
            pddb.sync().ok();
            log::info!("webcam settings saved: {:?}", s);
            self.notice("Saved as default");
        } else {
            self.notice("Save failed");
        }
    }

    fn set_view(&mut self, view: u8) {
        self.view = view.min(VIEW_ZOOM);
        // a preview needs the camera running; the status page gives a locally started one up
        if self.view != VIEW_STATUS {
            if !self.gfx.webcam_status().map(|s| s.active).unwrap_or(false) {
                self.camera_on();
            }
        } else if self.local_camera {
            self.camera_off();
        }
        self.gfx.webcam_preview_view(self.view).ok();
        self.force_redraw();
    }

    fn notice(&mut self, text: &str) {
        self.notice = Some((text.to_string(), self.tt.elapsed_ms()));
        self.force_redraw();
    }

    fn force_redraw(&mut self) {
        self.last_page.clear();
        self.redraw();
    }

    fn exposure_status(&self) -> WebcamExposureStatus {
        self.gfx.webcam_exposure_status().unwrap_or_default()
    }

    fn set_manual(&mut self, exposure: u32, pregain: u8, postgain: u8) {
        let exposure = exposure.clamp(1, 0x1fff) as u16;
        self.gfx.webcam_exposure(WebcamExposureMode::Manual { exposure, pregain, postgain }).ok();
        let text = format!("Exp {} gain {:02x}/{:02x}", exposure, pregain, postgain);
        self.notice(&text);
    }

    /// A button press while the menu is closed.
    pub fn key(&mut self, k: char) {
        match k {
            '🔥' => {
                if self.exposure_status().mode == 1 {
                    self.gfx.webcam_exposure(WebcamExposureMode::Auto).ok();
                    self.notice("Exposure: auto");
                } else {
                    self.gfx.webcam_exposure(WebcamExposureMode::Lock).ok();
                    self.notice("Exposure: locked");
                }
            }
            '↑' | '↓' => {
                // about a quarter stop per press, from whatever is in force right now
                let e = self.exposure_status();
                let cur = e.exposure.max(1) as u32;
                let new = if k == '↑' { cur * 5 / 4 + 1 } else { cur * 4 / 5 };
                self.set_manual(new, e.pregain, e.postgain);
            }
            '→' => self.set_view((self.view + 1) % 3),
            '←' => self.set_view((self.view + 2) % 3),
            _ => {}
        }
    }

    /// The menu is about to be drawn: it is only visible with the preview off.
    pub fn menu_open(&mut self) {
        if self.view != VIEW_STATUS {
            self.menu_saved_view = Some(self.view);
            self.gfx.webcam_preview_view(VIEW_STATUS).ok();
        }
    }

    /// The menu has closed: restore the preview and refresh the page.
    pub fn menu_closed(&mut self) {
        if let Some(v) = self.menu_saved_view.take() {
            self.gfx.webcam_preview_view(v).ok();
        }
        self.force_redraw();
    }

    pub fn menu_action(&mut self, item: usize) {
        match item {
            MENU_EXPOSURE_AUTO => {
                self.gfx.webcam_exposure(WebcamExposureMode::Auto).ok();
                self.notice("Exposure: auto");
            }
            MENU_EXPOSURE_LOCK => {
                self.gfx.webcam_exposure(WebcamExposureMode::Lock).ok();
                self.notice("Exposure: locked");
            }
            MENU_EXPOSURE_MANUAL => {
                let e = self.exposure_status();
                self.set_manual(e.exposure as u32, e.pregain, e.postgain);
            }
            MENU_GAIN_UP | MENU_GAIN_DOWN => {
                let e = self.exposure_status();
                let cur = e.postgain.max(0x10) as u32;
                let new =
                    if item == MENU_GAIN_UP { cur * 5 / 4 } else { cur * 4 / 5 }.clamp(0x10, 0xff) as u8;
                self.set_manual(e.exposure as u32, e.pregain, new);
            }
            MENU_WB_AUTO => {
                self.gfx.webcam_white_balance_auto().ok();
                self.notice("WB: sensor auto");
            }
            MENU_WB_CALIBRATE => match self.gfx.webcam_white_balance_calibrate() {
                Ok(_) => self.notice("WB: calibrating"),
                Err(_) => self.notice("WB: camera is off"),
            },
            MENU_VIEW_FULL => self.set_view(VIEW_FULL),
            MENU_VIEW_ZOOM => self.set_view(VIEW_ZOOM),
            MENU_VIEW_STATUS => self.set_view(VIEW_STATUS),
            MENU_ROTATE => {
                let on = !self.exposure_status().rotate;
                self.set_rotate(on);
                self.notice(if on { "Rotated 180" } else { "Upright" });
            }
            MENU_CAMERA => {
                let (streaming, _) = self.usb.uvc_status().unwrap_or((false, 0));
                if self.local_camera {
                    self.camera_off();
                    self.notice(if streaming { "Camera: host only" } else { "Camera: off" });
                } else {
                    self.camera_on();
                    if self.local_camera {
                        self.notice("Camera: kept on");
                    }
                }
            }
            MENU_SAVE => self.save_settings(),
            MENU_USB_RESET => {
                let usb = usb_bao1x::UsbHid::new();
                // the reply comes back over the link being reset; do not wait for it here
                std::thread::spawn(move || {
                    usb.bus_reset().ok();
                });
                self.notice("USB: re-enumerating");
            }
            _ => log::warn!("webcam menu: unknown item {}", item),
        }
    }

    /// Draw the status page if it is showing and has changed. Cheap to call often.
    pub fn redraw(&mut self) {
        if self.view != VIEW_STATUS || self.menu_saved_view.is_some() {
            return;
        }
        let now = self.tt.elapsed_ms();
        if let Some((_, t)) = self.notice {
            if now.saturating_sub(t) > NOTICE_MS {
                self.notice = None;
                self.last_page.clear();
            }
        }
        if !self.last_page.is_empty() && now.saturating_sub(self.last_draw_ms) < REDRAW_MS {
            return;
        }
        let page = self.status_text();
        if page == self.last_page {
            return;
        }
        self.gfx.clear().ok();
        let mut tv = TextView::new(
            Gid::dummy(),
            TextBounds::CenteredTop(Rectangle::new(Point::new(0, 0), Point::new(128, 128))),
        );
        write!(tv, "{}", page).ok();
        tv.draw_border = false;
        tv.clear_area = true;
        tv.ellipsis = false;
        tv.invert = true;
        tv.style = GlyphStyle::Small;
        self.gfx.draw_textview(&mut tv).ok();
        self.gfx.flush().ok();
        self.last_page = page;
        self.last_draw_ms = now;
    }

    fn status_text(&self) -> String {
        let mut s = String::new();
        writeln!(s, "~Webcam~").ok();
        let (streaming, frames) = self.usb.uvc_status().unwrap_or((false, 0));
        let cam = self.gfx.webcam_status().unwrap_or_default();
        writeln!(
            s,
            "Cam {} USB {}",
            if cam.active { "on" } else { "off" },
            if streaming { "live" } else { "idle" }
        )
        .ok();
        let e = self.exposure_status();
        if e.rotate {
            writeln!(s, "Rotated 180").ok();
        }
        writeln!(s, "Sent {} drop {}", frames, cam.dropped).ok();
        writeln!(s, "Exp {} {}", ["auto", "lock", "man"][(e.mode as usize).min(2)], e.exposure).ok();
        writeln!(s, "Gain {:02x}/{:02x}", e.pregain, e.postgain).ok();
        writeln!(
            s,
            "WB {} {:02x}/{:02x}/{:02x}",
            ["auto", "man", "cal"][(e.wb_mode as usize).min(2)],
            e.awb[0],
            e.awb[1],
            e.awb[2]
        )
        .ok();
        writeln!(s, "<view> ^exp *lock").ok();
        if let Some((n, _)) = &self.notice {
            writeln!(s, "{}", n).ok();
        }
        s
    }
}
