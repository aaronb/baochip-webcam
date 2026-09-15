use String;

use crate::{CommonEnv, ShellCmdApi};

/// Control the USB webcam: `webcam on|off|status`.
///
/// Normally the camera starts by itself when a host opens the video device (the USB service tells
/// the video server when the stream is committed). `on` forces capture regardless, which is
/// useful when bringing the feature up; `status` shows both sides of the pipeline.
#[derive(Debug)]
pub struct Webcam {}

impl<'a> ShellCmdApi<'a> for Webcam {
    cmd_api!(webcam);

    fn process(&mut self, args: String, env: &mut CommonEnv) -> Result<Option<String>, xous::Error> {
        use core::fmt::Write;
        let mut ret = String::new();
        let helpstring = "webcam [on [mode]|off|status|preview off|on|zoom|auto|lock|exposure <ms> [pregain] [postgain]|flicker 50|60|wb auto|cal|<r> <g> <b>|tp <pattern>|rotate on|off|usbreset|crop <words>]\nmodes: 0 768x576, 1 384x288, 2 160x120";
        let mut tokens = args.split_whitespace();
        let gfx = ux_api::service::gfx::Gfx::new(&env.xns).unwrap();
        match tokens.next() {
            Some("on") => {
                let mode = tokens.next().and_then(|t| t.parse::<usize>().ok()).unwrap_or(0);
                match gfx.webcam_control_mode(true, mode) {
                    Ok(true) => write!(ret, "webcam capturing (mode {})", mode).ok(),
                    Ok(false) => write!(ret, "webcam did not start (QR scan in progress?)").ok(),
                    Err(e) => write!(ret, "error: {:?}", e).ok(),
                }
            }
            Some("off") => match gfx.webcam_control(false) {
                Ok(_) => write!(ret, "webcam stopped").ok(),
                Err(e) => write!(ret, "error: {:?}", e).ok(),
            },
            Some("status") => {
                let usb = usb_bao1x::UsbHid::new();
                let (streaming, frames_sent) = usb.uvc_status().unwrap_or((false, 0));
                match gfx.webcam_status() {
                    Ok(s) => write!(
                        ret,
                        "camera: {} ({} captured, {} sent, {} dropped)\nusb: host {} ({} frames transmitted)",
                        if s.active { "capturing" } else { "off" },
                        s.captured,
                        s.sent,
                        s.dropped,
                        if streaming { "streaming" } else { "idle" },
                        frames_sent
                    )
                    .ok(),
                    Err(e) => write!(ret, "error: {:?}", e).ok(),
                };
                if let Ok(e) = gfx.webcam_exposure_status() {
                    write!(
                        ret,
                        "\nexposure: {} {}.{:02} ms pregain=0x{:02x} postgain=0x{:02x}\nwhite balance: {} gains=[{:02x} {:02x} {:02x}]\npreview: {}{}",
                        ["auto", "locked", "manual"][(e.mode as usize).min(2)],
                        e.exposure_us / 1000,
                        e.exposure_us % 1000 / 10,
                        e.pregain,
                        e.postgain,
                        ["sensor auto", "manual", "calibrating"][(e.wb_mode as usize).min(2)],
                        e.awb[0],
                        e.awb[1],
                        e.awb[2],
                        ["off", "full", "zoom"][(e.preview as usize).min(2)],
                        if e.rotate { ", rotated 180" } else { "" }
                    )
                    .ok();
                }
                None
            }
            Some("rotate") => {
                let on = match tokens.next() {
                    Some("on") => Some(true),
                    Some("off") => Some(false),
                    _ => None,
                };
                match on {
                    Some(on) => match gfx.webcam_rotate(on) {
                        Ok(_) => write!(ret, "picture {}", if on { "rotated 180" } else { "upright" }).ok(),
                        Err(e) => write!(ret, "error: {:?}", e).ok(),
                    },
                    None => write!(ret, "usage: webcam rotate on|off").ok(),
                }
            }
            Some("auto") => match gfx.webcam_exposure(ux_api::service::api::WebcamExposureMode::Auto) {
                Ok(_) => write!(ret, "exposure: auto").ok(),
                Err(e) => write!(ret, "error: {:?}", e).ok(),
            },
            Some("lock") => match gfx.webcam_exposure(ux_api::service::api::WebcamExposureMode::Lock) {
                Ok(_) => {
                    write!(ret, "exposure: locked (applied once auto has settled if the camera is starting)")
                        .ok()
                }
                Err(e) => write!(ret, "error: {:?}", e).ok(),
            },
            Some("preview") => {
                let (view, name) = match tokens.next() {
                    Some("off") => (0, "off"),
                    Some("zoom") => (2, "zoom (1:1 centre crop)"),
                    _ => (1, "full frame"),
                };
                match gfx.webcam_preview_view(view) {
                    Ok(_) => write!(ret, "preview {}", name).ok(),
                    Err(e) => write!(ret, "error: {:?}", e).ok(),
                }
            }
            Some("tp") => {
                // sensor test pattern (P0 0x8c/0x8d, values as in the Linux gc2145 driver)
                let pat = match tokens.next() {
                    Some("off") => Some(None),
                    Some("bars") => Some(Some(0x00)),
                    Some("white") => Some(Some(0x48)),
                    Some("yellow") => Some(Some(0x88)),
                    Some("cyan") => Some(Some(0x98)),
                    Some("green") => Some(Some(0x68)),
                    Some("magenta") => Some(Some(0xa8)),
                    Some("red") => Some(Some(0x58)),
                    Some("black") => Some(Some(0x08)),
                    _ => None,
                };
                match pat {
                    Some(None) => {
                        gfx.webcam_tune(5, 0, 0x8d, 0).ok();
                        match gfx.webcam_tune(5, 0, 0x8c, 0) {
                            Ok(_) => write!(ret, "test pattern off").ok(),
                            Err(e) => write!(ret, "error: {:?} (camera running?)", e).ok(),
                        }
                    }
                    Some(Some(v)) => {
                        gfx.webcam_tune(5, 0, 0x8c, 0x09).ok();
                        match gfx.webcam_tune(5, 0, 0x8d, v) {
                            Ok(_) => {
                                write!(ret, "test pattern 0x{:02x} (white balance gains still apply)", v).ok()
                            }
                            Err(e) => write!(ret, "error: {:?} (camera running?)", e).ok(),
                        }
                    }
                    None => {
                        write!(ret, "usage: webcam tp off|bars|white|yellow|cyan|green|magenta|red|black")
                            .ok()
                    }
                }
            }
            Some("usbreset") => {
                let usb = usb_bao1x::UsbHid::new();
                // the reply travels over the link being reset, so it is not waited for
                std::thread::spawn(move || {
                    usb.bus_reset().ok();
                });
                write!(ret, "USB bus reset: the host will see an unplug and a re-plug").ok()
            }
            Some("raw") => {
                let parse = |s: Option<&str>| -> Option<usize> { s.and_then(|t| t.parse::<usize>().ok()) };
                match (parse(tokens.next()), parse(tokens.next()), parse(tokens.next()), parse(tokens.next()))
                {
                    (Some(w), Some(h), Some(ratio), pad) => {
                        match gfx.webcam_raw(w, h, ratio, pad.unwrap_or(24)) {
                            Ok(_) => write!(
                                ret,
                                "raw capture {}x{} ratio {} pad {}",
                                w,
                                h,
                                ratio,
                                pad.unwrap_or(24)
                            )
                            .ok(),
                            Err(e) => write!(ret, "error: {:?}", e).ok(),
                        }
                    }
                    _ => write!(ret, "usage: webcam raw <w> <h> <ratio> [pad]").ok(),
                }
            }
            Some("flicker") => match tokens.next().and_then(|t| t.parse::<usize>().ok()) {
                Some(hz) if hz == 50 || hz == 60 => match gfx.webcam_tune(9, hz, 0, 0) {
                    Ok(_) => write!(ret, "anti-flicker step for {} Hz lighting", hz).ok(),
                    Err(e) => write!(ret, "error: {:?}", e).ok(),
                },
                _ => write!(ret, "usage: webcam flicker 50|60").ok(),
            },
            Some("crop") => match tokens.next().and_then(|t| t.parse::<usize>().ok()) {
                Some(words) => match gfx.webcam_tune(10, words, 0, 0) {
                    Ok(_) => write!(ret, "per-line crop {} words ({} samples)", words, words * 2).ok(),
                    Err(e) => write!(ret, "error: {:?}", e).ok(),
                },
                None => write!(ret, "usage: webcam crop <words> (bring-up, default 0)").ok(),
            },
            Some("clkdiv") => {
                let v = tokens.next().and_then(|t| {
                    t.strip_prefix("0x")
                        .map(|h| usize::from_str_radix(h, 16).ok())
                        .unwrap_or_else(|| t.parse::<usize>().ok())
                });
                match v {
                    Some(v) => match gfx.webcam_tune(4, v, 0, 0) {
                        Ok(_) => write!(ret, "ratio-1 clock divider 0x{:02x}", v).ok(),
                        Err(e) => write!(ret, "error: {:?}", e).ok(),
                    },
                    None => write!(ret, "usage: webcam clkdiv <0xNN>").ok(),
                }
            }
            Some("poke") => {
                let parse = |s: Option<&str>| -> Option<usize> {
                    s.and_then(|t| {
                        t.strip_prefix("0x")
                            .map(|h| usize::from_str_radix(h, 16).ok())
                            .unwrap_or_else(|| t.parse::<usize>().ok())
                    })
                };
                match (parse(tokens.next()), parse(tokens.next()), parse(tokens.next())) {
                    (Some(page), Some(reg), Some(val)) => match gfx.webcam_tune(5, page, reg, val) {
                        Ok(_) => write!(
                            ret,
                            "poked p{} 0x{:02x} <- 0x{:02x} (see log for readback)",
                            page, reg, val
                        )
                        .ok(),
                        Err(_) => write!(ret, "poke failed (camera off?)").ok(),
                    },
                    _ => write!(ret, "usage: webcam poke <page> <reg> <val>").ok(),
                }
            }
            Some("wb") => {
                let first = tokens.next();
                match first {
                    Some("auto") => match gfx.webcam_white_balance_auto() {
                        Ok(_) => write!(ret, "white balance: sensor auto").ok(),
                        Err(e) => write!(ret, "error: {:?}", e).ok(),
                    },
                    Some("cal") => match gfx.webcam_white_balance_calibrate() {
                        Ok(_) => write!(ret, "white balance: grey-world calibration started (see log)").ok(),
                        Err(e) => write!(ret, "error: {:?} (camera running?)", e).ok(),
                    },
                    _ => {
                        let parse = |s: Option<&str>| -> Option<u8> {
                            s.and_then(|t| {
                                t.strip_prefix("0x")
                                    .map(|h| u8::from_str_radix(h, 16).ok())
                                    .unwrap_or_else(|| t.parse::<u8>().ok())
                            })
                        };
                        match (parse(first), parse(tokens.next()), parse(tokens.next())) {
                            (Some(r), Some(g), Some(b)) => match gfx.webcam_white_balance([r, g, b]) {
                                Ok(_) => write!(
                                    ret,
                                    "white balance: manual gains 0x{:02x}/0x{:02x}/0x{:02x}",
                                    r, g, b
                                )
                                .ok(),
                                Err(e) => write!(ret, "error: {:?}", e).ok(),
                            },
                            _ => write!(
                                ret,
                                "usage: webcam wb auto | cal | <r> <g> <b> (4.4 fixed point, 0x40 = 1.0)"
                            )
                            .ok(),
                        }
                    }
                }
            }
            Some("exposure") => {
                let parse = |s: Option<&str>, default: u32| -> Option<u32> {
                    match s {
                        None => Some(default),
                        Some(t) => t
                            .strip_prefix("0x")
                            .map(|h| u32::from_str_radix(h, 16).ok())
                            .unwrap_or_else(|| t.parse::<u32>().ok()),
                    }
                };
                // milliseconds, fractions allowed
                let exposure_us =
                    tokens.next().and_then(|t| t.parse::<f32>().ok()).map(|ms| (ms * 1000.0) as u32);
                let pregain = parse(tokens.next(), 0x20);
                let postgain = parse(tokens.next(), 0x40);
                match (exposure_us, pregain, postgain) {
                    (Some(exposure_us), Some(pregain), Some(postgain)) if exposure_us > 0 => {
                        match gfx.webcam_exposure(ux_api::service::api::WebcamExposureMode::Manual {
                            exposure_us,
                            pregain: pregain as u8,
                            postgain: postgain as u8,
                        }) {
                            Ok(_) => write!(
                                ret,
                                "exposure: manual {}.{:02} ms, gains 0x{:02x}/0x{:02x}",
                                exposure_us / 1000,
                                exposure_us % 1000 / 10,
                                pregain,
                                postgain
                            )
                            .ok(),
                            Err(e) => write!(ret, "error: {:?}", e).ok(),
                        }
                    }
                    _ => write!(
                        ret,
                        "usage: webcam exposure <ms> [pregain] [postgain] (gains decimal or 0x hex; unity 0x20 and 0x40)"
                    )
                    .ok(),
                }
            }
            _ => write!(ret, "{}", helpstring).ok(),
        };
        Ok(Some(ret))
    }
}
