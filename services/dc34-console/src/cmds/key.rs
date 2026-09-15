use String;

use crate::{CommonEnv, ShellCmdApi};

/// Inject a badge button press from the console: `key up|down|left|right|fire|menu`, or any
/// single character. The press goes through the keyboard service to whoever listens for
/// keys (the vault, normally), exactly as a physical press would, which makes the badge UI
/// testable from the serial console.
#[derive(Debug)]
pub struct Key {}

impl<'a> ShellCmdApi<'a> for Key {
    cmd_api!(key);

    fn process(&mut self, args: String, env: &mut CommonEnv) -> Result<Option<String>, xous::Error> {
        use core::fmt::Write;
        let mut ret = String::new();
        let helpstring = "key up|down|left|right|fire|menu|<char> [repeat]";
        let mut tokens = args.split_whitespace();
        let name = tokens.next();
        let repeat = tokens.next().and_then(|t| t.parse::<usize>().ok()).unwrap_or(1).clamp(1, 20);
        let k = match name {
            Some("up") => Some('\u{2191}'),
            Some("down") => Some('\u{2193}'),
            Some("left") => Some('\u{2190}'),
            Some("right") => Some('\u{2192}'),
            Some("fire") | Some("center") | Some("centre") => Some('\u{1f525}'),
            Some("menu") | Some("select") => Some('\u{2234}'),
            Some(other) if other.chars().count() == 1 => other.chars().next(),
            _ => None,
        };
        match k {
            Some(k) => {
                let kbd = bao1x_api::keyboard::Keyboard::new(&env.xns).unwrap();
                for _ in 0..repeat {
                    kbd.inject_key(k);
                    env.ticktimer.sleep_ms(120).ok();
                }
                write!(ret, "sent {:?} x{}", k, repeat).ok();
            }
            None => {
                write!(ret, "{}", helpstring).ok();
            }
        }
        Ok(Some(ret))
    }
}
