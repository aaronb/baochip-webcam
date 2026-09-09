/// UART virtual address.
///
/// See https://github.com/betrusted-io/xous-core/blob/master/docs/memory.md
pub const UART_ADDR: usize = 0x3000_0000;

pub struct Uart {}
impl Uart {
    pub fn putc(&mut self, c: u8) {
        const SFR_SR: *mut u32 = (UART_ADDR + 8) as *mut u32;
        const SFR_TXD: *mut u32 = (UART_ADDR + 0) as *mut u32;
        while unsafe{SFR_SR.read_volatile()} != 0 {}
        unsafe{SFR_TXD.write_volatile(c as u32);}
    }
}
use core::fmt::{Error, Write};
impl Write for Uart {
    fn write_str(&mut self, s: &str) -> Result<(), Error> {
        for c in s.bytes() {
            self.putc(c);
        }
        Ok(())
    }
}

#[macro_use]
pub mod debug_print_hardware {
    #[macro_export]
    macro_rules! print
    {
        ($($args:tt)+) => ({
                use core::fmt::Write;
                let _ = write!(crate::debug::Uart{}, $($args)+);
        });
    }
}

#[macro_export]
macro_rules! println
{
    () => ({
        $crate::print!("\r\n")
    });
    ($fmt:expr) => ({
        $crate::print!(concat!($fmt, "\r\n"))
    });
    ($fmt:expr, $($args:tt)+) => ({
        $crate::print!(concat!($fmt, "\r\n"), $($args)+)
    });
}
