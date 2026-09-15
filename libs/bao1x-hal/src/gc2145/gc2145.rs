use bao1x_api::camera::*;
use bao1x_api::*;
use utralib::CSR;
use utralib::utra;
use utralib::utra::udma_camera::REG_CAM_CFG_GLOB;

use super::constants::*;
use super::tables::*;
use crate::ifram::IframRange;
use crate::udma::Udma;
use crate::udma::*;

pub const GC2145_DEV: u8 = 0x3C;

pub const CFG_FRAMEDROP_EN: utralib::Field = utralib::Field::new(1, 0, REG_CAM_CFG_GLOB);
pub const CFG_FRAMEDROP_VAL: utralib::Field = utralib::Field::new(6, 1, REG_CAM_CFG_GLOB);
pub const CFG_FRAMESLICE_EN: utralib::Field = utralib::Field::new(1, 7, REG_CAM_CFG_GLOB);
pub const CFG_FORMAT: utralib::Field = utralib::Field::new(3, 8, REG_CAM_CFG_GLOB);
pub const CFG_SHIFT: utralib::Field = utralib::Field::new(4, 11, REG_CAM_CFG_GLOB);
pub const CFG_SOF_SYNC: utralib::Field = utralib::Field::new(1, 30, REG_CAM_CFG_GLOB);
pub const CFG_GLOB_EN: utralib::Field = utralib::Field::new(1, 31, REG_CAM_CFG_GLOB);

/// Rate of the sensor's timing unit. The datasheet (7.1.1) gives the row time as
/// `Hb + Sh_delay + win_width + 4` and the frame as `VB + win_height` rows, in units of the
/// readout clock, but not that clock's rate, which follows from the PLL and divider settings
/// (P0:0xf7 = 0x1d and 0xf8 = 0x85 from the init table, 0xfa = 0x19 from `set_resolution`).
/// Measured 2026-09-15: the 768x576 webcam mode (1889-unit rows, 1212-row frames) ran at
/// 12.04 fps with the exposure well inside the frame.
pub const TIMING_UNITS_PER_SECOND: u64 = 27_565_000;

/// Mains frequency the AEC's anti-flicker step is derived from until `set_mains_hz` says otherwise
pub const DEFAULT_MAINS_HZ: u32 = 60;

/// Exposure and white-balance state of the GC2145 (page 0 registers 0x03/0x04, 0xb1..0xb6, 0x82).
#[derive(Debug, Clone, Copy, Default)]
pub struct Gc2145Exposure {
    /// exposure in rows (13 bits); a row's duration depends on the readout window, see
    /// `Gc2145::exposure_us_for_rows`
    pub exposure: u16,
    /// digital pre-gain (P0:0xb1). The datasheet gives no format; unity is 0x20, its default
    /// (the AEC's own pre-gain ceiling, P1:0x1f, is 0x35 in `GC2145_AEC`)
    pub pregain: u8,
    /// digital post-gain (P0:0xb2); unity is 0x40, its default
    pub postgain: u8,
    /// white-balance gains R, G, B, 4.4 fixed point
    pub awb: [u8; 3],
    pub aec_on: bool,
    pub awb_on: bool,
}

pub struct Gc2145 {
    csr: CSR<u32>,
    ifram: Option<IframRange>,
    resolution: Resolution,
    /// output size the sensor was configured for (before slicing)
    dims: (usize, usize),
    slicing: Option<(usize, usize)>,
    /// row duration of the configured readout window, in sensor timing units
    row_units: u32,
    /// frame length of the configured readout window, in rows
    frame_rows: u32,
    /// mains frequency the AEC's anti-flicker step is derived from
    mains_hz: u32,
}

impl Udma for Gc2145 {
    fn csr_mut(&mut self) -> &mut CSR<u32> { &mut self.csr }

    fn csr(&self) -> &CSR<u32> { &self.csr }
}

impl Gc2145 {
    /// Extra columns captured per line for `Resolution::Res160x120`. The first 3 words (6 px) of
    /// every captured line are stale pipeline carry-over, and the sensor's last few columns
    /// come out dark, so the consumer takes one image width starting 3 words into each line
    /// (see bao-video's webcam path). Word-aligned (24 px = 48 bytes).
    pub const LINE_PAD: usize = 24;

    #[cfg(feature = "std")]
    /// Safety: clocks must be turned on before this is called
    pub unsafe fn new() -> Result<Self, xous::Error> {
        let ifram_virt = xous::syscall::map_memory(
            xous::MemoryAddress::new(crate::board::CAM_IFRAM_ADDR),
            None,
            crate::board::CAM_IFRAM_LEN_PAGES * 4096,
            xous::MemoryFlags::R | xous::MemoryFlags::W,
        )?;
        let ifram = IframRange::from_raw_parts(
            crate::board::CAM_IFRAM_ADDR,
            ifram_virt.as_ptr() as usize,
            ifram_virt.len(),
        );
        Ok(Gc2145::new_with_ifram(ifram))
    }

    pub unsafe fn new_with_ifram(ifram: IframRange) -> Self {
        #[cfg(target_os = "xous")]
        let csr_range = xous::syscall::map_memory(
            xous::MemoryAddress::new(utra::udma_camera::HW_UDMA_CAMERA_BASE),
            None,
            4096,
            xous::MemoryFlags::R | xous::MemoryFlags::W,
        )
        .expect("couldn't map cam port");
        #[cfg(target_os = "xous")]
        let csr = CSR::new(csr_range.as_mut_ptr() as *mut u32);
        #[cfg(not(target_os = "xous"))]
        let csr = CSR::new(utra::udma_camera::HW_UDMA_CAMERA_BASE as *mut u32);

        Self {
            csr,
            ifram: Some(ifram),
            // bogus value
            resolution: Resolution::Res160x120,
            dims: (160, 120),
            slicing: None,
            // the 768x576 webcam window's timing, until a window is configured
            row_units: 1889,
            frame_rows: 1212,
            mains_hz: DEFAULT_MAINS_HZ,
        }
    }

    /// Snapshot of the sensor's exposure and white-balance state (page 0 registers).
    pub fn read_exposure(&self, i2c: &mut dyn I2cApi) -> Gc2145Exposure {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        let mut b = [0u8; 1];
        let mut rd = |adr: u8| {
            self.peek(i2c, adr, &mut b);
            b[0]
        };
        let exposure = ((rd(0x03) as u16 & 0x1f) << 8) | rd(0x04) as u16;
        let pregain = rd(0xb1);
        let postgain = rd(0xb2);
        let awb = [rd(0xb3), rd(0xb4), rd(0xb5)];
        let aec_on = rd(0xb6) & 0x01 != 0;
        let awb_on = rd(0x82) & 0x02 != 0;
        Gc2145Exposure { exposure, pregain, postgain, awb, aec_on, awb_on }
    }

    /// Freeze exposure at its current value: disable the AEC engine and write the values it
    /// had reached back as manual settings. White balance is left as it is. Returns the frozen
    /// state.
    pub fn lock_exposure(&self, i2c: &mut dyn I2cApi) -> Gc2145Exposure {
        let cur = self.read_exposure(i2c);
        self.set_exposure(i2c, cur.exposure, cur.pregain, cur.postgain);
        Gc2145Exposure { aec_on: false, ..cur }
    }

    /// Manual exposure: AEC off, `exposure` in rows (13 bits; see `exposure_rows_for_us`), gains
    /// as in `Gc2145Exposure` (unity 0x20 pre, 0x40 post). White balance is not touched.
    pub fn set_exposure(&self, i2c: &mut dyn I2cApi, exposure: u16, pregain: u8, postgain: u8) {
        self.set_aec_enable(i2c, false);
        self.poke(i2c, 0x03, ((exposure >> 8) & 0x1f) as u8);
        self.poke(i2c, 0x04, (exposure & 0xff) as u8);
        self.poke(i2c, 0xb1, pregain);
        self.poke(i2c, 0xb2, postgain);
    }

    /// Manual white balance: AWB off, gains R, G, B in 4.4 fixed point (0x40 = 1.0).
    pub fn set_awb_gains(&self, i2c: &mut dyn I2cApi, awb: [u8; 3]) {
        self.set_awb_enable(i2c, false);
        self.poke(i2c, 0xb3, awb[0]);
        self.poke(i2c, 0xb4, awb[1]);
        self.poke(i2c, 0xb5, awb[2]);
    }

    /// Turn the sensor's automatic exposure engine on or off (page 0 register 0xb6 bit 0).
    /// Switching it off leaves the exposure and gains at the values it last wrote.
    pub fn set_aec_enable(&self, i2c: &mut dyn I2cApi, on: bool) {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        let mut b = [0u8; 1];
        self.peek(i2c, 0xb6, &mut b);
        self.poke(i2c, 0xb6, if on { b[0] | 0x01 } else { b[0] & !0x01 });
    }

    /// Turn the sensor's automatic white-balance engine on or off (page 0 register 0x82 bit 1).
    /// Switching it off leaves the gains at the values it last wrote.
    pub fn set_awb_enable(&self, i2c: &mut dyn I2cApi, on: bool) {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        let mut b = [0u8; 1];
        self.peek(i2c, 0x82, &mut b);
        self.poke(i2c, 0x82, if on { b[0] | 0x02 } else { b[0] & !0x02 });
    }

    /// Duration of one row of the configured readout window, in nanoseconds.
    pub fn row_ns(&self) -> u64 {
        (self.row_units.max(1) as u64 * 1_000_000_000 / TIMING_UNITS_PER_SECOND).max(1)
    }

    /// Frame time of the configured readout window while the exposure fits inside the frame
    /// (a longer exposure stretches the frame), in microseconds.
    pub fn frame_us(&self) -> u32 { (self.frame_rows as u64 * self.row_ns() / 1000) as u32 }

    /// Exposure rows (13 bits, at least one) for an exposure time, for the configured window.
    pub fn exposure_rows_for_us(&self, us: u32) -> u16 {
        let ns = self.row_ns();
        ((us as u64 * 1000 + ns / 2) / ns).clamp(1, 0x1fff) as u16
    }

    /// Exposure time in microseconds of `rows` exposure rows, for the configured window.
    pub fn exposure_us_for_rows(&self, rows: u16) -> u32 { (rows as u64 * self.row_ns() / 1000) as u32 }

    /// Set the mains frequency (50 or 60 Hz; anything else is taken as 60) the AEC's anti-flicker
    /// step is derived from. Takes effect at the next `init_window`, or at once through
    /// `apply_anti_flicker`.
    pub fn set_mains_hz(&mut self, hz: u32) { self.mains_hz = if hz == 50 { 50 } else { 60 }; }

    pub fn mains_hz(&self) -> u32 { self.mains_hz }

    /// Program the AEC's anti-flicker step (P1:0x25/0x26) and exposure levels 1-4
    /// (P1:0x27..0x2e) for the configured readout window, as the Linux driver does per mode.
    /// Both are in rows, so they follow the row time: the step is the lighting's flicker period
    /// (half the mains period), and the levels cap the AEC at the whole steps that fit in a
    /// frame, so the engine avoids banding and never stretches the frame. Returns (step, level)
    /// in rows.
    pub fn apply_anti_flicker(&self, i2c: &mut dyn I2cApi) -> (u16, u16) {
        let ns = self.row_ns();
        let flicker_ns = 1_000_000_000u64 / (2 * self.mains_hz as u64);
        let step = ((flicker_ns + ns / 2) / ns).clamp(1, 0x1fff) as u16;
        let level = ((self.frame_rows / step as u32).max(1) * step as u32).min(0x1fff) as u16;
        // page 1
        self.poke(i2c, GC2145_REG_RESET, 0x01);
        self.poke(i2c, 0x25, (step >> 8) as u8);
        self.poke(i2c, 0x26, step as u8);
        for reg in [0x27u8, 0x29, 0x2b, 0x2d] {
            self.poke(i2c, reg, (level >> 8) as u8);
            self.poke(i2c, reg + 1, level as u8);
        }
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        (step, level)
    }

    /// Timing of the readout window `set_resolution` wrote (datasheet 7.1.1): row time
    /// `Hb + Sh_delay + win_width + 4` in timing units, and frame length `VB + win_height` in
    /// rows (`Vt + 8` with `Vt = win_height - 8`). The blanking registers are read back; the
    /// window size is not, because its registers still read the init table's window right
    /// after a write (measured: every mode read 1618x1216).
    fn set_timing(&mut self, i2c: &mut dyn I2cApi, readout: (u16, u16)) {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        let mut b = [0u8; 2];
        let mut rd = |adr: u8, high_bits: u8| {
            self.peek(i2c, adr, &mut b);
            ((b[0] & high_bits) as u32) << 8 | b[1] as u32
        };
        let hb = rd(0x05, 0x0f);
        let vb = rd(0x07, 0x1f);
        let sh_delay = rd(0x11, 0x03);
        self.row_units = hb + sh_delay + readout.0 as u32 + 4;
        self.frame_rows = vb + readout.1 as u32;
    }

    /// Frame length of the configured readout window in rows, while the exposure fits inside it.
    pub fn frame_rows(&self) -> u32 { self.frame_rows }

    pub fn release_ifram(&mut self) {
        if let Some(ifram) = self.ifram.take() {
            xous::syscall::unmap_memory(ifram.virt_range).unwrap();
        }
    }

    pub fn claim_ifram(&mut self) -> Result<(), xous::Error> {
        if self.ifram.is_none() {
            let ifram = xous::syscall::map_memory(
                xous::MemoryAddress::new(crate::board::CAM_IFRAM_ADDR),
                None,
                crate::board::CAM_IFRAM_LEN_PAGES * 4096,
                xous::MemoryFlags::R | xous::MemoryFlags::W,
            )?;
            let cam_ifram = unsafe {
                crate::ifram::IframRange::from_raw_parts(
                    crate::board::CAM_IFRAM_ADDR,
                    ifram.as_ptr() as usize,
                    ifram.len(),
                )
            };
            self.ifram = Some(cam_ifram);
        }
        Ok(())
    }

    pub fn has_ifram(&self) -> bool { self.ifram.is_some() }

    pub fn poke(&self, i2c: &mut dyn I2cApi, adr: u8, dat: u8) {
        const MAX_RETRIES: usize = 3;
        let mut retries = 0;
        while retries < MAX_RETRIES {
            match i2c.i2c_write(GC2145_DEV, adr, &[dat]) {
                Ok(_) => break,
                Err(_e) => {
                    #[cfg(feature = "std")]
                    log::warn!("I2C error in camera {}/{}: {:?}", retries + 1, MAX_RETRIES, _e);
                    retries += 1;
                }
            }
        }
    }

    // chip does not support sequential reads
    pub fn peek(&self, i2c: &mut dyn I2cApi, adr: u8, dat: &mut [u8]) {
        for (i, d) in dat.iter_mut().enumerate() {
            let mut one_byte = [0u8];
            i2c.i2c_read(GC2145_DEV, adr + i as u8, &mut one_byte, false).expect("read failed");
            *d = one_byte[0];
        }
    }

    fn gc2145_set_window(&self, i2c: &mut dyn I2cApi, mut reg: u8, x: u16, y: u16, w: u16, h: u16) {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);

        /* Y/row offset */
        self.poke(i2c, reg, (y >> 8) as u8);
        reg += 1;
        self.poke(i2c, reg, (y & 0xff) as u8);
        reg += 1;

        /* X/col offset */
        self.poke(i2c, reg, (x >> 8) as u8);
        reg += 1;
        self.poke(i2c, reg, (x & 0xff) as u8);
        reg += 1;

        /* Window height */
        self.poke(i2c, reg, (h >> 8) as u8);
        reg += 1;
        self.poke(i2c, reg, (h & 0xff) as u8);
        reg += 1;

        /* Window width */
        self.poke(i2c, reg, (w >> 8) as u8);
        reg += 1;
        self.poke(i2c, reg, (w & 0xff) as u8);
    }

    /// Returns the readout window size written (P0:0x0d..0x10).
    fn set_resolution(&self, i2c: &mut dyn I2cApi, w: u16, h: u16, ratio: u16) -> (u16, u16) {
        // these define the scaling of the image. If "digital zoom" is required, decrease
        // these numbers to get a higher magnification.
        let c_ratio = ratio;
        let r_ratio = ratio;

        /* Calculates the window boundaries to obtain the desired resolution */
        let win_w = w * c_ratio;
        let win_h = h * r_ratio;
        let x = ((win_w / c_ratio) - w) / 2;
        let y = ((win_h / r_ratio) - h) / 2;
        // The readout window must start on an even row and column: the ISP demosaics on the
        // assumption that the first pixel of the window is the first pixel of a Bayer quad, and
        // an odd start shifts the colour filter phase by one pixel (the red and blue sites land
        // on real green pixels, so everything comes out green or magenta). The caller's extra
        // line makes the 768x576 window 1154 rows tall, which put the start at row 23.
        let win_x = ((UXGA_HSIZE - win_w) / 2) & !1;
        let win_y = ((UXGA_VSIZE - win_h) / 2) & !1;

        /* Set readout window first. */
        self.gc2145_set_window(i2c, GC2145_REG_BLANK_WINDOW_BASE, win_x, win_y, win_w + 16, win_h + 8);

        /* Set cropping window next. */
        self.gc2145_set_window(i2c, GC2145_REG_WINDOW_BASE, x, y, w, h);

        /* Enable crop */
        self.poke(i2c, GC2145_REG_CROP_ENABLE, GC2145_CROP_SET_ENABLE);

        /* Set Sub-sampling ratio and mode */
        self.poke(i2c, GC2145_REG_SUBSAMPLE, ((r_ratio << 4) | c_ratio) as u8);

        // Sub-sample mode: nearest-neighbour averaging plus "use" mode for real sub-sampling.
        // At ratio 1 that mode produces no frames at all (measured); the "smooth" mode the init
        // table starts from works there.
        let mode = if ratio == 1 { GC2145_SUBSAMPLE_MODE_SMOOTH } else { 0x32 };
        self.poke(i2c, GC2145_REG_SUBSAMPLE_MODE, mode);

        self.delay(30);

        // faster clock enables a faster frame rate
        // now at 35Hz frame rate
        self.poke(i2c, 0xFA, 0x19);
        (win_w + 16, win_h + 8)
    }

    #[inline(never)]
    pub fn init(&mut self, i2c: &mut dyn I2cApi, resolution: Resolution) {
        let (w, h) = resolution.into();
        // Sub-sampling ratio: 320x240 reads a 640x480 window at 1/2. 160x120 keeps the same
        // 640x480 window (same field of view) at 1/4 rather than zooming in on a 320x240 window.
        // Only even ratios are clean on this sensor (odd ones scramble the Bayer phase).
        let ratio = match resolution {
            Resolution::Res160x120 => 4u16,
            _ => 2u16,
        };
        // Full-frame capture (no slicing) shows the first ~6 pixels of every line as stale
        // pipeline carry-over, the sensor's dummy columns come out dark, and the last captured
        // line is unreliable. Capture `LINE_PAD` extra columns and one extra line and let the
        // caller slice/skip them (see `Self::LINE_PAD`).
        let (line_w, lines) = match resolution {
            Resolution::Res160x120 => (w + Self::LINE_PAD, h + 1),
            _ => (w, h),
        };
        self.init_window(i2c, line_w as u16, lines as u16, ratio);
        self.resolution = resolution;
    }

    /// Reset and configure the sensor to output a `window_w` x `window_h` image, produced by
    /// reading a centred `window_w * ratio` x `window_h * ratio` region of the sensor and
    /// sub-sampling it by `ratio` (even values only). Also configures the camera DMA for that
    /// line length and the AEC's anti-flicker step for the window's row time. `resolution()`
    /// reports `window_w` x `window_h` until slicing is set.
    #[inline(never)]
    pub fn init_window(&mut self, i2c: &mut dyn I2cApi, window_w: u16, window_h: u16, ratio: u16) {
        // initiate a reset
        self.poke(i2c, GC2145_REG_RESET, GC2145_REG_SW_RESET);
        self.delay(300); // wait for reset

        // do the init pokes, these settings are from the zephyr-OS reference code
        for &[adr, dat] in GC2145_INIT.iter() {
            self.poke(i2c, adr, dat);
        }
        // setup AEC, these settings are yanked out of the Linux kernel
        for &[adr, dat] in GC2145_AEC.iter() {
            self.poke(i2c, adr, dat);
        }

        // set up YUV mode. The luma-only consumers (QR scanning) do not care about the chroma
        // order; the webcam does, see `GC2145_REG_OUTPUT_FMT_YCRYCB`.
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        self.delay(30);
        let mut buf = [0u8; 1];
        self.peek(i2c, GC2145_REG_OUTPUT_FMT, &mut buf);
        self.delay(30);
        self.poke(
            i2c,
            GC2145_REG_OUTPUT_FMT,
            (buf[0] & !GC2145_REG_OUTPUT_FMT_MASK) | GC2145_REG_OUTPUT_FMT_YCRYCB,
        );
        self.delay(30);

        crate::println!("camera window {}x{} (subsample 1/{})", window_w, window_h, ratio);
        let readout = self.set_resolution(i2c, window_w, window_h, ratio);
        self.set_timing(i2c, readout);
        let (step, level) = self.apply_anti_flicker(i2c);
        crate::println!(
            "row {} ns, frame {} rows, anti-flicker step {} rows at {} Hz, AEC ceiling {} rows",
            self.row_ns(),
            self.frame_rows,
            step,
            self.mains_hz,
            level
        );
        // NOTE: ratio 1 (no sub-sampling) does not produce a coherent image on this board: rows
        // arrive misaligned with the line length whatever the crop, readout window, PLL or
        // clock divider settings (measured 2026-09-09). Even ratios 2 and 4 are fine.
        let dma_w = window_w as usize;
        self.dims = (dma_w, window_h as usize);
        self.slicing = None;

        crate::println!("udma setup");
        // set sync polarity
        let vsync_pol = 0;
        let hsync_pol = 0;
        self.csr.wo(
            utra::udma_camera::REG_CAM_VSYNC_POLARITY,
            self.csr.ms(utra::udma_camera::REG_CAM_VSYNC_POLARITY_R_CAM_VSYNC_POLARITY, vsync_pol)
                | self.csr.ms(utra::udma_camera::REG_CAM_VSYNC_POLARITY_R_CAM_HSYNC_POLARITY, hsync_pol),
        );

        // multiply by 1
        self.csr.wo(utra::udma_camera::REG_CAM_CFG_FILTER, 0x01_01_01);

        self.csr.wo(utra::udma_camera::REG_CAM_CFG_SIZE, (dma_w as u32 - 1) << 16);

        let global = self.csr.ms(CFG_FRAMEDROP_EN, 0)
            | self.csr.ms(CFG_FORMAT, Format::BypassLe as u32)
            | self.csr.ms(CFG_FRAMESLICE_EN, 0)
            | self.csr.ms(CFG_SOF_SYNC, 1)
            | self.csr.ms(CFG_SHIFT, 0);
        self.csr.wo(utra::udma_camera::REG_CAM_CFG_GLOB, global);
    }

    /// TODO: figure out how to length-bound this to...the frame slice size? line size? idk...
    /// The offset on the slice is a "cheater" parameter which is empirically calibrated based on
    /// observed data. I don't know the underlying cause of it, but I suspect it probably has to
    /// do with data in the pipeline that's not flushed, so the first three elements are "stale"
    pub fn rx_buf<T: UdmaWidths>(&self) -> &[T] { &self.ifram.as_ref().unwrap().as_slice()[3..] }

    /// The receive buffer from its very first element, without the "stale prefix" skip of
    /// `rx_buf`. Measured on hardware for full-frame 160x120 capture (no slicing): the frame
    /// starts at byte 0, and applying the skip rotates every row by 6 pixels.
    pub fn rx_buf_unskipped<T: UdmaWidths>(&self) -> &[T] { self.ifram.as_ref().unwrap().as_slice() }

    /// TODO: figure out how to length-bound this to...the frame slice size? line size? idk...
    pub unsafe fn rx_buf_phys<T: UdmaWidths>(&self) -> &[T] { &self.ifram.as_ref().unwrap().as_phys_slice() }

    /// TODO: Rework this to use the frame sync + automatic re-initiation on capture_await() for frames
    /// TODO: Also make an interrupt driven version of this.
    pub fn capture_async(&mut self) {
        // we want the sliced resolution so resolve resolution through the method call wrapper
        let (cols, rows) = self.resolution();
        let total_len = rows * cols;
        self.csr.rmwf(CFG_GLOB_EN, 1);
        unsafe { self.udma_enqueue(Bank::Rx, &self.rx_buf_phys::<u16>()[..total_len], CFG_EN | CFG_SIZE_16) }
    }

    pub fn capture_await(&mut self, _use_yield: bool) {
        while self.udma_busy(Bank::Rx) {
            #[cfg(feature = "std")]
            if _use_yield {
                xous::yield_slice();
            }
        }
    }

    // ---- Ring capture: a frame as a chain of DMA transfers ----------------------------------
    //
    // The receive channel queues two transfers, one active and one shadow. With `CFG_SOF_SYNC`
    // clear an enqueue takes effect at once (with it set, every enqueue is held until the next
    // start of frame, so chained transfers would come from successive frames). The pixel
    // pipeline (`CFG_GLOB_EN`) only starts at a start of frame and stops as soon as it is
    // cleared, so a frame is delimited by toggling it around a chain of transfers.

    /// Enable or disable the pixel pipeline. Enabling takes effect at the next start of frame;
    /// disabling is immediate.
    pub fn pipeline_enable(&mut self, en: bool) { self.csr.rmwf(CFG_GLOB_EN, en as u32); }

    /// Hold DMA enqueues until the next start of frame (`capture_async` relies on this being
    /// set; the ring capture clears it).
    pub fn set_sof_sync(&mut self, en: bool) { self.csr.rmwf(CFG_SOF_SYNC, en as u32); }

    /// Stop and discard any queued receive transfers.
    pub fn dma_clear(&mut self) { self.udma_reset(Bank::Rx); }

    /// `(transfer loaded, shadow transfer queued)` for the receive channel, from the channel's
    /// CFG readback. "Loaded" holds from the enqueue until the transfer has drained, including
    /// the time a queued transfer waits for the pipeline to start at a start of frame (the
    /// address readback `udma_busy` relies on stays zero until then).
    pub fn dma_state(&self) -> (bool, bool) {
        // safety: reads a register of this peripheral's own DMA channel
        let cfg = unsafe { self.csr().base().add(Bank::Rx as usize).add(DmaReg::Cfg.into()).read_volatile() };
        (cfg & CFG_EN != 0, cfg & CFG_SHADOW != 0)
    }

    /// Length in bytes of the camera's IFRAM.
    pub fn ifram_len(&self) -> usize { self.ifram.as_ref().map(|i| i.as_slice::<u8>().len()).unwrap_or(0) }

    /// Queue a receive transfer of `len` bytes into the camera IFRAM, `offset` bytes in.
    ///
    /// Safety: the range must lie inside the camera IFRAM, and its contents belong to the DMA
    /// until the transfer completes.
    pub unsafe fn enqueue_rx(&mut self, offset: usize, len: usize) {
        let buf = &self.rx_buf_phys::<u8>()[offset..offset + len];
        self.udma_enqueue(Bank::Rx, buf, CFG_EN | CFG_SIZE_16);
    }

    pub fn resolution(&self) -> (usize, usize) {
        if let Some((x, y)) = self.slicing { (x, y) } else { self.dims }
    }

    pub fn set_slicing(&mut self, ll: (usize, usize), ur: (usize, usize)) {
        let (llx, lly) = ll;
        let (urxx, uryy) = ur;
        let urx = urxx.saturating_sub(1);
        let ury = uryy.saturating_sub(1);
        self.csr.wo(utra::udma_camera::REG_CAM_CFG_LL, llx as u32 & 0xFFFF | ((lly as u32 & 0xFFFF) << 16));
        self.csr.wo(utra::udma_camera::REG_CAM_CFG_UR, urx as u32 & 0xFFFF | ((ury as u32 & 0xFFFF) << 16));
        self.csr.rmwf(CFG_FRAMESLICE_EN, 1);
        self.slicing = Some((urxx - llx, uryy - lly));
        // self.csr.wo(utra::udma_camera::REG_CAM_CFG_SIZE, (urx - llx) as u32 - 1);
    }

    pub fn disable_slicing(&mut self) {
        self.csr.rmwf(CFG_FRAMESLICE_EN, 0);
        self.slicing = None;
    }

    /// Returns (product ID, manufacturer ID)
    /// Should be 0x2155, 0x0078
    pub fn read_id(&self, i2c: &mut dyn I2cApi) -> (u16, u16) {
        let mut pid = [0u8; 2];
        let mut did = [0u8; 1];
        self.peek(i2c, GC2145_PIDH, &mut pid); // should return 0x2155
        self.peek(i2c, GC2145_I2C_ID, &mut did);
        (u16::from_be_bytes(pid), did[0] as u16)
    }

    pub fn delay(&self, quantum: usize) {
        #[cfg(feature = "std")]
        {
            let tt = xous_api_ticktimer::Ticktimer::new().unwrap();
            tt.sleep_ms(quantum).ok();
        }
        #[cfg(not(feature = "std"))]
        {
            use utralib::{CSR, utra};
            // abuse the d11ctime timer to create some time-out like thing
            let mut d11c = CSR::new(utra::d11ctime::HW_D11CTIME_BASE as *mut u32);
            d11c.wfo(utra::d11ctime::CONTROL_COUNT, 333_333); // 1.0ms per interval
            let mut polarity = d11c.rf(utra::d11ctime::HEARTBEAT_BEAT);
            for _ in 0..quantum {
                while polarity == d11c.rf(utra::d11ctime::HEARTBEAT_BEAT) {}
                polarity = d11c.rf(utra::d11ctime::HEARTBEAT_BEAT);
            }
            // we have to split this because we don't know where we caught the previous interval
            if quantum == 1 {
                while polarity == d11c.rf(utra::d11ctime::HEARTBEAT_BEAT) {}
            }
        }
    }
}
