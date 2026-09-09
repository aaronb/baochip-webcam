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

/// Exposure and white-balance state of the GC2145 (page 0 registers 0x03/0x04, 0xb1..0xb6, 0x82).
#[derive(Debug, Clone, Copy, Default)]
pub struct Gc2145Exposure {
    /// coarse exposure in line units (13 bits)
    pub exposure: u16,
    /// analog pre-gain, 4.4 fixed point (0x40 = 1.0)
    pub pregain: u8,
    /// digital post-gain, 4.4 fixed point
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

    /// Freeze exposure and white balance at their current values: disable the AEC and AWB
    /// engines and write the values they had reached back as manual settings. Returns the
    /// frozen state.
    pub fn lock_exposure(&self, i2c: &mut dyn I2cApi) -> Gc2145Exposure {
        let cur = self.read_exposure(i2c);
        self.set_exposure(i2c, cur.exposure, cur.pregain, cur.postgain, Some(cur.awb));
        Gc2145Exposure { aec_on: false, awb_on: false, ..cur }
    }

    /// Manual exposure: AEC and AWB off, `exposure` in line units (13 bits), gains in the
    /// sensor's 4.4 fixed-point format (0x40 = 1.0). `awb` gains are R, G, B; `None` leaves the
    /// current white-balance gains in place.
    pub fn set_exposure(
        &self,
        i2c: &mut dyn I2cApi,
        exposure: u16,
        pregain: u8,
        postgain: u8,
        awb: Option<[u8; 3]>,
    ) {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        let mut b = [0u8; 1];
        self.peek(i2c, 0xb6, &mut b);
        self.poke(i2c, 0xb6, b[0] & !0x01);
        self.peek(i2c, 0x82, &mut b);
        self.poke(i2c, 0x82, b[0] & !0x02);
        self.poke(i2c, 0x03, ((exposure >> 8) & 0x1f) as u8);
        self.poke(i2c, 0x04, (exposure & 0xff) as u8);
        self.poke(i2c, 0xb1, pregain);
        self.poke(i2c, 0xb2, postgain);
        if let Some([r, g, bb]) = awb {
            self.poke(i2c, 0xb3, r);
            self.poke(i2c, 0xb4, g);
            self.poke(i2c, 0xb5, bb);
        }
    }

    /// Manual white balance: AWB off, gains R, G, B in 4.4 fixed point (0x40 = 1.0).
    pub fn set_awb_gains(&self, i2c: &mut dyn I2cApi, awb: [u8; 3]) {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        let mut b = [0u8; 1];
        self.peek(i2c, 0x82, &mut b);
        self.poke(i2c, 0x82, b[0] & !0x02);
        self.poke(i2c, 0xb3, awb[0]);
        self.poke(i2c, 0xb4, awb[1]);
        self.poke(i2c, 0xb5, awb[2]);
    }

    /// Hand exposure and white balance back to the sensor's automatic engines.
    pub fn unlock_exposure(&self, i2c: &mut dyn I2cApi) {
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        let mut b = [0u8; 1];
        self.peek(i2c, 0xb6, &mut b);
        self.poke(i2c, 0xb6, b[0] | 0x01);
        self.peek(i2c, 0x82, &mut b);
        self.poke(i2c, 0x82, b[0] | 0x02);
    }

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

    fn set_resolution(&self, i2c: &mut dyn I2cApi, w: u16, h: u16, ratio: u16) {
        // these define the scaling of the image. If "digital zoom" is required, decrease
        // these numbers to get a higher magnification.
        let c_ratio = ratio;
        let r_ratio = ratio;

        /* Calculates the window boundaries to obtain the desired resolution */
        let win_w = w * c_ratio;
        let win_h = h * r_ratio;
        let x = ((win_w / c_ratio) - w) / 2;
        let y = ((win_h / r_ratio) - h) / 2;
        let win_x = (UXGA_HSIZE - win_w) / 2;
        let win_y = (UXGA_VSIZE - win_h) / 2;

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
    /// line length. `resolution()` reports `window_w` x `window_h` until slicing is set.
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

        // set up YUV mode
        self.poke(i2c, GC2145_REG_RESET, GC2145_SET_P0_REGS);
        self.delay(30);
        let mut buf = [0u8; 1];
        self.peek(i2c, GC2145_REG_OUTPUT_FMT, &mut buf);
        self.delay(30);
        self.poke(
            i2c,
            GC2145_REG_OUTPUT_FMT,
            (buf[0] & !GC2145_REG_OUTPUT_FMT_MASK) | GC2145_REG_OUTPUT_FMT_YCBYCR,
        );
        self.delay(30);

        crate::println!("camera window {}x{} (subsample 1/{})", window_w, window_h, ratio);
        self.set_resolution(i2c, window_w, window_h, ratio);
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
