#![no_std]
#![no_main]

use panic_rtt_target as _; // MUST HAVE for Panic (Crash) handling

use atsamd_hal::{pac, rtc::rtic::rtc_clock, rtc_monotonic};

mod tle8242;
mod uart;

pub use uart::*;

rtc_monotonic!(Mono, rtc_clock::Clock32k);

#[rtic::app(device = atsamd_hal::pac, peripherals = true, dispatchers = [EVSYS_0])]
mod app {
    use core::{sync::atomic::Ordering, time::Duration};

    use super::*;
    use atsamd_hal::{clock::{self, v2::{clock_system_at_reset, dpll::Dpll, gclk::{Gclk, GclkDiv16}, pclk::Pclk}, Tcc0Tcc1Clock}, nb, prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin, rtc::Rtc, sercom::spi::Flags, time::Hertz};
    use fugit::{ExtU32, ExtU64};
    use mcan::messageram::SharedMemory;
    use bsp::{can_deps::Capacities, serial_uart, tle_spi, Pins, SolPwrEn, Uart};
    use rtic_monotonics::Monotonic;
    use rtic_sync::portable_atomic::{AtomicU32, AtomicU64};
    use rtt_target::{rprintln, rtt_init_print};
    use tle8242::{Tle8242, TleChannel};
    use uart::Logger;

    static SLEEP_TICKS: AtomicU64 = AtomicU64::new(0);

    #[local]
    struct LocalResources {
        tle8242: Tle8242,
    }

    #[shared]
    struct SharedResources {

    }

    #[init(local=[
        #[link_section = ".can"]
        can_memory0: SharedMemory<Capacities> = SharedMemory::new()
        #[link_section = ".can"]
        can_memory1: SharedMemory<Capacities> = SharedMemory::new()
    ])]
    fn init(mut cx: init::Context) -> (SharedResources, LocalResources) {
        rtt_init_print!();

        cx.device.osc32kctrl.rtcctrl()
            .write(|w| w.rtcsel().ulp32k());

        Mono::start(cx.device.rtc); // Start this immedietly
        let pins = Pins::new(cx.device.port);
        let (_buses, clocks, tokens) = clock_system_at_reset(
            cx.device.oscctrl,
            cx.device.osc32kctrl,
            cx.device.gclk,
            cx.device.mclk,
            &mut cx.device.nvmctrl,
        );
        // Steal PAC controller (We need mclk later)
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };

        // Configure CPU to run at 100Mhz
        let (gclk1, dfll) = Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        let gclk1 = gclk1.div(GclkDiv16::Div(24)).enable();

        // Setup a peripheral channel to power up `Dpll0` from `Gclk1`
        let (pclk_dpll0, gclk1) = Pclk::enable(tokens.pclks.dpll0, gclk1);

        // Configure `Dpll0` with `2 * 50 + 0/32 = 100 MHz` frequency
        let dpll0 = Dpll::from_pclk(tokens.dpll0, pclk_dpll0)
            .loop_div(50, 0)
            .enable();

        // Swap source of `Gclk0` from Dfll to Dpll0, `48 Mhz -> 100 MHz`
        let (gclk0, _dfll, _dpll0) = clocks.gclk0.swap_sources(dfll, dpll0);

        let (pclk_sercom2, gclk0) = Pclk::enable(tokens.pclks.sercom2, gclk0);
        let (pclk_sercom5, gclk0) = Pclk::enable(tokens.pclks.sercom5, gclk0);

        let (tcc0tcc1clock, gclk0) = Pclk::enable(tokens.pclks.tcc0_tcc1, gclk0);
        
        let mut sol_pwr_en: SolPwrEn = pins.sol_pwr_en.into();
        sol_pwr_en.set_drive_strength(true);
        sol_pwr_en.set_high().unwrap();

        let spi = tle_spi(
            pclk_sercom2, 
            Hertz::MHz(20), 
            cx.device.sercom2, 
            &mut mclk, 
            pins.tle_sclk, 
            pins.tle_si,        
            pins.tle_so,
        );

        let uart = serial_uart(
            pclk_sercom5, 
            Hertz::Hz(115200), 
            cx.device.sercom5, 
            &mut mclk, 
            pins.uart_rx, 
            pins.uart_tx
        );

        Logger::init(
            uart,
            cx.device.dmac,
            &mut cx.device.pm,
            LogLevel::Debug
        );
        let tle8242 = Tle8242::new(
            spi,
            &mut cx.device.pm,
            pins.tle_clk,
            pins.tle_reset,
            pins.tle_cs,
            &tcc0tcc1clock.into(), 
            cx.device.tcc1, 
            &mut mclk
        );

        // SPI to TLE8242
        //let spi = bsp::tle_spi(pclk_sercom0, baud, sercom, &mut mclk, sclk, mosi, miso);
        query_tle::spawn();
        cpu_usage::spawn();
        (SharedResources{}, LocalResources{
            tle8242
        })
    }

    #[task(priority=1, local=[tle8242])]
    async fn query_tle(ctx: query_tle::Context) {
        let query = ctx.local.tle8242.read_version_info().await;
        //let query = ctx.local.tle8242.read_control_method().await;
        //let query = ctx.local.tle8242.read_version_info().await;
        //let query = ctx.local.tle8242.set_current(TleChannel::_3, 500).await;
        //let query = ctx.local.tle8242.read_version_info().await;
        //loop {

            //let query = ctx.local.tle8242.read_version_info(TleChannel::_3, 500).await;
            //if let Some(q) = query {
            //    rprintln!("Version: {:08b} Manf.ID: {:08b}",q.version, q.ic_manf_id);
            //}

            //ctx.local.tle8242.send_command(0).await;
            //Mono::delay(20u32.millis()).await
        //}
        //ctx.local.tle8242.read_version_info().await.unwrap();
        //for i in 20..31 {
        //    rprintln!("{}", i);
        //    let x = 1u32 << i;
        //    ctx.local.tle8242.test(x).await;
        //    Mono::delay(20u32.millis()).await
        //}
        rprintln!("Done");
        if let Some(q) = query {
            log_debug!("Version: {:08b} Manf.ID: {:08b}",q.version, q.ic_manf_id);
        }
        log_debug!("Test debug msg!");
        log_info!("Test info msg!");
        log_warn!("Test warn msg!");
        log_error!("Test error msg!");
    }

    #[task(priority=1)]
    async fn cpu_usage(mut cx: cpu_usage::Context) {
        let mut previous_tick = 0u64;
        loop {
            let events = SLEEP_TICKS.swap(0, Ordering::Relaxed);
            log_warn!("Cpu events: {}", events);
            Mono::delay(1000u64.millis()).await;
        }
    }



}