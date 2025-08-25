#![no_std]
#![no_main]

pub use bsp::hal as atsamd_hal;
use core::sync::atomic::Ordering;
use cortex_m::asm::nop;
use defmt::warn;
use defmt_rtt as _;

use atsamd_hal::{
    clock::v2::{
        clock_system_at_reset,
        dfll::FromUsb,
        dpll::Dpll,
        gclk::{Gclk, GclkDiv16, GclkDiv8},
        osculp32k::OscUlp32k,
        pclk::Pclk,
    },
    ehal::digital::OutputPin,
    pac::{self},
    qspi::Qspi,
};
use bsp::Pins;
use cortex_m_rt::entry;
use embassy_executor::{raw::Executor, Spawner};
use embassy_sync::mutex::Mutex;
use embassy_time::{Instant, Timer};

use crate::{
    extern_flash::{ExternFlash, ScsiState, FAT_PARTITION},
    os::*,
};

//mod tle8242;
mod app_info;
mod can;
mod extern_flash;
mod os;
mod usb;

//atsamd_hal::bind_multiple_interrupts!(struct Adc1Irqs {
//    ADC1: [ADC1_RESRDY, ADC1_OTHER] => atsamd_hal::adc::InterruptHandler<Adc1>;
//});
//
//atsamd_hal::bind_multiple_interrupts!(struct Adc0Irqs {
//    ADC0: [ADC0_RESRDY, ADC0_OTHER] => atsamd_hal::adc::InterruptHandler<Adc0>;
//});

#[entry]
fn main() -> ! {
    // CPU ENTRY

    // 1. Configure clocks
    let mut pac_peripherals = atsamd_hal::pac::Peripherals::take().unwrap();
    let mut cortex = cortex_m::Peripherals::take().unwrap();

    // Capture the reset reason
    let rst_reason = pac_peripherals.rstc.rcause().read();

    let pins = Pins::new(pac_peripherals.port);
    let (buses, clocks, tokens) = clock_system_at_reset(
        pac_peripherals.oscctrl,
        pac_peripherals.osc32kctrl,
        pac_peripherals.gclk,
        pac_peripherals.mclk,
        &mut pac_peripherals.nvmctrl,
    );
    // Steal PAC controller (We need mclk later)
    let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };

    defmt::info!("CPU START!");

    // Obeying the max clock speeds for 125C operation,
    // the processor clock setup shall be configured as follows
    //
    //     DFLL(48Mhz)
    //     ├── GCLK1 (2Mhz)
    //     │   ├── DPLL0(100Mhz)
    //     C   │   └── GCLK0(100Mhz)
    //     L   │       └── F_CPU
    //     K   │           └── QSPI
    //     │   └── DPLL1(160Mhz)
    //     R       ├── GCLK2(40Mhz)
    //     E       │   └── CAN0
    //     C       ├── GCLK3(80Mhz)
    //     O       │   ├── ADC0
    //     V       │   ├── ADC1
    //     E       │   ├── SERCOM(s)
    //     R       │   └── EIC
    //     Y       └── GCLK4(160Mhz)
    //     │           └── TC/TCC(s)
    //     └── GCLK6 (48Mhz)
    //         └── USB

    // Take GCLK1 from DFLL48 and divide by 12 to get 2Mhz
    let (gclk1, dfll) = Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
    let gclk1 = gclk1.div(GclkDiv16::Div(24)).enable(); // Gclk1 is now at 2Mhz
                                                        // Now configure both DPLLs. (loop div values from ATMEL SMART)
                                                        // DPLL0 loop div 50 = 100Mhz
                                                        // DPLL1 loop div 80 = 160Mhz
    let (clk_dpll0, gclk1) = Pclk::enable(tokens.pclks.dpll0, gclk1);
    let (clk_dpll1, _gclk1) = Pclk::enable(tokens.pclks.dpll1, gclk1);
    // DPLL0 at 100Mhz (2*50)
    let dpll0 = Dpll::from_pclk(tokens.dpll0, clk_dpll0)
        .loop_div(50, 0)
        .enable();
    // DPLL1 at 160Mhz (2x80)
    let dpll1 = Dpll::from_pclk(tokens.dpll1, clk_dpll1)
        .loop_div(80, 0)
        .enable();
    // Now swap GCLK0 so it is using DPLL0 as a reference rather than DFLL
    let (gclk0_100, dfll, _dpll0) = clocks.gclk0.swap_sources(dfll, dpll0);
    // Enable GCLK2 at 40Mhz (160/(4))
    let (gclk2, dpll1) = Gclk::from_source(tokens.gclks.gclk2, dpll1);
    let gclk2_40 = gclk2.div(GclkDiv8::Div(4)).enable();
    // Enable GCLK3 at 80Mhz (160/(2))
    let (gclk3, dpll1) = Gclk::from_source(tokens.gclks.gclk3, dpll1);
    let gclk3_80 = gclk3.div(GclkDiv8::Div(2)).enable();
    // Enable GCLK4 at 160Mhz (Match DPLL1 freq)
    let (gclk4, _dpll1) = Gclk::from_source(tokens.gclks.gclk4, dpll1);
    let gclk4_160 = gclk4.enable();

    // Start the systick driver (Now CPU runs at 100Mhz)
    os_start();

    let executor = EXECUTOR_THREAD_LOW.init(Executor::new(usize::MAX as *mut ()));
    let spawner_thread = SPAWNER_THREAD.init(executor.spawner());
    //warn!("{:032b}", rst_reason.bits());
    //if rst_reason.wdt().bit_is_set() {
    //    defmt::panic!("CPU previously Reset due to watchdog! - Cpu halted");
    //}

    // Switch DFLL to running with USB Clock recovery
    let (dfll_usb, _old_mode) = dfll.into_mode(FromUsb, |dfll| {});
    let (gclk6, _dfll) = Gclk::from_source(tokens.gclks.gclk6, dfll_usb);
    let gclk6_48 = gclk6.enable();
    let (pclk_usb, _gclk6_48) = Pclk::enable(tokens.pclks.usb, gclk6_48);
    // Setup the peripherals (Just the peripherals, not their usage)
    usb::usb_init(
        pac_peripherals.usb,
        pclk_usb,
        &mut mclk,
        pins.usb_dp,
        pins.usb_dm,
        &mut cortex.NVIC,
        pins.led_usb,
    );

    // EVSYS
    //let evsys = EvSysController::new(&mut mclk, pac_peripherals.evsys);
    //let evsys_channels = evsys.split(); // Split into 32 channels

    // EIC
    let (osculp32k, base) = OscUlp32k::enable(tokens.osculp32k.osculp32k, clocks.osculp32k_base);
    let gclk_7_32k = Gclk::from_source(tokens.gclks.gclk7, osculp32k).0.enable();
    let (eic_clock, gclk_7_32k) = Pclk::enable(tokens.pclks.eic, gclk_7_32k);
    //let mut eic = Eic::new(&mut mclk, (&eic_clock).into(), pac_peripherals.eic);

    let (token, _) = eic_clock.disable(gclk_7_32k);
    let (pclk_eic_fast, gclk0_100) = Pclk::enable(token, gclk0_100);

    //eic.switch_to_gclk(&pclk_eic_fast);

    //let eic_channels = eic.split();
    //let mut n2_rpm = eic_channels.0.with_pin(pins.rpm_n2.into_pull_down_interrupt()); // EIC T[16]

    //let n2_with_isr: gpio::Pin<_, PullDownInterrupt> = pins.rpm_n2.into();
    //let n2_rpm = eic_channels.0.with_pin(n2_with_isr); // EIC T[0]

    //n2_rpm.enable_interrupt();
    //n2_rpm.sense(pac::eic::config::Sense0select::Fall);
    //n2_rpm.filter(true);
    //n2_rpm.debounce();
    //n3_rpm.sense(pac::eic::config::Sense0select::Rise);
    //let rpm_1 = eic_channels.8.with_pin(pins.rpm_1); // EIC T[8] - Extra RPM sensor/Output shaft sensor
    //let rpm_2 = eic_channels.6.with_pin(pins.rpm_2); // EIC T[6] - Extra RPM sensor

    //let (_n2_rpm, n2_rpm_evsys) = n2_rpm.enable_event(evsys_channels.0);
    //let (mut n2_rpm, n3_rpm_evsys) = n2_rpm.enable_event(evsys_channels.1);
    //let (_rpm1, rpm1_rpm_evsys) = rpm_1.enable_event(evsys_channels.2);
    //let (_rpm2, rpm2_rpm_evsys) = rpm_2.enable_event(evsys_channels.3);

    let (pclk_tc2_tc3, gclk4_160) = Pclk::enable(tokens.pclks.tc2_tc3, gclk4_160);
    //let pulse_counter_n2: PulseCounter<Tc2PulseCounter, evsys::Ch1, eic::Ch0> = PulseCounterBuilder::default().build(pac_peripherals.tc2, &pclk_tc2_tc3, &mut mclk, n3_rpm_evsys);

    // QSPI
    let qspi = Qspi::new(
        &mut mclk,
        pac_peripherals.qspi,
        pins.extflash_sck,
        pins.extflash_cs,
        pins.extflash_data0,
        pins.extflash_data1,
        pins.extflash_data2,
        pins.extflash_data3,
    );

    //let apb_qspi = clocks.apbs.qspi;
    //let ahb_qspi = clocks.ahbs.qspi;
    //let (qspi, gclk0_100) = QspiBuilder::new(
    //    pins.extflash_sck,
    //    pins.extflash_cs,
    //    pins.extflash_data0,
    //    pins.extflash_data1,
    //    pins.extflash_data2,
    //    pins.extflash_data3,
    //)
    //.with_freq(50_000_000)
    //.with_mode(atsamd_hal::qspi::QspiMode::_0)
    //.build(pac_peripherals.qspi, ahb_qspi, apb_qspi, gclk0_100)
    //.unwrap();

    let flash = ExternFlash::new(qspi, pins.led_ext_flash.into());

    let mut sensor_pwr = pins.sensor_pwr_en.into_push_pull_output();
    sensor_pwr.set_high().unwrap();

    os_launch_internal_tasks(&spawner_thread, pac_peripherals.wdt);
    // Launch the async Initializing task for the TCU
    spawner_thread.must_spawn(initialize_async(spawner_thread, flash));
    //spawner_thread.must_spawn(monitor_pulse_counters(pulse_counter_n2, n2_rpm)); //,
    // Now OS is running, this loop keeps embassy running

    loop {
        let before = Instant::now().as_ticks();
        cortex_m::asm::wfe();
        let after = Instant::now().as_ticks();
        SLEEP_TICKS.fetch_add(after - before, Ordering::Relaxed);
        unsafe { executor.poll() };
    }
}

use pac::interrupt;

#[interrupt]
fn EIC_EXTINT_0() {
    defmt::info!("EIC INTERRUPT");
}

#[embassy_executor::task]
/// Start the TCU up with an async method, so we can do multiple things in parallel
/// (Like initializing EEPROM/QSPI whilst also initializing the TLE8242)
pub async fn initialize_async(spawner: &'static Spawner, mut flash: ExternFlash) {
    flash.init().await;
    let parts = flash.into_partitions();
    let _ = FAT_PARTITION.init(Mutex::new((parts.usb_fat32, ScsiState::default())));
}
