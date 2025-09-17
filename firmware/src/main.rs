#![no_std]
#![no_main]

use atsamd_hal::pac::SYST;
use atsamd_hal::prelude::ExtU64;
use atsamd_hal::prelude::_embedded_hal_watchdog_Watchdog;
use atsamd_hal::rtc::rtic::rtc_clock;
use atsamd_hal::rtic_time::Monotonic;
use atsamd_hal::{
    clock::v2::{
        clock_system_at_reset,
        dfll::FromUsb,
        dpll::Dpll,
        gclk::{Gclk, GclkDiv16, GclkDiv8},
        osculp32k::OscUlp32k,
        pclk::Pclk,
        rtcosc::RtcOsc,
    },
    pac::SCB,
};
use core::cell::Cell;
use core::cell::RefCell;
use core::cell::UnsafeCell;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use core::{
    fmt::write,
    ops::Deref,
    panic::{PanicInfo, PanicMessage},
};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;
use defmt_rtt as _;
use diag_common::{dyn_panic::AppPanicInfo, ram_info::modify_bootloader_info};
use mcan::embedded_can::StandardId;

pub mod can;
pub mod diag;
pub mod maths;
pub mod sensors;
pub mod usb;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    modify_bootloader_info(|inf| {
        let panic = AppPanicInfo::new(info);
        inf.app_panic = Some(panic);
    });
    SCB::sys_reset();
}

atsamd_hal::rtc_monotonic!(Mono, rtc_clock::Clock32k);

const ISOTP_BUF_SIZE: usize = 4096;

pub const CAN_ID_TX: StandardId = unsafe { StandardId::new_unchecked(0x7E9) };
pub const CAN_ID_RX: StandardId = unsafe { StandardId::new_unchecked(0x7E1) };

#[rtic::app(device = atsamd_hal::pac, dispatchers = [DAC_EMPTY_0, DAC_EMPTY_1, DAC_RESRDY_0, DAC_RESRDY_1])]
mod app {
    use core::{
        cmp::min,
        sync::atomic::{compiler_fence, Ordering},
    };

    use crate::{
        diag::KwpServer,
        sensors::{AdcData, AdcPins},
        usb::UsbData,
    };
    use atsamd_hal::{
        can::Dependencies,
        clock::v2::{pclk, types::Can0},
        fugit::HertzU32,
        serial_number,
        usb::{
            usb_device::{
                bus::UsbBusAllocator,
                device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid},
            },
            UsbBus,
        },
        watchdog::Watchdog,
    };
    use bsp::can_deps::{Capacities, RxFifo0};
    use cortex_m::{
        asm::{self, delay, wfe, wfi},
        interrupt::{disable, enable, free},
    };
    use defmt::info;
    use diag_common::isotp_endpoints::{
        can_isotp::{make_isotp_endpoint, IsoTpInterruptHandler, IsotpConsumer, IsotpCtsMsg},
        usb_isotp::{new_usb_isotp, UsbIsoTpConsumer},
        SharedIsoTpBuf,
    };
    use futures::FutureExt;
    use heapless::format;
    use mcan::{
        embedded_can::{Id, StandardId},
        filter::Filter,
        interrupt::{state::EnabledLine0, Interrupt, OwnedInterruptSet},
        message::Raw,
        messageram::SharedMemory,
    };
    use rtic_sync::{arbiter::Arbiter, signal::Signal};
    use usbd_serial::{DefaultBufferStore, SerialPort, USB_CLASS_CDC};

    use super::*;

    #[local]
    struct Resources {
        adc_data: AdcData,

        isotp_isr: IsoTpInterruptHandler<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        isotp_thread: IsotpConsumer<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        usb_isotp_thread: UsbIsoTpConsumer<
            'static,
            UsbBus,
            DefaultBufferStore,
            DefaultBufferStore,
            ISOTP_BUF_SIZE,
        >,

        can0_interrupts: OwnedInterruptSet<pclk::ids::Can0, EnabledLine0>,
        can0_fifo0: RxFifo0,

        diag_server: KwpServer,
    }

    #[shared]
    struct Shared {
        #[lock_free]
        usb_data: UsbData<'static>,
        cpu_ticks: u32,
        wdt: Watchdog,
    }

    #[init(local = [
        #[link_section = ".can"]
        message_ram: SharedMemory<Capacities> = SharedMemory::new(),
        usb_ctrl_buf: [u8; 256] = [0; 256],
        usb_alloc: Option<UsbBusAllocator<UsbBus>> = None,
        usb_sn: heapless::String<32> = heapless::String::new(),
        isotp_can_fc_signal: Signal<IsotpCtsMsg> = Signal::new(),
        isotp_msg_signal_can: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        isotp_msg_signal_usb: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        arbiter_cantx: Option<Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>> = None
        arbiter_serial: Option<Arbiter<SerialPort<'static, UsbBus, DefaultBufferStore, DefaultBufferStore>>> = None
    ])]
    fn init(cx: init::Context) -> (Shared, Resources) {
        let mut device = cx.device;
        let mut core: rtic::export::Peripherals = cx.core;
        let pins = bsp::Pins::new(device.port);
        let (mut buses, clocks, tokens) = clock_system_at_reset(
            device.oscctrl,
            device.osc32kctrl,
            device.gclk,
            device.mclk,
            &mut device.nvmctrl,
        );
        // Steal PAC controller (We need mclk later)
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };

        let mut wdt = Watchdog::new(device.wdt);
        wdt.feed();

        // Obeying the max clock speeds for 125C operation,
        // the processor clock setup shall be configured as follows
        //
        //     OSCULP(32Khz)
        //     └── RTIC OS Monotonic
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
        // Switch DFLL to running with USB Clock recovery
        let (dfll_usb, _old_mode) = dfll.into_mode(FromUsb, |_dfll| {});
        let (gclk6, _dfll) = Gclk::from_source(tokens.gclks.gclk6, dfll_usb);
        let gclk6_48 = gclk6.enable();

        // Enable GCLK2 at 40Mhz (160/(4))
        let (gclk2, dpll1) = Gclk::from_source(tokens.gclks.gclk2, dpll1);
        let gclk2_40 = gclk2.div(GclkDiv8::Div(4)).enable();
        // Enable GCLK3 at 80Mhz (160/(2))
        let (gclk3, dpll1) = Gclk::from_source(tokens.gclks.gclk3, dpll1);
        let gclk3_80 = gclk3.div(GclkDiv8::Div(2)).enable();
        // Enable GCLK4 at 160Mhz (Match DPLL1 freq)
        let (gclk4, _dpll1) = Gclk::from_source(tokens.gclks.gclk4, dpll1);
        let gclk4_160 = gclk4.enable();

        // Enable the 32Khz clock and start the RTIC Monotonic driver
        let (osculp32k, _) = OscUlp32k::enable(tokens.osculp32k.osculp32k, clocks.osculp32k_base);
        let _ = RtcOsc::enable(tokens.rtcosc, osculp32k);
        Mono::start(device.rtc);

        // Setup the systick timer to generate an interrupt at 10Hz, this will be
        // used to monitor CPU usage
        core.SYST
            .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        // Set reload value to maximum as CPU monitor resets it
        core.SYST.set_reload(0xFF_FFFF);
        core.SYST.clear_current();
        core.SYST.enable_counter();

        // Init ADCs
        let (pclk_adc0, gclk3_80) = Pclk::enable(tokens.pclks.adc0, gclk3_80);
        let (pclk_adc1, gclk3_80) = Pclk::enable(tokens.pclks.adc1, gclk3_80);
        let apb_adc0 = buses.apb.enable(tokens.apbs.adc0);
        let apb_adc1 = buses.apb.enable(tokens.apbs.adc1);
        let adc_pins = AdcPins {
            vbatt_sense: pins.vbatt_sense.into(),
            vsensor_sense: pins.vsensor_sense.into(),
            accel_plus: pins.accel_p_sense.into(),
            accel_minus: pins.accel_m_sense.into(),
            tft: pins.tft.into(),
            sol_pwr_sense: pins.sol_pwr_sense.into(),
            vsol_sense: pins.vsol_sense.into(),
        };
        let adc_data = AdcData::new(
            device.adc0,
            device.adc1,
            device.supc,
            adc_pins,
            apb_adc0,
            apb_adc1,
            pclk_adc0,
            pclk_adc1,
        );

        // -- CAN init --
        let (clk_can, gclk2_40) = Pclk::enable(tokens.pclks.can0, gclk2_40);
        let (can0_deps, gclk2_40) = Dependencies::new(
            gclk2_40,
            clk_can,
            clocks.ahbs.can0,
            pins.can_rx.into_mode(),
            pins.can_tx.into_mode(),
            device.can0,
        );

        let mut can0_cfg =
            mcan::bus::CanConfigurable::new(HertzU32::Hz(500_000), can0_deps, cx.local.message_ram)
                .unwrap();
        can0_cfg
            .filters_standard()
            .push(Filter::Classic {
                action: mcan::filter::Action::StoreFifo0,
                filter: StandardId::ZERO,
                mask: StandardId::ZERO,
            })
            .ok();

        // Enable new MSG interrupt for FIFO0
        let interrupts = can0_cfg
            .interrupts()
            .split([Interrupt::RxFifo0NewMessage].iter().copied().collect())
            .unwrap();
        let line0_interrupts = can0_cfg.interrupt_configuration().enable_line_0(interrupts);
        let mut can = can0_cfg.finalize().unwrap();
        let arbiter_cantx: &'static _ = cx.local.arbiter_cantx.insert(Arbiter::new(can.tx));
        let (isotp_isr, isotp_thread) = make_isotp_endpoint(
            Id::Standard(CAN_ID_TX),
            Id::Standard(CAN_ID_RX),
            arbiter_cantx,
            cx.local.isotp_can_fc_signal,
            cx.local.isotp_msg_signal_can,
        );

        // Init USB
        let (usb_clock, gclk2_48) = Pclk::enable(tokens.pclks.usb, gclk6_48);
        let usb_bus = UsbBus::new(
            &(usb_clock.into()),
            &mut mclk,
            pins.usb_dm,
            pins.usb_dp,
            device.usb,
        );
        // Bus allocator
        let usb_alloc: &'static _ = cx.local.usb_alloc.insert(UsbBusAllocator::new(usb_bus));
        // SerialPort CDC
        let uart: &'static _ = cx
            .local
            .arbiter_serial
            .insert(Arbiter::new(SerialPort::new(usb_alloc)));
        // Write down the device serial number in ASCII form
        let sn = serial_number();
        for b in sn {
            let _ = cx.local.usb_sn.push_str(&format!(2; "{b:02X}").unwrap());
        }
        // Build up the USB device
        let usb =
            UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x16c0, 0x27de), cx.local.usb_ctrl_buf)
                .strings(&[StringDescriptors::default()
                    .manufacturer("rnd-ash")
                    .product("Ultimate NAG52 V2")
                    .serial_number(cx.local.usb_sn)])
                .expect("Failed to set strings")
                .device_class(USB_CLASS_CDC)
                .usb_rev(UsbRev::Usb200)
                .build()
                .unwrap();
        // Configure the USB ISOTP endpoint (Over serial)
        let (isotp_usb_tx, isotp_usb_thread) = new_usb_isotp(uart, cx.local.isotp_msg_signal_usb);
        let usb_data = UsbData {
            led: pins.led_usb.into(),
            dev: usb,
            isotp: isotp_usb_tx,
        };

        async_init::spawn().unwrap();
        cpu_monitor::spawn().unwrap();
        wdt.feed();
        (
            Shared {
                usb_data,
                cpu_ticks: 0,
                wdt,
            },
            Resources {
                adc_data,
                can0_fifo0: can.rx_fifo_0,
                can0_interrupts: line0_interrupts,
                isotp_isr,
                isotp_thread,
                usb_isotp_thread: isotp_usb_thread,

                diag_server: KwpServer::new(),
            },
        )
    }

    #[idle(shared=[cpu_ticks])]
    fn idle(mut ctx: idle::Context) -> ! {
        let mut syst = unsafe { cortex_m::Peripherals::steal().SYST };
        loop {
            cortex_m::interrupt::disable();
            syst.clear_current();
            wfi();
            ctx.shared
                .cpu_ticks
                .lock(|x| *x += 0xFF_FFFF - syst.cvr.read());
            unsafe {
                cortex_m::interrupt::enable();
            }
        }
    }

    #[task(priority = 2, shared=[cpu_ticks, wdt])]
    async fn cpu_monitor(mut ctx: cpu_monitor::Context) {
        loop {
            // Reset
            //let asleep = OS_ASLEEP_TICKS.swap(0, Ordering::SeqCst);
            let mut asleep = 0;
            ctx.shared.cpu_ticks.lock(|x| {
                asleep = *x;
                *x = 0;
            });

            let actual_ticks = 100_000_000 / 4; // 100Mhz for 1 second
            info!(
                "CPU: {}%",
                (actual_ticks - asleep) as f32 / actual_ticks as f32 * 100.0
            );
            ctx.shared.wdt.lock(|wdt| wdt.feed());
            Mono::delay(250u64.millis()).await;
        }
    }

    #[task(priority = 2, local = [usb_isotp_thread, isotp_thread, diag_server])]
    async fn diag_task(cx: diag_task::Context) {
        let diag_task::LocalResources {
            usb_isotp_thread,
            isotp_thread,
            diag_server,
            ..
        } = cx.local;
        let mut is_usb: bool = false;
        loop {
            futures::select_biased! {
                buf = isotp_thread.read_payload().fuse() => {
                    is_usb = false;
                    let response = diag_server.process_cmd(
                        buf.payload(),
                        Mono::now().duration_since_epoch().to_millis(),
                    );
                    let _ = isotp_thread.write_payload(&mut Mono, response).await;
                },
                buf = usb_isotp_thread.read().fuse() => {
                    is_usb = true;
                    let response = diag_server.process_cmd(
                        buf.payload(),
                        Mono::now().duration_since_epoch().to_millis(),
                    );
                    let _ = usb_isotp_thread.write(response).await;
                },
                _ = Mono::delay(10.millis()).fuse() => {
                    // Fallthrough so we update KWP server
                }
            }
            if let Some(tx) = diag_server
                .update(Mono::now().duration_since_epoch().to_millis())
                .await
            {
                match is_usb {
                    true => {
                        let _ = usb_isotp_thread.write(tx).await;
                    }
                    false => {
                        let _ = isotp_thread.write_payload(&mut Mono, tx).await;
                    }
                }
            }
        }
    }

    #[task(priority = 1)]
    async fn async_init(_: async_init::Context) {
        adc_task::spawn().unwrap();
        diag_task::spawn().unwrap();
        Mono::delay(1000u64.millis()).await;
        diag_common::ram_info::modify_bootloader_info(|info| {
            info.reset_counter = 0;
        });
    }

    #[task(priority = 1, local=[adc_data], shared=[wdt])]
    async fn adc_task(mut cx: adc_task::Context) {
        loop {
            cx.local.adc_data.update().await;
            cx.shared.wdt.lock(|wdt| wdt.feed());
            Mono::delay(100u64.millis()).await;
        }
    }

    // -- HARDWARE TASKS BELOW --

    #[task(priority = 1, binds=USB_TRCPT0, shared=[usb_data])]
    fn usb_trcpt0(cx: usb_trcpt0::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=USB_TRCPT1, shared=[usb_data])]
    fn usb_trcpt1(cx: usb_trcpt1::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=USB_OTHER, shared=[usb_data])]
    fn usb_other(cx: usb_other::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=CAN0, local=[can0_interrupts, can0_fifo0, isotp_isr])]
    fn can0(cx: can0::Context) {
        for interrupt in cx.local.can0_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for msg in cx.local.can0_fifo0.into_iter() {
                        if msg.id() == cx.local.isotp_isr.rx_id {
                            cx.local.isotp_isr.on_frame_rx(msg.data(), 20, 8);
                        } else {
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

static OS_ASLEEP_TICKS: AtomicU32 = AtomicU32::new(0);
