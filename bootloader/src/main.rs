#![no_std]
#![no_main]

use crate::bl_info::{BootloaderInfo, MemoryRegion};

use atsamd_hal::{
    can::Dependencies,
    clock::v2::{
        clock_system_at_reset,
        dfll::FromUsb,
        dpll::Dpll,
        gclk::{Gclk, GclkDiv16},
        pclk::Pclk,
    },
    clock::v2::{osculp32k::OscUlp32k, pclk, rtcosc::RtcOsc},
    ehal::digital::{InputPin, OutputPin},
    fugit::ExtU64,
    fugit::HertzU32,
    gpio::{Alternate, F, PD08},
    nvm::Nvm,
    pac::Peripherals,
    prelude::_embedded_hal_Pwm,
    prelude::_embedded_hal_watchdog_WatchdogEnable,
    pwm::{Channel, TCC0Pinout, Tcc0Pwm},
    rtc::rtic::rtc_clock,
    rtic_time::Monotonic,
    serial_number,
    trng::Trng,
    usb::{
        usb_device::{
            bus::UsbBusAllocator,
            device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid},
        },
        UsbBus,
    },
    watchdog::*,
};

use core::sync::atomic::Ordering;

use bsp::can_deps::{Capacities, RxFifo0};
use futures::FutureExt;
use mcan::{
    embedded_can::Id,
    interrupt::{state::EnabledLine0, Interrupt, OwnedInterruptSet},
    message::Raw,
    tx_buffers::{DynTx, TxBufferSet},
};
use rtic_sync::{
    arbiter::Arbiter,
    signal::{Signal, SignalReader},
};

use crate::isotp::{make_isotp_endpoint, IsoTpInterruptHandler, IsotpConsumer, IsotpCtsMsg};

use core::{panic::PanicInfo, sync::atomic::AtomicU8};
use defmt::info;
use heapless::format;
use mcan::{embedded_can::StandardId, filter::Filter, messageram::SharedMemory};

use defmt_rtt as _;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::kwp::KwpServer;

mod bl_info;
pub mod isotp;
pub mod kwp;
mod usb_isotp;
use usb_isotp::*;

pub static ST_MIN_EGS: AtomicU8 = AtomicU8::new(10);
pub static BS_EGS: AtomicU8 = AtomicU8::new(0x20);

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));

    let bsp_peripherals = unsafe { Peripherals::steal() };
    let pins = bsp::Pins::new(bsp_peripherals.port);
    let _ = pins.led_eeprom.into_push_pull_output().set_high();
    let _ = pins.led_ext_flash.into_push_pull_output().set_high();
    let _ = pins.led_tle_act.into_push_pull_output().set_high();
    let _ = pins.led_usb.into_push_pull_output().set_high();
    loop {}
}

pub const CAN_ID_TX: StandardId = unsafe { StandardId::new_unchecked(0x7E9) };
pub const CAN_ID_RX: StandardId = unsafe { StandardId::new_unchecked(0x7E1) };

fn can_app_start(bl_info: &BootloaderInfo) -> bool {
    #[cfg(not(feature = "skip-app-check"))]
    return bl_info.app_flashing_not_done == 0;
    #[cfg(feature = "skip-app-check")]
    return true;
}

atsamd_hal::rtc_monotonic!(Mono, rtc_clock::Clock32k);

#[rtic::app(device = atsamd_hal::pac, dispatchers = [DAC_EMPTY_0])]
mod app {
    use super::*;

    #[local]
    struct Resources {
        isotp_isr: IsoTpInterruptHandler,
        isotp_thread: IsotpConsumer,
        isotp_usb_rx: SignalReader<'static, (usize, [u8; 4096])>,
        diag_server: KwpServer,
        tcc_led: Tcc0Pwm<PD08, Alternate<F>>,
        can0_interrupts: OwnedInterruptSet<pclk::ids::Can0, EnabledLine0>,
        can0_fifo0: RxFifo0,
    }

    #[shared]
    struct Shared {
        usb: UsbData<'static>,
    }

    #[init(local = [
        #[link_section = ".can"]
        message_ram: SharedMemory<Capacities> = SharedMemory::new(),
        usb_ctrl_buf: [u8; 256] = [0; 256],
        usb_alloc: Option<UsbBusAllocator<UsbBus>> = None,
        usb_sn: heapless::String<32> = heapless::String::new(),
        rx_ready: Signal<IsotpCtsMsg> = Signal::new(),
        tx_ready: Signal<(usize, [u8; 4096])> = Signal::new(),
        tx_ready_usb: Signal<(usize, [u8; 4096])> = Signal::new(),
        arbiter_cantx: Option<Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>> = None
    ])]
    fn init(cx: init::Context) -> (Shared, Resources) {
        // Check for good app and try to bootload
        let mut device = cx.device;
        let mut core: rtic::export::Peripherals = cx.core;
        let pins = bsp::Pins::new(device.port);
        #[cfg(not(feature = "stay-in-bootloader"))]
        {
            let rst_rsn = device.rstc.rcause().read();
            if pins.eeprom_sda.into_floating_input().is_low().unwrap() {
                defmt::warn!("Magic pin connected, staying in bootloader");
            } else if rst_rsn.wdt().bit_is_set() {
                defmt::warn!("Watchdog triggered, staying in bootloader!");
            } else {
                let bl_info = bl_info::get_bootloader_info();
                if can_app_start(bl_info) {
                    let app_addr = MemoryRegion::Application.range_exclusive().start;
                    #[cfg(feature = "skip-app-check")]
                    defmt::warn!("Skip app check enabled - Launching app without verifying");
                    // Start watchdog in bootloader, this way if the CPU freezes,
                    // then the watchdog shall reset, and the bootloader will know!
                    let mut wdt = Watchdog::new(device.wdt);
                    // (1 second / 1024) * period = timeout
                    // 2048 = 2 seconds
                    wdt.start(WatchdogTimeout::Cycles2K as u8);
                    unsafe {
                        core.SCB.invalidate_icache();
                        core.SCB.vtor.write(app_addr);
                        cortex_m::asm::bootload(app_addr as *const u32);
                    }
                }
            }
        }
        // Did not jump to app, start the bootloader diagnostic system
        info!("Bootloader diag start");
        // No app, or we have to stay in bootloader, so now we setup clocks and start the TCU
        // in bootloader mode
        let (mut _buses, clocks, tokens) = clock_system_at_reset(
            device.oscctrl,
            device.osc32kctrl,
            device.gclk,
            device.mclk,
            &mut device.nvmctrl,
        );

        // Clock GCLK0 to 100Mhz
        let (gclk1, dfll) = Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        let gclk1 = gclk1.div(GclkDiv16::Div(24)).enable(); // Gclk1 is now at 2Mhz
        let (clk_dpll0, _gclk1) = Pclk::enable(tokens.pclks.dpll0, gclk1);
        // DPLL0 at 100Mhz (2*50)
        let dpll0 = Dpll::from_pclk(tokens.dpll0, clk_dpll0)
            .loop_div(50, 0)
            .enable();
        let (_gclk0_100, dfll, _dpll0) = clocks.gclk0.swap_sources(dfll, dpll0);
        let (dfll_usb, _old_mode) = dfll.into_mode(FromUsb, |_dfll| {});
        let (gclk2, _dpll0) = Gclk::from_source(tokens.gclks.gclk2, dfll_usb);
        let gclk2_48 = gclk2.enable();
        let mut mclk = unsafe { clocks.pac.steal().3 };

        // Enable the 32Khz clock  and start the RTIC Monotonic driver
        let (osculp32k, _) = OscUlp32k::enable(tokens.osculp32k.osculp32k, clocks.osculp32k_base);
        let _ = RtcOsc::enable(tokens.rtcosc, osculp32k);
        Mono::start(device.rtc);

        let (clk_can, gclk2_48) = Pclk::enable(tokens.pclks.can0, gclk2_48);

        let (can0_deps, gclk2_48) = Dependencies::new(
            gclk2_48,
            clk_can,
            clocks.ahbs.can0,
            pins.can_rx.into_mode(),
            pins.can_tx.into_mode(),
            device.can0,
        );
        let mut can0_cfg =
            mcan::bus::CanConfigurable::new(HertzU32::Hz(500_000), can0_deps, cx.local.message_ram)
                .unwrap();
        // Only 1 filter for CAN Diag Rx ID
        can0_cfg
            .filters_standard()
            .push(Filter::Classic {
                action: mcan::filter::Action::StoreFifo0,
                filter: CAN_ID_RX,
                mask: StandardId::MAX,
            })
            .ok();

        // Enable new MSG interrupt for FIFO0
        let interrupts = can0_cfg
            .interrupts()
            .split([Interrupt::RxFifo0NewMessage].iter().copied().collect())
            .unwrap();
        let line0_interrupts = can0_cfg.interrupt_configuration().enable_line_0(interrupts);

        // USB Init
        let (usb_clock, gclk2_48) = Pclk::enable(tokens.pclks.usb, gclk2_48);
        let usb_bus = UsbBus::new(
            &(usb_clock.into()),
            &mut mclk,
            pins.usb_dm,
            pins.usb_dp,
            device.usb,
        );
        let usb_alloc: &'static _ = cx.local.usb_alloc.insert(UsbBusAllocator::new(usb_bus));

        let uart = SerialPort::new(&usb_alloc);

        let sn = serial_number();
        for b in sn {
            let _ = cx.local.usb_sn.push_str(&format!(2; "{b:02X}").unwrap());
        }

        let usb =
            UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x16c0, 0x27de), cx.local.usb_ctrl_buf)
                .strings(&[StringDescriptors::default()
                    .manufacturer("rnd-ash")
                    .product("Ultimate NAG52 V2 BOOTLOADER")
                    .serial_number(cx.local.usb_sn)])
                .expect("Failed to set strings")
                .device_class(USB_CLASS_CDC)
                .usb_rev(UsbRev::Usb200)
                .build()
                .unwrap();

        let (tx_usb, rx_usb) = cx.local.tx_ready_usb.split();

        let usb_data = UsbData {
            led: pins.led_usb.into(),
            dev: usb,
            usb_serial: uart,
            reading: None,
            tmp: [0; 4096],
            sender: tx_usb,
        };

        // LED status init (Pulsing)
        let (clock_tcc0, _gclk2_48) = Pclk::enable(tokens.pclks.tcc0_tcc1, gclk2_48);
        let pinout = TCC0Pinout::Pd8(pins.led_status);
        let tcc_led = Tcc0Pwm::new(
            &clock_tcc0.into(),
            HertzU32::kHz(1),
            device.tcc0,
            pinout,
            &mut mclk,
        );
        let mut can = can0_cfg.finalize().unwrap();
        let _ = can.tx.cancel_multi(TxBufferSet::all());

        let arbiter_cantx: &'static _ = cx.local.arbiter_cantx.insert(Arbiter::new(can.tx));

        let (isotp_isr, isotp_thread) = make_isotp_endpoint(
            CAN_ID_TX,
            CAN_ID_RX,
            arbiter_cantx,
            cx.local.rx_ready,
            cx.local.tx_ready,
        );

        let nvm = Nvm::new(device.nvmctrl);

        let trng = Trng::new(&mut mclk, device.trng);

        let server = KwpServer::new(nvm, trng);
        diag_task::spawn().unwrap();
        led_task::spawn().unwrap();
        (
            Shared { usb: usb_data },
            Resources {
                tcc_led,
                isotp_usb_rx: rx_usb,
                isotp_thread,
                isotp_isr,
                diag_server: server,
                can0_interrupts: line0_interrupts,
                can0_fifo0: can.rx_fifo_0,
            },
        )
    }

    #[task(priority = 2, shared=[usb], local = [isotp_usb_rx, isotp_thread, diag_server])]
    async fn diag_task(mut cx: diag_task::Context) {
        let diag_task::LocalResources {
            isotp_usb_rx,
            isotp_thread,
            diag_server,
            ..
        } = cx.local;
        let mut is_usb: bool = false;
        loop {
            futures::select_biased! {
                (size, buf) = isotp_thread.read_payload().fuse() => {
                    is_usb = false;
                    let response = diag_server.process_cmd(
                        &buf[..size],
                        Mono::now().duration_since_epoch().to_millis(),
                    );
                    let _ = isotp_thread.write_payload(response).await;
                },
                (size, buf) = isotp_usb_rx.wait().fuse() => {
                    is_usb = true;
                    let response = diag_server.process_cmd(
                        &buf[..size],
                        Mono::now().duration_since_epoch().to_millis(),
                    );
                    cx.shared
                        .usb
                        .lock(|usb| usb.write_payload(response));
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
                    true => cx.shared.usb.lock(|usb| {
                        usb.write_payload(tx);
                    }),
                    false => {
                        let _ = isotp_thread.write_payload(tx).await;
                    }
                }
            }
        }
    }

    #[task(priority = 2, local=[tcc_led])]
    async fn led_task(cx: led_task::Context) {
        const DELAY_MS: u64 = 20;

        let mut i: u32 = 0;
        let max = cx.local.tcc_led.get_max_duty() / 4;
        let step = max as u64 / (2000 / DELAY_MS);
        loop {
            cx.local.tcc_led.set_duty(Channel::_1, i as u32 % max);
            i = i.wrapping_add(step as u32);
            Mono::delay(DELAY_MS.millis()).await;
        }
    }

    #[task(priority = 1, binds=USB_TRCPT0, shared=[usb])]
    fn usb_trcpt0(mut cx: usb_trcpt0::Context) {
        cx.shared.usb.lock(|dev| {
            poll_usb(dev);
        });
    }

    #[task(priority = 1, binds=USB_TRCPT1, shared=[usb])]
    fn usb_trcpt1(mut cx: usb_trcpt1::Context) {
        cx.shared.usb.lock(|dev| {
            poll_usb(dev);
        });
    }

    #[task(priority = 1, binds=USB_OTHER, shared=[usb])]
    fn usb_other(mut cx: usb_other::Context) {
        cx.shared.usb.lock(|dev| {
            poll_usb(dev);
        });
    }

    #[task(priority = 1, binds=CAN0, local=[can0_interrupts, can0_fifo0, isotp_isr])]
    fn can0(cx: can0::Context) {
        for interrupt in cx.local.can0_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for msg in cx.local.can0_fifo0.into_iter() {
                        if msg.id() == Id::Standard(cx.local.isotp_isr.rx_id) {
                            let stmin = ST_MIN_EGS.load(Ordering::Relaxed);
                            let bs = BS_EGS.load(Ordering::Relaxed);
                            cx.local.isotp_isr.on_frame_rx(msg.data(), stmin, bs);
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
