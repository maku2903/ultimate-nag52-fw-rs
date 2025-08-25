#![no_std]
#![no_main]

use core::{borrow::BorrowMut, panic::PanicInfo};

use bsp::LedStatus;

use atsamd_hal::{
    can::Dependencies,
    clock::v2::{
        clock_system_at_reset,
        dfll::FromUsb,
        dpll::Dpll,
        gclk::{Gclk, GclkDiv16},
        pclk::Pclk,
    },
    ehal::digital::{InputPin, OutputPin},
    fugit::HertzU32,
    nvm::Nvm,
    pac::{self, Peripherals},
    trng::Trng,
    usb::{
        usb_device::{
            bus::UsbBusAllocator,
            device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid},
        },
        UsbBus,
    },
};

use cortex_m::{asm::delay, interrupt::free, peripheral::NVIC};
use cortex_m_rt::entry;
use mcan::{
    embedded_can::StandardId, filter::Filter, message::Raw, messageram::SharedMemory,
    rx_fifo::DynRxFifo,
};

mod bl_info;
pub mod isotp;
pub mod kwp;
mod usb_isotp;
use usb_isotp::*;

use defmt_rtt as _;
use systick_timer::Timer;
use usbd_serial::SerialPort;

use crate::{
    bl_info::{app_crc, APP_ADDR_START},
    isotp::IsoTp,
    kwp::KwpServer,
};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    loop {}
    //SCB::sys_reset()
}

pub const CAN_ID_TX: StandardId = unsafe { StandardId::new_unchecked(0x7E9) };
pub const CAN_ID_RX: StandardId = unsafe { StandardId::new_unchecked(0x7E1) };

#[link_section = ".can"]
static mut SHARED_MEM_CAN0: SharedMemory<bsp::can_deps::Capacities> = SharedMemory::new();

// Set up for micro-second resolution, reload every 1 milliseond, 48 MHz clock
pub static INSTANCE: Timer = Timer::new(1000, 4799, 48_000_000);

#[cortex_m_rt::exception]
fn SysTick() {
    INSTANCE.systick_handler();
}

#[entry]
fn main() -> ! {
    // Check for good app and try to bootload
    let mut bsp_peripherals = Peripherals::take().unwrap();
    let pins = bsp::Pins::new(bsp_peripherals.port);
    let mut core_peripehrals = cortex_m::Peripherals::take().unwrap();
    let bl_info = bl_info::get_bootloader_info();
    delay(1000);
    if pins.eeprom_sda.into_floating_input().is_low().unwrap() {
        defmt::warn!("Magic pin connected, staying in bootloader");
    } else if cfg!(feature = "stay-in-bootloader") {
        defmt::warn!("Staying in bootloader due to compile flag");
    } else if bl_info.flashing_not_done == 0 {
        let app_crc = app_crc();
        if app_crc == bl_info.application_crc {
            defmt::info!("Can jump to app! - Valid data");
            unsafe {
                core_peripehrals.SCB.invalidate_icache();
                core_peripehrals.SCB.vtor.write(APP_ADDR_START);
                cortex_m::asm::bootload(APP_ADDR_START as *const u32);
            }
        } else {
            defmt::error!("Cannot jump to app - CRC Invalid!");
        }
    } else {
        defmt::error!("Cannot jump to app - Flashing not done");
    }

    // Did not jump to app, start the bootloader diagnostic system

    // No app, or we have to stay in bootloader, so now we setup clocks and start the TCU
    // in bootloader mode
    let (mut _buses, clocks, tokens) = clock_system_at_reset(
        bsp_peripherals.oscctrl,
        bsp_peripherals.osc32kctrl,
        bsp_peripherals.gclk,
        bsp_peripherals.mclk,
        &mut bsp_peripherals.nvmctrl,
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
    INSTANCE.start(&mut core_peripehrals.SYST);

    let (clk_can, gclk2_48) = Pclk::enable(tokens.pclks.can0, gclk2_48);

    let (can0_deps, gclk2_48) = Dependencies::new(
        gclk2_48,
        clk_can,
        clocks.ahbs.can0,
        pins.can_rx.into_mode(),
        pins.can_tx.into_mode(),
        bsp_peripherals.can0,
    );
    let mut can0_cfg = mcan::bus::CanConfigurable::new(HertzU32::Hz(500_000), can0_deps, unsafe {
        &mut *(&raw mut SHARED_MEM_CAN0)
    })
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

    // USB Init
    let (usb_clock, _gclk2_48) = Pclk::enable(tokens.pclks.usb, gclk2_48);
    let usb_bus = UsbBus::new(
        &(usb_clock.into()),
        &mut mclk,
        pins.usb_dm,
        pins.usb_dp,
        bsp_peripherals.usb,
    );
    let usb_alloc = USB_ALLOCATOR.init(UsbBusAllocator::new(usb_bus));
    let ctrl_buf = USB_CTRL_BUF.init([0; 256]);

    let tx_buf = USB_ISOTP_TX.init([0; 4096]);
    let rx_buf = USB_ISOTP_RX.init([0; 4096]);

    let uart = SerialPort::new_with_store_and_interface_names(
        usb_alloc,
        tx_buf.borrow_mut(),
        rx_buf.borrow_mut(),
        Some("ULTIMATE-NAG52 COMM"),
        Some("ULTIMATE-NAG52 DATA"),
    );

    let usb = UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x16c0, 0x27de), ctrl_buf)
        .strings(&[StringDescriptors::default()
            .manufacturer("rnd-ash")
            .product("Ultimate NAG52 V2")
            .serial_number("BOOTLOADER")])
        .expect("Failed to set strings")
        .device_class(0x08)
        .max_packet_size_0(64)
        .unwrap()
        .usb_rev(UsbRev::Usb200)
        .build()
        .unwrap();

    unsafe {
        free(|cs| {
            let mut r = USB.borrow(cs).borrow_mut();
            *r = Some(UsbData {
                led: pins.led_usb.into(),
                dev: usb,
                usb_serial: uart,
            });

            core_peripehrals
                .NVIC
                .set_priority(pac::interrupt::USB_OTHER, 1);
            core_peripehrals
                .NVIC
                .set_priority(pac::interrupt::USB_TRCPT0, 1);
            core_peripehrals
                .NVIC
                .set_priority(pac::interrupt::USB_TRCPT1, 1);
            NVIC::unmask(pac::interrupt::USB_OTHER);
            NVIC::unmask(pac::interrupt::USB_TRCPT0);
            NVIC::unmask(pac::interrupt::USB_TRCPT1);
        });
    }

    let mut can = can0_cfg.finalize().unwrap();
    let mut can_tx = can.tx;
    let mut isotp = IsoTp::default();
    let nvm = Nvm::new(bsp_peripherals.nvmctrl);

    let trng = Trng::new(&mut mclk, bsp_peripherals.trng);

    let mut server = KwpServer::new(nvm, trng);
    let mut diag_buffer = [0u8; 4096];
    let mut req_len: Option<usize>;

    let mut stat_led: LedStatus = pins.led_status.into();
    let mut is_usb = false;
    let mut isotp_usb_state = UsbIsotpState::Idle;
    loop {
        req_len = None;
        if let Ok(packet) = can.rx_fifo_0.receive() {
            if let Some(isotp_rx) = isotp.on_packet_rx(packet.data(), &mut can_tx) {
                defmt::debug!("CAN ISOTP Rx: {:02X}", isotp_rx);
                diag_buffer[..isotp_rx.len()].copy_from_slice(isotp_rx);
                is_usb = false;
                req_len = Some(isotp_rx.len());
            }
        } else if let Some(size) = read_payload(&mut diag_buffer, &mut isotp_usb_state) {
            is_usb = true;
            req_len = Some(size)
        }

        if let Some(len) = req_len {
            stat_led.set_high().unwrap();
            defmt::debug!("KWP IN: {:02X}", diag_buffer[..len]);
            let response = server.process_cmd(&diag_buffer[..len], INSTANCE.now());
            stat_led.set_low().unwrap();
            match is_usb {
                true => write_usb(response),
                false => isotp.write_payload(response, &mut can_tx),
            }
        }
        // Do this after Txing on CAN
        if let Some(tx) = server.update(INSTANCE.now()) {
            match is_usb {
                true => write_usb(tx),
                false => isotp.write_payload(tx, &mut can_tx),
            }
        }
    }
}

use pac::interrupt;

#[interrupt]
fn USB_OTHER() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT0() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT1() {
    poll_usb();
}
