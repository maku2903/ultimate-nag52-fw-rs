//! USB implementation

use core::{cell::{OnceCell, RefCell, UnsafeCell}, net::Ipv4Addr, ops::{Deref, DerefMut}, sync::atomic::AtomicI32};

use atsamd_hal::{clock::v2::{gclk::GclkId, pclk::Pclk, types::Usb}, ehal::digital::OutputPin, pac::{self, Mclk, Peripherals, NVIC}, serial_number, usb::{usb_device::{bus::UsbBusAllocator, device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid}, LangID}, UsbBus}};
use bsp::{LedUsb, UsbDm, UsbDp};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, once_lock::OnceLock};
use pac::interrupt;
use smoltcp::{iface::{self, Interface, SocketHandle, SocketSet}, socket::{dhcpv4::{self, RetryConfig}, tcp, AnySocket}, time::{Duration, Instant}, wire::{DhcpOption, EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr}};
use static_cell::StaticCell;
use usbd_ethernet::{DeviceState, Ethernet};
use usbd_serial::{SerialPort, UsbError, USB_CLASS_CDC};
use usbd_storage::{subclass::scsi::Scsi, transport::bbb::BulkOnly};
use core::fmt::Write;

use crate::{extern_flash::{self, reset_scsi_state, MAX_LUN, USB_PACKET_SIZE, USB_TRANSPORT_BUF}};
pub mod uart;


static SERIAL_STR: StaticCell<heapless::String<32>> = StaticCell::new();
static EP_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static USB_CTRL_BUF: StaticCell<[u8; 256]> = StaticCell::new();

static I: AtomicI32 = AtomicI32::new(0);

pub struct UsbPeripherals<'a> {
    pub dev: UsbDevice<'a, UsbBus>,
    pub led: LedUsb,
    pub uart: SerialPort<'a, UsbBus>,
    pub scsi: Scsi<BulkOnly<'a, UsbBus, &'a mut [u8]>>
}

static USB_ALLOCATOR: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();
static USB: OnceLock<embassy_sync::mutex::Mutex<CriticalSectionRawMutex, UsbPeripherals<'static>>> = OnceLock::new();


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

fn poll_usb() {
    if let Some(mut lock) = USB.try_get().map(|x| x.try_lock().ok()).flatten() {
        let usb = lock.deref_mut();
        if usb.dev.poll(&mut [&mut usb.uart, &mut usb.scsi]) {
            usb.led.set_high().unwrap();
            if usb.dev.state() == UsbDeviceState::Default || usb.dev.state() == UsbDeviceState::Suspend {
                reset_scsi_state();
            }
            let _ = usb.uart.read(&mut []);

            let _ = usb.scsi.poll(|cmd| {
                let _ = extern_flash::process_scsi_command(cmd);
            });
            usb.led.set_low().unwrap();
        }
    }
}

pub fn usb_init<Gclk: GclkId>(usb: pac::Usb, clock: Pclk<Usb, Gclk>, mclk: &mut Mclk, dp: impl Into<UsbDp>, dm: impl Into<UsbDm>, nvic: &mut NVIC, led_usb: impl Into<LedUsb>) {
    let sn = serial_number();
    let str = SERIAL_STR.init(heapless::String::<32>::new());
    for c in sn {
        let mut buf = heapless::String::<2>::new();
        write!(buf, "{c:02X}").unwrap();
        str.push_str(&buf).unwrap();
    }

    let mut usb_alloc = USB_ALLOCATOR.init(UsbBusAllocator::new(bsp::native_usb(clock, usb, mclk, dp, dm)));

    let mut uart = SerialPort::new_with_interface_names(
        usb_alloc,
        Some("ULTIMATE-NAG52 COMM"),
        Some("ULTIMATE-NAG52 DATA")
    );
    let usb_scsi_buf = USB_TRANSPORT_BUF.init([0; 4096]);
    let mut scsi = usbd_storage::subclass::scsi::Scsi::new(usb_alloc, USB_PACKET_SIZE as u16, MAX_LUN, usb_scsi_buf.as_mut_slice()).unwrap();

    let ctrl_buf = USB_CTRL_BUF.init([0; 256]);

    let mut bus = UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x16c0, 0x27dd), ctrl_buf)
        .strings(&[StringDescriptors::new(LangID::EN)
                    .manufacturer("rnd-ash")
                    .product("Ultimate NAG52 V2")
                    .serial_number(str)])
                .expect("Failed to set strings") // CDC
                .composite_with_iads()
                .self_powered(true)
                .usb_rev(atsamd_hal::usb::usb_device::device::UsbRev::Usb200)
                .build()
                .unwrap();

    defmt::info!("USB Ready");
    // Do this before we enable the USB interrupts
    let _ = USB.init(Mutex::new(UsbPeripherals {
        dev: bus,
        led: led_usb.into(),
        uart,
        scsi,
    }));

    unsafe {
        nvic.set_priority(interrupt::USB_OTHER, 1);
        nvic.set_priority(interrupt::USB_TRCPT0, 1);
        nvic.set_priority(interrupt::USB_TRCPT1, 1);
        NVIC::unmask(interrupt::USB_OTHER);
        NVIC::unmask(interrupt::USB_TRCPT0);
        NVIC::unmask(interrupt::USB_TRCPT1);
    }
}