use core::cell::{Cell, RefCell};

use atsamd_hal::{
    ehal::digital::OutputPin,
    usb::{
        usb_device::{bus::UsbBusAllocator, device::UsbDevice},
        UsbBus,
    },
};

use bsp::LedUsb;
use cortex_m::interrupt::{free, Mutex};
use static_cell::StaticCell;
use usbd_serial::{embedded_io::Write, UsbError};

pub static USB_ISOTP_RX: StaticCell<[u8; 4096]> = StaticCell::new();
pub static USB_ISOTP_TX: StaticCell<[u8; 4096]> = StaticCell::new();
pub static USB_CTRL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
pub static USB_ALLOCATOR: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();
pub static USB: Mutex<RefCell<Option<UsbData<'static>>>> = Mutex::new(RefCell::new(None));

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum UsbIsotpState {
    Idle,
    Reading { expected_size: usize, pos: usize },
}

pub struct UsbData<'a> {
    pub led: LedUsb,
    pub dev: UsbDevice<'a, UsbBus>,
    pub usb_serial: usbd_serial::SerialPort<'a, UsbBus, &'a mut [u8], &'a mut [u8]>,
}

#[unsafe(link_section = ".data")]
pub(crate) fn poll_usb() {
    free(|cs| {
        if let Some(usb_data) = USB.borrow(cs).borrow_mut().as_mut() {
            usb_data.led.set_high().unwrap();
            usb_data.dev.poll(&mut [&mut usb_data.usb_serial]);
            usb_data.led.set_low().unwrap();
        }
    });
}

#[unsafe(link_section = ".data")]
pub(crate) fn write_usb(data: &[u8]) {
    free(|cs| {
        if let Some(usb_data) = USB.borrow(cs).borrow_mut().as_mut() {
            let size = (data.len() as u16).to_le_bytes();
            let _ = usb_data.usb_serial.write_all(&size);
            let _ = usb_data.usb_serial.write_all(data);
        }
    })
}

#[unsafe(link_section = ".data")]
#[inline]
fn read_usb(buf: &mut [u8]) -> usbd_serial::Result<usize> {
    free(|cs| {
        if let Some(usb_data) = USB.borrow(cs).borrow_mut().as_mut() {
            usb_data.usb_serial.read(buf)
        } else {
            Err(UsbError::WouldBlock)
        }
    })
}

#[unsafe(link_section = ".data")]
pub(crate) fn read_payload(dest: &mut [u8], state: &mut UsbIsotpState) -> Option<usize> {
    let mut ret = None;
    if *state == UsbIsotpState::Idle {
        // Try to read header
        if let Ok(2) = read_usb(&mut dest[..2]) {
            let size = u16::from_le_bytes(dest[..2].try_into().unwrap());
            if size > 0 && size <= dest.len() as u16 {
                // Size check OK
                *state = UsbIsotpState::Reading {
                    expected_size: size as usize,
                    pos: 0,
                }
            }
        }
    }
    if let UsbIsotpState::Reading { expected_size, pos } = state {
        if let Ok(size) = read_usb(&mut dest[*pos..*expected_size]) {
            *pos += size;
            if *expected_size == *pos {
                ret = Some(*pos);
                *state = UsbIsotpState::Idle;
            }
        }
    }
    ret
}
