use core::cell::RefCell;

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
use usbd_serial::{embedded_io::Write, DefaultBufferStore, UsbError};

pub static USB_CTRL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
pub static USB_ALLOCATOR: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();
pub static USB: Mutex<RefCell<Option<UsbData<'static>>>> = Mutex::new(RefCell::new(None));
pub static USB_SN: StaticCell<heapless::String<32>> = StaticCell::new();

pub struct UsbData<'a> {
    pub led: LedUsb,
    pub dev: UsbDevice<'a, UsbBus>,
    pub usb_serial: usbd_serial::SerialPort<'a, UsbBus, DefaultBufferStore, DefaultBufferStore>,
}

#[derive(Default)]
pub struct UsbIsoTp {
    // Expected size, current pos
    reading: Option<(usize, usize)>,
}

impl UsbIsoTp {
    pub fn read_payload(&mut self, dest: &mut [u8]) -> Option<usize> {
        let mut ret = None;
        match &mut self.reading {
            None => {
                if let Ok(2) = read_usb(&mut dest[..2]) {
                    let size = u16::from_le_bytes(dest[..2].try_into().unwrap());
                    if size > 0 && size <= dest.len() as u16 {
                        // Size check OK
                        self.reading = Some((size as usize, 0));
                    }
                }
            }
            Some((expected_size, pos)) => {
                if let Ok(size) = read_usb(&mut dest[*pos..*expected_size]) {
                    *pos += size;
                    if *expected_size == *pos {
                        ret = Some(*pos);
                        self.reading = None;
                    }
                }
            }
        }
        ret
    }

    pub fn write_payload(&self, buf: &[u8]) {
        write_usb(buf);
    }
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
pub fn write_usb(data: &[u8]) {
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
