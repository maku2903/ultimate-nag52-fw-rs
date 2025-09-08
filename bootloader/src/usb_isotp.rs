use atsamd_hal::{
    ehal::digital::OutputPin,
    usb::{usb_device::device::UsbDevice, UsbBus},
};

use bsp::LedUsb;
use rtic_sync::signal::SignalWriter;
use usbd_serial::{embedded_io::Write, DefaultBufferStore};

pub struct UsbData<'a> {
    pub led: LedUsb,
    pub dev: UsbDevice<'a, UsbBus>,
    pub usb_serial: usbd_serial::SerialPort<'a, UsbBus, DefaultBufferStore, DefaultBufferStore>,
    pub reading: Option<(usize, usize)>,
    pub sender: SignalWriter<'static, (usize, [u8; 4096])>,
    pub tmp: [u8; 4096],
}

impl<'a> UsbData<'a> {
    pub fn write_payload(&mut self, buf: &[u8]) {
        let _ = self.led.set_high();
        let size = (buf.len() as u16).to_le_bytes();
        let _ = self.usb_serial.write_all(&size);
        let _ = self.usb_serial.write_all(buf);
        let _ = self.led.set_low();
    }
}

pub(crate) fn poll_usb(usb: &mut UsbData) {
    let _ = usb.led.set_high();
    if usb.dev.poll(&mut [&mut usb.usb_serial]) {
        if usb.reading == None {
            let mut tmp = [0; 2];
            if let Ok(2) = usb.usb_serial.read(&mut tmp) {
                let size = u16::from_le_bytes(tmp) & 0xFFF;
                if size != 0 {
                    // Size check OK
                    usb.reading = Some((size as usize, 0));
                }
            }
        }
        if let Some((expected_size, pos)) = &mut usb.reading {
            while let Ok(size) = usb.usb_serial.read(&mut usb.tmp[*pos..*expected_size]) {
                *pos += size;
                if *expected_size == *pos {
                    let _ = usb.sender.write((*expected_size, usb.tmp.clone()));
                    usb.reading = None;
                    break;
                }
            }
        }
    }
    let _ = usb.led.set_low();
}
