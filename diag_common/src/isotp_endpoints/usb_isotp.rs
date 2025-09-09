use core::borrow::BorrowMut;

use rtic_sync::{
    arbiter::Arbiter,
    signal::{SignalReader, SignalWriter},
};

use usbd_serial::embedded_io::{self, Write};

use crate::isotp_endpoints::SharedIsoTpBuf;

pub struct UsbIsoTpInterruptHandler<
    'a,
    B: usb_device::bus::UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
    const N: usize,
> {
    serial: &'a Arbiter<usbd_serial::SerialPort<'a, B, RS, WS>>,
    reading: Option<(SharedIsoTpBuf<N>, usize)>,
    sender: SignalWriter<'static, SharedIsoTpBuf<N>>,
}

impl<'a, B: usb_device::bus::UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>, const N: usize>
    UsbIsoTpInterruptHandler<'a, B, RS, WS, N>
{
    pub fn poll(&mut self) {
        if let Some(serial) = self.serial.try_access().as_mut() {
            if self.reading.is_none() {
                let mut tmp = [0; 2];
                if let Ok(2) = serial.read(&mut tmp) {
                    let size = u16::from_le_bytes(tmp) & 0xFFF;
                    if size != 0 && size <= N as u16 {
                        // Size check OK
                        let tmp = SharedIsoTpBuf::new();
                        self.reading = Some((tmp, size as usize));
                    }
                }
            }
            if let Some((buffer, expected_size)) = &mut self.reading {
                while let Ok(size) = serial.read(&mut buffer.data[buffer.size..*expected_size]) {
                    buffer.size += size;
                    if buffer.size == *expected_size {
                        self.sender.write(*buffer);
                        self.reading = None;
                        break;
                    }
                }
            }
        }
    }
}

pub struct UsbIsoTpConsumer<
    'a,
    B: usb_device::bus::UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
    const N: usize,
> {
    serial: &'a Arbiter<usbd_serial::SerialPort<'a, B, RS, WS>>,
    pending_msg: SignalReader<'a, SharedIsoTpBuf<N>>,
}

impl<'a, B: usb_device::bus::UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>, const N: usize>
    UsbIsoTpConsumer<'a, B, RS, WS, N>
{
    pub async fn read(&mut self) -> SharedIsoTpBuf<N> {
        self.pending_msg.wait().await
    }

    // TODO - usbd_serial needs to expose io module to allow this to return the right type
    pub async fn write<E: embedded_io::Error>(&mut self, buffer: &[u8]) -> Result<(), ()> {
        if buffer.len() > N {
            return Err(());
        }
        let size = (buffer.len() as u16).to_le_bytes();
        let mut serial = self.serial.access().await;
        serial.write_all(&size).map_err(|_| ())?;
        serial.write_all(buffer).map_err(|_| ())?;
        Ok(())
    }
}
