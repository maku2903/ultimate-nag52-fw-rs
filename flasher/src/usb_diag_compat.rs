use std::sync::Arc;

use color_eyre::eyre::Report;
use ecu_diagnostics::channel::{ChannelError, IsoTPChannel, PayloadChannel};
use serial_rs::{SerialPort, SerialPortSettings};

pub struct UsbDiagIface {
    serial: Box<dyn SerialPort>
}

unsafe impl Send for UsbDiagIface{}
unsafe impl Sync for UsbDiagIface{}

impl UsbDiagIface {
    pub fn new(port: &str) -> Result<Self, Report> {

        let settings = SerialPortSettings::default()
            .read_timeout(Some(1000))
            .write_timeout(Some(1000));

        let serial = serial_rs::new_from_path(port, Some(settings))?;
        Ok(Self {
            serial
        })
    }
}

impl IsoTPChannel for UsbDiagIface {
    fn set_iso_tp_cfg(&mut self, _cfg: ecu_diagnostics::channel::IsoTPSettings) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }
}

impl PayloadChannel for UsbDiagIface {
    fn open(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn close(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn set_ids(&mut self, _send: u32, _recv: u32) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn read_bytes(&mut self, _timeout_ms: u32) -> ecu_diagnostics::channel::ChannelResult<Vec<u8>> {
        let mut len_bytes = [0; 2];

        self.serial.read_exact(&mut len_bytes).map_err(|e| ChannelError::IOError(Arc::new(e)))?;
        let len = u16::from_le_bytes(len_bytes) as usize;
        let mut buf = vec![0; len];
        self.serial.read_exact(&mut buf).map_err(|e| ChannelError::IOError(Arc::new(e)))?;
        if buf.is_empty() {
            Err(ChannelError::BufferEmpty)
        } else {
            Ok(buf)
        }
           
    }

    fn write_bytes(
        &mut self,
        _addr: u32,
        _ext_id: Option<u8>,
        buffer: &[u8],
        _timeout_ms: u32,
    ) -> ecu_diagnostics::channel::ChannelResult<()> {
        let mut b = Vec::new();
        b.extend_from_slice(&(buffer.len() as u16).to_le_bytes());
        b.extend_from_slice(buffer);
        self.serial.write_all(&b).map_err(|e| ChannelError::IOError(Arc::new(e)))?;
        Ok(())
    }

    fn clear_rx_buffer(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn clear_tx_buffer(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }
}
