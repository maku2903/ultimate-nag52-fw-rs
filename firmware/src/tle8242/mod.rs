use atsamd_hal::{bind_multiple_interrupts, clock::{Tc2Tc3Clock, Tcc0Tcc1Clock}, dmac::{DmaController, PriorityLevel}, ehal_async::spi::{SpiBus, SpiDevice}, embedded_io_async::Write, gpio::{AlternateF, PA16}, nb::block, pac::{Dmac, Mclk, Pm, Tc2, Tcc1}, prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin, pwm::{Channel, Pwm0, Pwm2, TC2Pinout, TCC1Pinout, Tcc1Pwm}, sercom::{spi::{self, Duplex, Flags, Spi, SpiFuture, SpiFutureDuplex, SpiFutureDuplexDma}, Sercom2}, time::Hertz, typelevel::NoneT};
use bsp::{pin_alias, TleClk, TleCs, TleReset, TleSpi, TleSpiPads};
use commands::{AvgCurrentRead, ControlMethodFaultMaskConfig, CurrentDitherAmplitudeSet, ICVersion, MsgId, MsgIdWithChannel, TleHeader, TleMessage, TleMessagePayload};
use cortex_m::{prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write}, register::control::Control};
use embedded_hal::{pwm::SetDutyCycle};

bind_multiple_interrupts!(struct SpiIrqs {
    SERCOM2: [SERCOM2_0, SERCOM2_1, SERCOM2_2, SERCOM2_3, SERCOM2_OTHER] => atsamd_hal::sercom::spi::InterruptHandler<Sercom2>;
});

atsamd_hal::bind_multiple_interrupts!(struct DmacIrqs {
    DMAC: [DMAC_0, DMAC_1, DMAC_2, DMAC_OTHER] => atsamd_hal::dmac::InterruptHandler;
});

use atsamd_hal::prelude::_embedded_hal_Pwm;
use embedded_hal_nb::spi::FullDuplex;
use packed_struct::{types::bits::ByteArray, PackedStructSlice};
use rtt_target::rprintln;

pub mod commands;

pub struct Tle8242 {
    spi: SpiFutureDuplex<spi::Config<TleSpiPads>>,
    cs: TleCs
}

#[repr(u8)]
pub enum TleChannel {
    _1 = 0,
    _2 = 1,
    _3 = 2,
    _4 = 3,
    _5 = 4,
    _6 = 5,
    _7 = 6,
    _8 = 7
}

const R_SENSE: f32 = 0.05;
const R_SENSE_EQ_VAL: f32 = 320.0 / 0.05;

const TLE_CLOCK_FREQ: Hertz = Hertz::MHz(25);

impl Tle8242 {
    pub fn new(
        mut spi: TleSpi, 
        pm: &mut Pm,
        clk_pin: impl Into<TleClk>, // TC2 - WO0
        reset_pin: impl Into<TleReset>,
        cs_pin: impl Into<TleCs>,
        clock: &Tcc0Tcc1Clock,
        tcc: Tcc1,
        mclk: &mut Mclk
    ) -> Self {
        let mut pin: TleClk = clk_pin.into();
        pin.set_drive_strength(true);
        let mut pin_reset: TleReset = reset_pin.into();
        pin_reset.set_drive_strength(true);
        pin_reset.set_low().unwrap();
        rprintln!("Clock is {}", clock.freq());
        let mut pwm_tcc: Tcc1Pwm<PA16, AlternateF> = Tcc1Pwm::new(
            clock, 
            TLE_CLOCK_FREQ,
            tcc, 
            TCC1Pinout::Pa16(pin),
            mclk
        );
        let max = pwm_tcc.get_max_duty();
        pwm_tcc.set_duty(Channel::_0, max/2);
        rprintln!("Max duty is {}", max);
        let spi = spi.into_future(SpiIrqs);
        pin_reset.set_high().unwrap(); // Since we are done initializing
        let mut cs_pin = cs_pin.into();
        cs_pin.set_drive_strength(true);
        Self {
            spi,
            cs: cs_pin
        }
    }


    pub async fn send_command(&mut self, cmd: u32) -> Option<u32> {
        let mut buf: [u8; 4] = cmd.to_le_bytes().try_into().unwrap();
        self.spi.transfer_in_place(&mut buf).await.ok()?;
        Some(u32::from_le_bytes(buf.try_into().unwrap()))
    }

    pub async fn write_and_read(&mut self, buf: &mut [u8; 4]) -> Option<()> {
        self.spi.transfer_in_place(buf).await.ok()
    }

    pub async fn test(&mut self, u: u32) {
        let mut buf = [0u8; 4];
        buf = u.to_le_bytes();
        rprintln!("OUT: {:02X?}", buf);
        self.spi.transfer_in_place(&mut buf).await.unwrap();
        rprintln!("IN: {:02X?}", buf);
    }

    pub async fn read_version_info(&mut self) -> Option<ICVersion> {
        let q = ICVersion::default();
        let header = MsgId::new(ICVersion::MSG_ID);
        self.read_data(header, q).await
    }

    pub async fn read_control_method(&mut self) -> Option<ControlMethodFaultMaskConfig> {
        let q = ControlMethodFaultMaskConfig::default();
        let header = MsgId::new(ControlMethodFaultMaskConfig::MSG_ID);
        self.read_data(header, q).await
    }

    pub async fn set_current(&mut self, channel: TleChannel, ma: u16) -> Option<()> {

        let spi_val: f32 = ma as f32 / R_SENSE_EQ_VAL;
        let setpoint_val = (spi_val * 2048.0) as u16;
        rprintln!("Setpoint val calc is {}", setpoint_val);

        let mut q = CurrentDitherAmplitudeSet::default();
        q.current_setpoint = setpoint_val;
        q.en = true;
        q.dither_step_size = 0;
        let header = MsgIdWithChannel::new(CurrentDitherAmplitudeSet::MSG_ID, channel);
        let resp = self.write_data(header, q).await?;
        Some(())

    }

    #[inline(always)]
    pub async fn read_data<H: TleHeader, MSG: TleMessage>(&mut self, header: H, data: MSG) -> Option<MSG> {
        self.send_and_receive(header, data, false).await
    }

    #[inline(always)]
    pub async fn write_data<H: TleHeader, MSG: TleMessage>(&mut self, header: H, data: MSG) -> Option<MSG> {
        self.send_and_receive(header, data, true).await
    }

    pub async fn send_and_receive<H: TleHeader, MSG: TleMessage>(&mut self, header: H, data: MSG, write: bool) -> Option<MSG> {
        let mut full_msg = TleMessagePayload::default();
        full_msg.msg_id = header.pack().unwrap().as_bytes_slice()[0];
        full_msg.r_w = write;
        data.pack_to_slice(&mut full_msg.payload).unwrap();
        let mut buf = [0u8; 4];
        full_msg.pack_to_slice(&mut buf).unwrap();
        rprintln!("Out to TLE: {:08b} {:02X?}", buf[0], buf);
        self.cs.set_low().unwrap();
        let result = if self.write_and_read(&mut buf).await.is_some() {
            let response = TleMessagePayload::unpack_from_slice(&buf).unwrap();
            rprintln!("IN: {:02X?}", buf);
            if response.msg_id == full_msg.msg_id {
                let data = MSG::unpack_from_slice(&response.payload).unwrap();
                Some(data)
            } else {
                rprintln!("TLE MESSAGE ID DID NOT MATCH!");
                None
            }
        } else {
            rprintln!("TLE SPI transfer failed!");
            None
        };
        self.cs.set_high().unwrap();
        result
    }
}

