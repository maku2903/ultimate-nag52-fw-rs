use atsamd_hal::{async_hal::interrupts::{TC5, TC7}, bind_multiple_interrupts, clock::{Tc2Tc3Clock, Tc6Tc7Clock, Tcc0Tcc1Clock}, dmac::{DmaController, PriorityLevel}, ehal_async::spi::{SpiBus, SpiDevice}, embedded_io_async::Write, gpio::{AlternateF, PA16}, nb::block, pac::{Dmac, Mclk, Pm, Tc2, Tc7, Tcc0, Tcc1}, prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin, pwm::{Channel, Pwm0, Pwm2, Pwm7, TC2Pinout, TC7Pinout, TCC0Pinout, TCC1Pinout, Tcc0Pwm, Tcc1Pwm}, sercom::{spi::{self, Duplex, Flags, Spi, SpiFuture, SpiFutureDuplex, SpiFutureDuplexDma}, Sercom2, Sercom6}, time::Hertz, timer::{TimerCounter, TimerCounter7}, typelevel::NoneT};
use bsp::{pin_alias, LedTleAct, TleClk, TleClkG, TleCs, TlePhaseSync, TleReset, TleSpi, TleSpiPads};
use commands::{AvgCurrentRead, ControlMethodFaultMaskConfig, ControlVariableSet, CurrentDitherAmplitudeSet, ICVersion, MainPeriodSet, MaxMinCurrentRead, MsgId, MsgIdWithChannel, PwmDutyCycle, TleHeader, TleMessage, TleMessagePayload};
use cortex_m::{prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write}, register::control::Control};
use embedded_hal::{pwm::SetDutyCycle};

mod solenoids;

bind_multiple_interrupts!(struct SpiIrqs {
    SERCOM6: [SERCOM6_0, SERCOM6_1, SERCOM6_2, SERCOM6_3, SERCOM6_OTHER] => atsamd_hal::sercom::spi::InterruptHandler<Sercom6>;
});

atsamd_hal::bind_multiple_interrupts!(struct DmacIrqs {
    DMAC: [DMAC_0, DMAC_1, DMAC_2, DMAC_OTHER] => atsamd_hal::dmac::InterruptHandler;
});

use atsamd_hal::prelude::_embedded_hal_Pwm;
use embedded_hal_nb::spi::FullDuplex;
use packed_struct::{types::bits::ByteArray, PackedStruct, PackedStructSlice};
use rtt_target::rprintln;

pub mod commands;

pub struct Tle8242 {
    spi: SpiFutureDuplex<spi::Config<TleSpiPads>>,
    cs: TleCs,
    act_led: LedTleAct,
    ps: TlePhaseSync
}

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum TleChannel {
    _0 = 0,
    _1 = 1,
    _2 = 2,
    _3 = 3,
    _4 = 4,
    _5 = 5,
    _6 = 6,
    _7 = 7
}

const R_SENSE: f32 = 0.05;
const R_SENSE_EQ_VAL: f32 = 320.0 / 0.05;

const TLE_CLOCK_FREQ: Hertz = Hertz::MHz(25);

impl Tle8242 {
    pub fn new(
        mut spi: TleSpi, 
        clk_pin: impl Into<TleClkG>,
        reset_pin: impl Into<TleReset>,
        phase_pin: impl Into<TlePhaseSync>,
        cs_pin: impl Into<TleCs>,
        clock: &Tcc0Tcc1Clock,
        tcc: Tcc0,
        led: impl Into<LedTleAct>,
        mclk: &mut Mclk
    ) -> Self {
        let mut clk_pin = clk_pin.into();
        clk_pin.set_drive_strength(false);
        let pinout = TCC0Pinout::Pa20(clk_pin);
        let mut tcc = Tcc0Pwm::new(clock, Hertz::kHz(40_000), tcc, pinout, mclk);
        rprintln!("Max duty: {} - {}", tcc.get_max_duty(), clock.freq().raw());
        tcc.set_duty(Channel::_0, 2);
        tcc.enable(Channel::_0);
        let mut phase_pin: TlePhaseSync = phase_pin.into();
        let mut pin_reset: TleReset = reset_pin.into();
        pin_reset.set_drive_strength(false);
        pin_reset.set_low().unwrap();
        let spi = spi.into_future(SpiIrqs);
        pin_reset.set_high().unwrap(); // Since we are done initializing
        let mut cs_pin = cs_pin.into();
        cs_pin.set_drive_strength(false);
        phase_pin.set_drive_strength(false);
        phase_pin.set_low().unwrap();
        Self {
            spi,
            cs: cs_pin,
            ps: phase_pin,
            act_led: led.into()
        }
    }


    pub async fn send_command(&mut self, cmd: &[u8; 4]) -> Option<[u8; 4]> {
        self.act_led.set_high().unwrap();
        let mut res: [u8; 4] = [0 ;4];
        self.cs.set_low().unwrap();
        cortex_m::asm::delay(150);
        self.spi.write(cmd).await.ok()?;
        cortex_m::asm::delay(50);
        self.cs.set_high().unwrap();

        cortex_m::asm::delay(500);

        self.cs.set_low().unwrap();
        cortex_m::asm::delay(150);
        self.spi.read(&mut res).await.ok()?;
        cortex_m::asm::delay(50);
        self.cs.set_high().unwrap();
        self.act_led.set_low().unwrap();
        Some(res)
    }

    pub async fn send_command_batch<const N: usize>(&mut self, cmd: [[u8; 4]; N]) -> Option<[[u8; 4]; N]> {
        let mut res = [[0,0,0,0]; N];
        // Send the first command request
        self.cs.set_low().unwrap();
        cortex_m::asm::delay(150);
        self.spi.write(&cmd[0]).await.ok()?;
        cortex_m::asm::delay(50);
        self.cs.set_high().unwrap();

        // Iterate over the middle messages, the last message response will be read below this
        for i in 1..N-1 {
            cortex_m::asm::delay(500);
            // Transfer in place for middle messages
            let req = cmd[i];
            res[i-1] = req; // Write request to i-1's response. This allows transfer in place to work
            self.cs.set_low().unwrap();
            cortex_m::asm::delay(150);
            self.spi.transfer_in_place(&mut res[i-1]).await.ok()?;
            cortex_m::asm::delay(50);
            self.cs.set_high().unwrap();
        }
        // Read the final message response by doing SPI read
        cortex_m::asm::delay(500);
        self.cs.set_low().unwrap();
        cortex_m::asm::delay(150);
        self.spi.read(&mut res[N-1]).await.ok()?;
        cortex_m::asm::delay(50);
        self.cs.set_high().unwrap();

        Some(res)
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

    pub async fn read_current(&mut self, channel: TleChannel) -> Option<f32> {
        let header = MsgIdWithChannel::new(AvgCurrentRead::MSG_ID, channel);
        let q = AvgCurrentRead::default();
        let response = self.read_data(header, q).await.unwrap();
        if response.valid {
            Some((response.avg as f32 / (2u32.pow(13) as f32)) * R_SENSE_EQ_VAL)
        } else {
            None
        }
    }

    pub async fn configure_channel(&mut self, channel: TleChannel) {

    }

    pub async fn set_current(&mut self, channel: TleChannel, ma: u16) -> Option<()> {
        // Set Control method on channel 3
        let header = MsgId::new(ControlMethodFaultMaskConfig::MSG_ID);
        let mut q = ControlMethodFaultMaskConfig::default();
        q.cm3 = false;
        self.write_data(header, q).await.unwrap();

        //let header = MsgIdWithChannel::new(MainPeriodSet::MSG_ID, channel);
        //let mut q = MainPeriodSet::default();
        //q.divider_n = 160;
        //q.divider_m = 2;
        //rprintln!("{:#?}",self.write_data(header, q).await.unwrap());

        //let header = MsgIdWithChannel::new(PwmDutyCycle::MSG_ID, channel);
        //let mut q = PwmDutyCycle::default();
        //q.pwm = 4096;
        //rprintln!("{:#?}",self.write_data(header, q).await.unwrap());

        //let header = MsgIdWithChannel::new(MaxMinCurrentRead::MSG_ID, channel);
        //let mut q = MaxMinCurrentRead::default();
        //let response = self.read_data(header, q).await.unwrap();
        //rprintln!("{:#?}", response);


        //self.ps.set_high().unwrap();
        //cortex_m::asm::delay(50_000);
        //self.ps.set_low().unwrap();

        let header = MsgIdWithChannel::new(ControlVariableSet::MSG_ID, channel);
        let mut q = ControlVariableSet::default();
        q.kp = 2428;
        q.ki = 628;
        rprintln!("{:#?}", self.write_data(header, q).await.unwrap());

        let spi_val: f32 = ma as f32 / R_SENSE_EQ_VAL;
        let setpoint_val = (spi_val * 2048.0) as u16;
        rprintln!("Setpoint val calc is {}", setpoint_val);

        let mut q = CurrentDitherAmplitudeSet::default();
        q.current_setpoint = setpoint_val;
        q.en = true;
        q.dither_step_size = 0;
        let header = MsgIdWithChannel::new(CurrentDitherAmplitudeSet::MSG_ID, channel);
        rprintln!("{:#?}", self.write_data(header, q).await.unwrap());
        Some(())

    }

    #[inline(always)]
    pub async fn read_data<MSG: TleMessage>(&mut self, header: MSG::HEADER, data: MSG) -> Option<MSG> {
        self.send_and_receive(header, data, false).await
    }

    #[inline(always)]
    pub async fn write_data<MSG: TleMessage>(&mut self, header: MSG::HEADER, data: MSG) -> Option<MSG> {
        self.send_and_receive(header, data, true).await
    }

    pub async fn send_and_receive<MSG: TleMessage>(&mut self, header: MSG::HEADER, data: MSG, write: bool) -> Option<MSG> {
        let mut full_msg = TleMessagePayload::default();
        full_msg.msg_id = header.pack().unwrap().as_bytes_slice()[0];
        full_msg.r_w = write;
        data.pack_to_slice(&mut full_msg.payload).unwrap();
        let mut buf = [0u8; 4];
        full_msg.pack_to_slice(&mut buf).unwrap();
        self.cs.set_low().unwrap();

        let result = if let Some(read) = self.send_command(&mut buf).await {
            let response = TleMessagePayload::unpack_from_slice(&read).unwrap();
            if response.msg_id == full_msg.msg_id {
                let data = MSG::unpack_from_slice(&response.payload).unwrap();
                Some(data)
            } else {
                None
            }
        } else {
            None
        };
        result
    }

    /// Calcualtes the KP and KI constant values for each solenoid channel.
    /// 
    /// ## Parameters
    /// * c - Constant value for control algorithm (Default = 0.707)
    /// * v_batt - Battery voltage, in Vols
    /// * l_c - Inductance of the solenoid
    /// * r_c - Resistance of the solenoid, in ohms
    /// * r_sense - Resistance of the shunt resister, in ohms
    /// * f_clk - Clk driver frequency
    /// * f_pwm - PWM frequency
    /// 
    /// # Returns
    /// Tuple of (KP, KI)
    fn calc_kp_ki(v_batt: f32, r_c: f32, l_c: f32, r_sense: f32, f_clk: u32, f_pwm: u32, c: f32) -> Result<(u32, u32), KpKiErr> {
        // ωn
        let wn: f32 = f_pwm as f32 / (c*5.0);
        // KP'
        let kp_ = ((2.0*c*wn) - (r_c/l_c)) * (l_c / v_batt);
        if kp_ < 0.0 {
            return Err(KpKiErr::KpiTooSmall(kp_))
        }
        // KI'
        let ki_ = (l_c / v_batt) * (wn*wn);

        let kp = (kp_ * ((0.04 * f_clk as f32)/(r_sense * f_pwm as f32))) as u32;
        let ki = (ki_ * ((0.04 * f_clk as f32)/(r_sense * (f_pwm.pow(2)) as f32))) as u32;
        if kp > 4095 {
            return Err(KpKiErr::KpOverflow(kp));
        } else if ki > 4095 {
            return Err(KpKiErr::KiOverflow(ki));
        } else {
            Ok((kp, ki))
        }
    }
}

pub enum KpKiErr {
    KpiTooSmall(f32),
    KpOverflow(u32),
    KiOverflow(u32)
}

