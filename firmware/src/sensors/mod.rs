use atsamd_hal::adc::CpuVoltageSource;
use atsamd_hal::{
    adc::{
        Accumulation, Adc0, Adc1, AdcBuilder, AdcResolution, FutureAdc, InterruptHandler,
        Prescaler, Reference,
    },
    clock::{
        self,
        v2::{
            apb::ApbClk,
            gclk::GclkId,
            pclk::{Pclk, PclkId},
        },
    },
    pac::{self, Supc},
};
use bsp::{AccelMSense, AccelPSense, SolPwrSense, Tft, VBattSense, VSensorSense, VsolSense};
use defmt::info;

use crate::maths;

atsamd_hal::bind_multiple_interrupts!(pub struct Adc0Irqs {
    ADC0: [ADC0_RESRDY, ADC0_OTHER] => InterruptHandler<Adc0>;
});

atsamd_hal::bind_multiple_interrupts!(pub struct Adc1Irqs {
    ADC1: [ADC1_RESRDY, ADC1_OTHER] => InterruptHandler<Adc1>;
});

// All values here are from 0-4095 (12bit ADC reading)
pub struct AnalogSensors {
    /// Feedback of the 5V sensor supply
    vsense_feedback: u16,
    /// Feedback of the KL15 terminal
    vbatt_feedback: u16,
    /// Accelerator + Input voltage
    accel_p_sense: u16,
    /// Accelerator - Input voltage
    accel_m_sense: u16,
    /// TFT Sensor on the valve body
    tft: u16,
    /// Linear feedback from high side switch for solenoids
    sol_pwr_sense: u16,
    /// Voltage feedback of the solenoid supply (KL87)
    vsol_sense: u16,
}

pub struct AdcPins {
    pub vbatt_sense: VBattSense,
    pub vsensor_sense: VSensorSense,
    pub accel_plus: AccelPSense,
    pub accel_minus: AccelMSense,
    pub tft: Tft,
    pub sol_pwr_sense: SolPwrSense,
    pub vsol_sense: VsolSense,
}

pub struct AdcData {
    adc0: FutureAdc<Adc0, Adc0Irqs>,
    adc1: FutureAdc<Adc1, Adc1Irqs>,
    /// Required for reading the board temperature
    supc: Supc,
    /// Pins required
    pins: AdcPins,
}

impl AdcData {
    pub fn new<P: GclkId>(
        adc0: pac::Adc0,
        adc1: pac::Adc1,
        supc: Supc,
        pins: AdcPins,
        apb_adc0: ApbClk<clock::v2::pclk::ids::Adc0>,
        apb_adc1: ApbClk<clock::v2::pclk::ids::Adc1>,
        pclk_adc0: Pclk<clock::v2::pclk::ids::Adc0, P>,
        pclk_adc1: Pclk<clock::v2::pclk::ids::Adc1, P>,
    ) -> Self {
        let adc_adc0 = AdcBuilder::new(Accumulation::Single(AdcResolution::_12))
            .with_vref(Reference::Intvcc1)
            .with_clock_divider(Prescaler::Div8)
            .with_clock_cycles_per_sample(8)
            .enable(adc0, apb_adc0, &pclk_adc0)
            .unwrap()
            .into_future(Adc0Irqs);
        let adc_adc1 = AdcBuilder::new(Accumulation::Single(AdcResolution::_12))
            .with_vref(Reference::Intvcc1)
            .with_clock_divider(Prescaler::Div8)
            .with_clock_cycles_per_sample(8)
            .enable(adc1, apb_adc1, &pclk_adc1)
            .unwrap()
            .into_future(Adc1Irqs);
        Self {
            adc0: adc_adc0,
            adc1: adc_adc1,
            supc,
            pins,
        }
    }

    /// Assume ADC reading is 0-4095 (=0-3.3V)
    /// R1 and R2 are in Ohms
    pub fn adc_reading_to_source(adc: u16, r1: u32, r2: u32) -> u16 {
        let div = (r2 * 1000) / (r1 + r2);
        let v_out = 3300 * (((adc as u32) * 1000) / 4096);
        (v_out / div) as u16
    }

    pub async fn update(&mut self) {
        let mcu_core_supply = self.adc0.read_cpu_voltage(CpuVoltageSource::Core).await;
        // What 4095 means from ADC
        let mcu_io_supply = self.adc0.read_cpu_voltage(CpuVoltageSource::Io).await;

        // R1 is known (2.0KOhm)
        // R2 is the TFT Sensor
        let tft_res = self.adc0.read(&mut self.pins.tft).await;
        let tft_voltage = (tft_res as u32 * mcu_io_supply as u32) / 4095;
        // Reverse so we figure out R2 (TFT Resistance)
        // Vout = Vin * (R2/(R1+R2))
        // => R2 = (Vout * R1) / (Vin - Vout)
        //
        let tft_resistance = if tft_res == 4095 {
            0
        } else {
            (tft_voltage * 2000) / (mcu_io_supply as u32 - tft_voltage)
        };
        let tft_temp = maths::interp(tft_resistance as i32, TFT_LOOKUP);
        // Vout = Vin * (R2 / (R1+R2))

        // Voltage supplies (0-12V) (R1 = 10KOhm, R2 = 2.2KOhm)
        let board_supply = Self::adc_reading_to_source(
            self.adc1.read(&mut self.pins.vbatt_sense).await,
            10_000,
            2_200,
        );
        let solenoid_supply = Self::adc_reading_to_source(
            self.adc1.read(&mut self.pins.vsol_sense).await,
            10_000,
            2_200,
        );
        // Voltage supplies (0-5V)
        let sensor_supply = Self::adc_reading_to_source(
            self.adc1.read(&mut self.pins.vsensor_sense).await,
            10_000,
            15_000,
        );

        // Current draw
        let solenoid_current = self.adc0.read(&mut self.pins.sol_pwr_sense).await;
        // CPU Temperature
        let cpu_temp = self.adc0.read_cpu_temperature(&mut self.supc).await;
        //info!(
        //    "TFT: ({} mV {} Ohm {} C), CPU: {} C. [V_KL30: {} V V_KL87: {} V] MCU Core: {}mV - MCU IO: {}mV",
        //    tft_voltage,
        //    tft_resistance,
        //    tft_temp,
        //    cpu_temp,
        //    board_supply  as f32 / 1000.0,
        //    solenoid_supply as f32 / 1000.0,
        //    mcu_core_supply,
        //    mcu_io_supply
        //);
    }
}

// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
// KTY83/110
// Resistance (Ohm), Temperature
const TFT_LOOKUP: &[(i32, i32)] = &[
    (500, -55),
    (525, -50),
    (577, -40),
    (632, -30),
    (691, -20),
    (754, -10),
    (820, 0),
    (889, 10),
    (962, 20),
    (1000, 25),
    (1039, 30),
    (1118, 40),
    (1202, 50),
    (1288, 60),
    (1379, 70),
    (1472, 80),
    (1569, 90),
    (1670, 100),
    (1774, 110),
    (1882, 120),
    (1937, 125),
    (1993, 130),
    (2107, 140),
    (2225, 150),
    (2346, 160),
    (2471, 170),
    (2535, 175),
];
