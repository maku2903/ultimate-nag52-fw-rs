#![no_std]

use atsamd_hal::{can::Dependencies, clock::v2::{gclk::Gclk0Id, pclk::Pclk}, ehal::digital::OutputPin, pac, sercom::{spi, uart::{self, BaudMode, Oversampling}, IoSet1, IoSet3, Sercom0, Sercom2, Sercom4, Sercom5}, time::Hertz};
pub use atsamd_hal as hal;

pub mod can_deps;

use core::convert::Into;

hal::bsp_peripherals!(
    Sercom5 { UartSercom }
    Sercom2 { TleSpiSercom }
    Sercom4 { KlineSercom }
);

hal::bsp_pins!(
    // -- PORT A -- //
    PA00 {
        name: start_ctrl
    },
    PA01 {
        name: kd_sense
    },
    PA02 {
        name: tft
        aliases: {
            AlternateB: Adc0Tft
        }
    },
    PA03 {
        name: vbatt
        aliases: {
            AlternateB: Adc0Vbatt
        }
    },
    PA04 {
        name: tle_reset
        aliases: {
            PushPullOutput: TleReset
        }
    },
    PA05 {
        name: tle_phase_sync
    },
    PA06 {
        name: sol_pwr_en
        aliases: {
            PushPullOutput: SolPwrEn
        }
    },
    PA08 {
        name: buzzer,
        aliases: {
            AlternateF: BuzzerPwm
        }
    },
    PA09 {
        name: tcc_pwm,
        aliases: {
            AlternateF: TccPwm
        }
    },
    PA10 {
        name: tcc_cutoff,
        aliases: {
            AlternateF: TccCutoff
        }
    },

    PA12 {
        name: tle_cs,
        aliases: {
            PushPullOutput: TleCs
        }
    },

    PA13 {
        name: tle_sclk,
        aliases: {
            AlternateC: TleSclk
        }
    },
    PA14 {
        name: tle_so,
        aliases: {
            AlternateC: TleMiso
        }
    },
    PA15 {
        name: tle_si,
        aliases: {
            AlternateC: TleMosi
        }
    },
    PA16 {
        name: tle_clk,
        aliases: {
            AlternateF: TleClk
        }
    },

    PA17 {
        name: trrs_a,
    },
    PA18 {
        name: trrs_b,
    },
    PA19 {
        name: trrs_c,
    },
    PA20 {
        name: trrs_d,
    },
    PA21 {
        name: prg_btn_sense,
    },

    PA22 {
        name: can_tx,
        aliases: {
            AlternateI: Can0Tx
        }
    },
    PA23 {
        name: can_rx,
        aliases: {
            AlternateI: Can0Rx
        }
    },

    // -- PORT B -- //
    PB00 {
        name: rpm_3,
    },
    PB01 {
        name: rpm_2,
    },
    PB02 {
        name: rpm_1,
    },
    PB03 {
        name: brake_in,
    },

    PB04 {
        name: accel_p,
        aliases: {
            AlternateB: Adc1AccelP
        }
    },
    PB05 {
        name: accel_m,
        aliases: {
            AlternateB: Adc1AccelM
        }
    },
    PB06 {
        name: v_sensors,
        aliases: {
            AlternateB: Adc1SensorVoltMon
        }
    },

    PB07 {
        name: tcc_feedback,
    },

    PB08 {
        name: sol_pwr_sense,
        aliases: {
            AlternateB: Adc0SolPwrSense
        }
    },

    PB10 {
        name: rpm_n2,
    },
    PB11 {
        name: rpm_n3,
    },
    PB12 {
        name: tle_fault,
    },

    PB13 {
        name: lin_tx,
        aliases: {
            AlternateC: LinTx
        }
    },
    PB14 {
        name: lin_rx,
        aliases: {
            AlternateC: LinRx
        }
    },
    PB15 {
        name: lin_slp,
    }

    PB16 {
        name: uart_tx,
        aliases: {
            AlternateC: UartTx
        }
    },
    PB17 {
        name: uart_rx,
        aliases: {
            AlternateC: UartRx
        }
    },
);

pub type TleSpiPads = spi::Pads<TleSpiSercom, IoSet1, TleMiso, TleMosi, TleSclk>;
pub type TleSpi = spi::Spi<spi::Config<TleSpiPads>, spi::Duplex>;

pub fn tle_spi(
    pclk_sercom0: Pclk<Sercom2, Gclk0Id>,
    baud: Hertz,
    sercom: TleSpiSercom,
    mclk: &mut pac::Mclk,
    sclk: impl Into<TleSclk>,
    mosi: impl Into<TleMosi>,
    miso: impl Into<TleMiso>,
) -> TleSpi {
    let mut miso: TleMiso = miso.into();
    miso.set_drive_strength(true);
    let mut mosi: TleMosi = mosi.into();
    mosi.set_drive_strength(true);
    let mut sclk: TleSclk = sclk.into();
    sclk.set_drive_strength(true);

    let pads = spi::Pads::default()
        .data_in(miso)
        .data_out(mosi)
        .sclk(sclk);
    spi::Config::new(mclk, sercom, pads, pclk_sercom0.freq())
        .baud(baud)
        .spi_mode(spi::MODE_0)
        .bit_order(spi::BitOrder::MsbFirst)
        .enable()
}

pub type UartPads = uart::Pads<UartSercom, IoSet1, UartRx, UartTx>;
pub type Uart = uart::Uart<uart::Config<UartPads>, uart::Duplex>;

pub fn serial_uart(
    pclk: Pclk<Sercom5, Gclk0Id>,
    baud: impl Into<Hertz>,
    sercom: Sercom5,
    mclk: &mut pac::Mclk,
    uart_rx: impl Into<UartRx>,
    uart_tx: impl Into<UartTx>,
) -> Uart {
    let baud = baud.into();
    let pads = uart::Pads::default().rx(uart_rx.into()).tx(uart_tx.into());
    uart::Config::new(mclk, sercom, pads, pclk.freq())
        .baud(baud, BaudMode::Fractional(Oversampling::Bits16))
        .enable()
}

pub type KlinePads = uart::Pads<KlineSercom, IoSet1, LinRx, LinTx>;
pub type Kline = uart::Uart<uart::Config<KlinePads>, uart::Duplex>;

//pub fn lin_uart(
//    pclk: Pclk<Sercom4, Gclk0Id>,
//    baud: impl Into<Hertz>,
//    sercom: Sercom4,
//    mclk: &mut pac::Mclk,
//    lin_rx: impl Into<LinRx>,
//    lin_tx: impl Into<LinTx>,
//) -> Kline {
//    let baud = baud.into();
//    let pads = uart::Pads::default().rx(lin_rx.into()).tx(lin_tx.into());
//    uart::Config::new(mclk, sercom, pads, pclk.freq())
//        .baud(baud, BaudMode::Fractional(Oversampling::Bits16))
//        .enable()
//}