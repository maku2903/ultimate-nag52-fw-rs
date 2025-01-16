use atsamd_hal::{dmac::{self, DmaController, PriorityLevel}, nb, pac::{Dmac, Pm}, sercom::uart::{self, Uart}};
pub use atsamd_hal::embedded_io::Write;

pub struct Logger{
    pub uart: Uart<uart::Config<UartPads>, uart::Duplex, dmac::Channel<dmac::Ch1, dmac::Ready>, dmac::Channel<dmac::Ch0, dmac::Ready>>,
    pub max_level: LogLevel,
}

pub static mut GLOBAL_LOGGER: Option<Logger> = None;

#[derive(PartialOrd, PartialEq)]
pub enum LogLevel {
    Debug = 3,
    Info = 2,
    Warn = 1,
    Error = 0,
}

impl LogLevel {
    pub fn char(&self) -> &'static str {
        match self {
            LogLevel::Debug => "D",
            LogLevel::Info => "I",
            LogLevel::Warn => "W",
            LogLevel::Error => "E",
        }
    }
}

#[macro_export]
macro_rules! log_error {
    ($($args:tt)+) => {
        $crate::log_uart!(LogLevel::Error, $($args)+)
    }
}

#[macro_export]
macro_rules! log_warn {
    ($($args:tt)+) => {
        $crate::log_uart!(LogLevel::Warn, $($args)+)
    }
}

#[macro_export]
macro_rules! log_info {
    ($($args:tt)+) => {
        $crate::log_uart!(LogLevel::Info, $($args)+)
    }
}

#[macro_export]
macro_rules! log_debug {
    ($($args:tt)+) => {
        $crate::log_uart!(LogLevel::Debug, $($args)+)
    }
}

#[macro_export]
macro_rules! log_uart {
    ($lvl:expr, $($args:tt)+) => {
        if let Some(logger) = unsafe { GLOBAL_LOGGER.as_mut() } {
            let lvl = $lvl;
            if lvl <= logger.max_level {
                write!(&mut logger.uart, "{} ({}) {}:{}: ", lvl.char(), Mono::now().duration_since_epoch().to_millis(), module_path!(), line!());
                write!(&mut logger.uart, $($args)*);
                write!(&mut logger.uart, "\n");
            }
        }
    }
}

use bsp::UartPads;

impl Logger {
    //pub fn log(msg: &str) {
    //    if let Some(logger) = unsafe { GLOBAL_LOGGER.as_mut() } {
    //        write!(&mut logger.uart, "").unwrap();
    //    }
    //}

    pub fn init(
        uart: bsp::Uart,
        dmac: Dmac,
        pm: &mut Pm,
        max_level: LogLevel
    ) {

        let mut dma = DmaController::init(dmac, pm);
        let channels = dma.split();
        let chan0 = channels.0.init(PriorityLevel::Lvl0);
        let chan1 = channels.1.init(PriorityLevel::Lvl0);

        let uart = uart.with_tx_channel(chan0)
            .with_rx_channel(chan1);
        let logger= Logger {
            uart,
            max_level
        };
        unsafe { GLOBAL_LOGGER = Some(logger) };
    }
}
