//! Content for embassy and CPU usage time keeping

mod panic;
mod defmt_timestamp;

use core::sync::atomic::{AtomicU16, Ordering};

use atsamd_hal::{interrupt::{self, InterruptExt, Priority}, pac::{Peripherals, Wdt}, watchdog};
use cortex_m_rt::exception;
use embassy_executor::{raw::Executor, InterruptExecutor, SendSpawner, Spawner};
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, Ticker, Timer};
use portable_atomic::AtomicU64;
use static_cell::StaticCell;
use systick_timer::SystickDriver;
use embedded_hal::watchdog::*;

use crate::usb::uart::uart_write;

// Systick handler
embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver<16> = SystickDriver::new(100_000_000, 0xA0_FF));

#[exception]
fn SysTick() {
    DRIVER.systick_interrupt();
}

// For CPU usage
pub static SLEEP_TICKS: AtomicU64 = AtomicU64::new(0);

// Executor for thread (Low priority tasks)
pub static EXECUTOR_THREAD_LOW: StaticCell<Executor> = StaticCell::new();
pub static SPAWNER_THREAD: StaticCell<Spawner> = StaticCell::new();
// Executor for interrupts (Medium priority tasks)
pub static EXECUTOR_INTERRUPT_MED: InterruptExecutor = InterruptExecutor::new();
// Executor for interrupts (Medium priority tasks)
pub static EXECUTOR_INTERRUPT_HIGH: InterruptExecutor = InterruptExecutor::new();

static CPU_USAGE: AtomicU16 = AtomicU16::new(0);

/// Called when OS clocks are configured,
/// to launch the Embassy system running on SYST
/// 
/// Also enables the watchdog of the CPU. Each task must kick the watchdog
/// on every loop iteration to ensure the CPU does not reset
pub fn os_start() {
    defmt::info!("OS DRIVER START!");
    let mut core = unsafe { cortex_m::Peripherals::steal() };
    DRIVER.start(&mut core.SYST);
    // Start the interrupt executors
}

pub static SPAWNER_INTERRUPT_HIGH: OnceLock<SendSpawner> = OnceLock::new();

/// Launches the OS's internal tasks
pub fn os_launch_internal_tasks(spawner: &Spawner, wdt: Wdt) {
    let mut watchdog = watchdog::Watchdog::new(wdt);
    // Runs at 1024Hz, so 2048 cycles = 2 seconds
    watchdog.start(watchdog::WatchdogTimeout::Cycles2K as u8);

    //let isr_high = SPAWNER_INTERRUPT_HIGH.try_get().unwrap();
    spawner.must_spawn(task_cpu_usage_monitor());

    spawner.must_spawn(task_wdt_feeder(watchdog));
    spawner.must_spawn(task_cpu_print_usage()); // Test (Remove later)
}

#[embassy_executor::task]
async fn task_wdt_feeder(mut watchdog: watchdog::Watchdog) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        watchdog.feed();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn task_cpu_usage_monitor() {
    defmt::info!("OS CPUMON START!");
    let mut previous_tick = 0u64;
    let mut previous_sleep_tick = 0u64;
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        let current_tick = Instant::now().as_ticks();
        let current_sleep_tick = SLEEP_TICKS.load(Ordering::Relaxed);
        let sleep_tick_difference = (current_sleep_tick - previous_sleep_tick) as f32;
        let tick_difference = (current_tick - previous_tick) as f32;
        let usage = 1f32 - sleep_tick_difference / tick_difference;
        previous_tick = current_tick;
        previous_sleep_tick = current_sleep_tick;
        CPU_USAGE.store((usage * 10000.0) as u16, Ordering::Relaxed);
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn task_cpu_print_usage() {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        defmt::info!("Usage: {}% - {}", CPU_USAGE.load(Ordering::Relaxed) as f32 / 100.0, Instant::now().as_ticks());
        uart_write();
        Timer::after_micros(100).await;
        ticker.next().await;
    }
    
}