#![no_std]
pub const MAX_RESET_COUNT: u8 = 5;

pub mod bl_info;

pub struct BootloaderState {
    // Bootloader -> Application
    /// If this counter goes above [`MAX_RESET_COUNT`]
    /// it will trigger the bootloader to not start the app
    /// essentially an emergency recovery mode. User can trigger
    /// this by quick pressing the reset button 5 times rapidly
    reset_counter: u8,
    // Application -> Bootloader
    /// Diagnostic session
    /// has requested the bootloader stays active
    /// for an application update
    diag_request_bootloader: bool,
}

const BOOTLOADER_COMM_ADDR: *mut BootloaderState = 0x20010000 as *mut BootloaderState;

pub fn get_bootloader_state() -> BootloaderState {
    unsafe { core::ptr::read(BOOTLOADER_COMM_ADDR) }
}

pub fn set_bootloader_state(s: BootloaderState) {
    unsafe {
        core::ptr::write(BOOTLOADER_COMM_ADDR, s);
    }
}

//const BOOTLOADER_INFO_ADDR: *const BootloaderInfo = 0x20010000 as *mut BootloaderState;

//pub fn get_bootloader_info() -> BootloaderState {
//    unsafe { core::ptr::read(BOOTLOADER_COMM_ADDR) }
//}
