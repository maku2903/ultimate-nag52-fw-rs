#![no_std]
#![no_main]

use core::panic::PanicInfo;

use atsamd_hal::{
    nvm::Nvm,
    pac::{Peripherals, SCB},
};
use bootloader::bl_info::{MemoryRegion, mutate_bootloader_info, region_crc};
use cortex_m_rt::entry;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    SCB::sys_reset();
}

#[entry]
fn main() -> ! {
    let bl_info = bootloader::bl_info::get_bootloader_info();
    // 0 = Pending, 0xFF = not pending
    if bl_info.bootloader_flashing_pending == 0 {
        let bsp_peripherals = Peripherals::take().unwrap();
        let mut nvm = Nvm::new(bsp_peripherals.nvmctrl);
        unsafe {
            // We have to copy the bootloader portions
            let scratch_crc = region_crc(MemoryRegion::BootloaderScratch.range_exclusive());
            if scratch_crc == bl_info.bootloader_flashing_crc {
                // Can copy (Sig. valid)
                let bootloader_region = MemoryRegion::Bootloader;
                let copy_region = MemoryRegion::BootloaderScratch;
                // Erase bootloader
                let _ = nvm.erase_flash(
                    bootloader_region.start_addr() as *mut u32,
                    bootloader_region.blocks_8k(),
                );
                //// Copy over the scratch area to the bootloader
                let _ = nvm.write_flash(
                    bootloader_region.start_addr() as *mut u32,
                    copy_region.start_addr() as *const u32,
                    bootloader_region.range_exclusive().len() as u32 / 4,
                    atsamd_hal::nvm::WriteGranularity::Page,
                );
            }
            // Set the flags so that we don't do this on next boot
            let _ = mutate_bootloader_info(&mut nvm, |info| {
                info.bootloader_flashing_pending = 0xFF;
                info.bootloader_flashing_crc = 0xFFFF_FFFF;
            });
        }
    }
    // Jump to bootloader
    unsafe {
        let core_peripehrals = cortex_m::Peripherals::steal();
        let bootloader_addr = MemoryRegion::Bootloader.start_addr();
        core_peripehrals.SCB.vtor.write(bootloader_addr);
        cortex_m::asm::bootload(bootloader_addr as *const u32)
    }
}
