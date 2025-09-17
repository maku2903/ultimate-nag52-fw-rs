#![allow(dead_code)]
use core::ops::Range;

use atsamd_hal::nvm::{self, Nvm};
use konst::{parsing::Parser, result};

const KB: u32 = 1024;
const SECTOR_SIZE: u32 = 8192;

const PRELOADER_ADDR_RANGE: Range<u32> = 0..(8 * KB);
const BOOTLOADER_ADDR_RANGE: Range<u32> = (8 * KB)..(128 * KB);
const BOOTLOADER_INFO_ADDR_RANGE: Range<u32> = (120 * KB)..(128 * KB);
const BOOTLOADER_SCRATCH_ADDR_RANGE: Range<u32> = (128 * KB)..(248 * KB);
const APP_ADDR_RANGE: Range<u32> = (128 * KB)..(1024 * KB);

pub enum MemoryRegion {
    Preloader,
    Bootloader,
    BootloaderInfo,
    BootloaderScratch,
    Application,
}

impl MemoryRegion {
    pub const fn range_exclusive(&self) -> Range<u32> {
        match self {
            MemoryRegion::Preloader => PRELOADER_ADDR_RANGE,
            MemoryRegion::Bootloader => BOOTLOADER_ADDR_RANGE,
            MemoryRegion::BootloaderInfo => BOOTLOADER_INFO_ADDR_RANGE,
            MemoryRegion::BootloaderScratch => BOOTLOADER_SCRATCH_ADDR_RANGE,
            MemoryRegion::Application => APP_ADDR_RANGE,
        }
    }

    pub const fn blocks_8k(&self) -> u32 {
        let range = self.range_exclusive();
        (range.end - range.start) / SECTOR_SIZE
    }

    pub const fn start_addr(&self) -> u32 {
        self.range_exclusive().start
    }
}

// Ensure everything is aligned to per-page
static_assertions::const_assert!(PRELOADER_ADDR_RANGE.start % SECTOR_SIZE == 0);
static_assertions::const_assert!(BOOTLOADER_ADDR_RANGE.start % SECTOR_SIZE == 0);
static_assertions::const_assert!(BOOTLOADER_INFO_ADDR_RANGE.start % SECTOR_SIZE == 0);
static_assertions::const_assert!(APP_ADDR_RANGE.start % SECTOR_SIZE == 0);

#[derive(defmt::Format, Clone, Copy, PartialEq, Eq)]
#[repr(C, packed(4))]
pub struct BootloaderInfo {
    pub name: [u8; 8],
    pub version_major: u8,
    pub version_minor: u8,
    pub version_patch: u8,

    pub rustc_version_major: u8,
    pub rustc_version_minor: u8,
    pub rustc_version_patch: u8,

    pub compile_year: u8,
    pub compile_month: u8,
    pub compile_week: u8,
    pub compile_day: u8,
    pub app_flashing_not_done: u8,
    pub application_crc: u32,
    /// 0 = Pending copy (Check CRC)
    /// 1 = No pending copy (No update)
    pub bootloader_flashing_pending: u8,
    pub bootloader_flashing_crc: u32,
}

static_assertions::const_assert!(size_of::<BootloaderInfo>() < SECTOR_SIZE as usize);

const fn parse_u8(s: &str) -> u8 {
    let mut p = Parser::new(s);
    result::unwrap!(p.parse_u8())
}

#[cfg(not(feature = "lib"))]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".bl_info")]
static BOOTLOADER_INFO: BootloaderInfo = BootloaderInfo {
    name: *b"UN52 xGS",
    version_major: parse_u8(env!("CARGO_PKG_VERSION_MAJOR")),
    version_minor: parse_u8(env!("CARGO_PKG_VERSION_MINOR")),
    version_patch: parse_u8(env!("CARGO_PKG_VERSION_PATCH")),
    compile_year: parse_u8(env!("BUILD_YEAR")),
    compile_month: parse_u8(env!("BUILD_MONTH")),
    compile_week: parse_u8(env!("BUILD_WEEK")),
    compile_day: parse_u8(env!("BUILD_DAY")),
    rustc_version_major: parse_u8(env!("RUSTC_VER_MAJOR")),
    rustc_version_minor: parse_u8(env!("RUSTC_VER_MINOR")),
    rustc_version_patch: parse_u8(env!("RUSTC_VER_PATCH")),
    app_flashing_not_done: 1,
    application_crc: 0xFFFF_FFFF,
    bootloader_flashing_pending: 1,
    bootloader_flashing_crc: 0xFFFF_FFFF,
};

#[inline(always)]
pub fn region_crc(addr_range: Range<u32>) -> u32 {
    let start = core::ptr::slice_from_raw_parts(addr_range.start as *mut u8, addr_range.len());
    unsafe { embedded_crc32c::crc32c(start.as_ref().unwrap()) }
}

#[inline(always)]
pub fn get_bootloader_info() -> &'static BootloaderInfo {
    unsafe {
        let ptr = BOOTLOADER_INFO_ADDR_RANGE.start as *const BootloaderInfo;
        ptr.as_ref().unwrap()
    }
}

/// Modify the bootloader info page in NVM memory
///
/// # Safety
/// This function erases the info page and then writes it back, therefore
/// if power loss occurs during erase, then the bootloader info section
/// will be corrupt
pub unsafe fn mutate_bootloader_info<F: FnOnce(&mut BootloaderInfo)>(
    nvm: &mut Nvm,
    f: F,
) -> nvm::Result<()> {
    let bl_state = *get_bootloader_info();
    let mut modified = bl_state;
    f(&mut modified);
    if modified != bl_state {
        let start_addr = BOOTLOADER_INFO_ADDR_RANGE.start;
        let size = core::mem::size_of::<BootloaderInfo>() / 4;
        let mut read: [u32; SECTOR_SIZE as usize / 4] = [0; _];
        // Copy the page to a temp buffer
        core::ptr::copy_nonoverlapping(
            start_addr as *mut u32,
            read.as_mut_ptr(),
            SECTOR_SIZE as usize / size_of::<u32>(),
        );
        let to_copy =
            core::ptr::slice_from_raw_parts(&modified as *const BootloaderInfo as *const u32, size);
        read[..size].copy_from_slice(to_copy.as_ref().unwrap());

        // Erase the page
        nvm.erase_flash(start_addr as *mut u32, 1)?;
        nvm.write_flash_from_slice(
            start_addr as *mut u32,
            &read,
            atsamd_hal::nvm::WriteGranularity::Page,
        )?;
    }
    Ok(())
}