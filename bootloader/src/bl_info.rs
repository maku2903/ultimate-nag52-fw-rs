use atsamd_hal::nvm::{self, Nvm};
use konst::{parsing::Parser, result};

const BOOTLOADER_INFO_ADDR: u32 = 0x10000 - 512; // See memory.x

pub (crate) const APP_ADDR_START: u32 = 0x00010000; // 64KB
pub (crate) const APP_ADDR_END: u32 = 0x00080000; // 512KB

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
    pub flashing_not_done: u8,
    pub application_crc: u32,
}

const fn parse_u8(s: &str) -> u8 {
    let mut p = Parser::new(s);
    result::unwrap!(p.parse_u8())
}

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
    flashing_not_done: 1,
    application_crc: 0xFFFF_FFFF,
};

pub (crate) fn app_crc() -> u32 {
    let start = core::ptr::slice_from_raw_parts(
        APP_ADDR_START as *mut u8,
        (APP_ADDR_END - APP_ADDR_START) as usize,
    );
    unsafe { embedded_crc32c::crc32c(start.as_ref().unwrap()) }
}

pub fn get_bootloader_info() -> &'static BootloaderInfo {
    unsafe {
        let ptr = BOOTLOADER_INFO_ADDR as *const BootloaderInfo;
        ptr.as_ref().unwrap()
    }
}

pub (crate) fn mutate_bootloader_info<F: FnOnce(&mut BootloaderInfo)>(
    nvm: &mut Nvm,
    f: F,
) -> nvm::Result<()> {
    let bl_state = *get_bootloader_info();
    let mut modified = bl_state;
    f(&mut modified);
    if modified != bl_state {
        let erase_start = (BOOTLOADER_INFO_ADDR / 8192) * 8192;
        let into = (BOOTLOADER_INFO_ADDR % 8192) as usize / 4;
        let size = core::mem::size_of::<BootloaderInfo>() / 4;
        let mut read: [u32; 8192 / 4] = [0; _];
        unsafe {
            // Copy the page to a temp buffer
            core::ptr::copy_nonoverlapping(
                erase_start as *mut u32,
                read.as_mut_ptr(),
                8192 / size_of::<u32>(),
            );
            let to_copy = core::ptr::slice_from_raw_parts(
                &modified as *const BootloaderInfo as *const u32,
                size,
            );
            read[into..into + size].copy_from_slice(to_copy.as_ref().unwrap());

            // Erase the page
            nvm.erase_flash(erase_start as *mut u32, 1)?;
            nvm.write_flash_from_slice(
                erase_start as *mut u32,
                &read,
                atsamd_hal::nvm::WriteGranularity::Page,
            )?;
        }
    }
    Ok(())
}
