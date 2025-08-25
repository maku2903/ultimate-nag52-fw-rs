pub struct AppInfo {
    // Data used by bootloader
    flashing_done: u8,
    app_crc: u32,
    // General info
    version_major: u8,
    version_minor: u8,
    version_patch: u8,

    compile_year: u8,
    compile_month: u8,
    compile_day: u8,
}

#[unsafe(link_section = ".app_info")]
static APP_INFO: AppInfo = AppInfo {
    // Data used by bootloader
    flashing_done: 0xFF,
    app_crc: 0xFFFF_FFFF,
    // General info
    version_major: 1,
    version_minor: 2,
    version_patch: 3,
    compile_year: 4,
    compile_month: 5,
    compile_day: 6,
};
