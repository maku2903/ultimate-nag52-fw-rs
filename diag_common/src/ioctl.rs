/// Input/output control by local ID
/// request commands
pub enum IoctlRequest {
    /// EGS Compatibility - Variant coding
    EgsVariantCoding = 0x01,
    /// EGS Compatibility - ECU Mode
    EgsMode = 0x10,
    /// EGS Compatibility - Shift solenoids
    /// ONLY available for Adjustment Ty 01/07
    ShiftSolenoids = 0x20,
    /// EGS Compatibility - TCC Solenoid
    TccSolenoid = 0x23,
    /// EGS Compatibility - MPC Solenoid
    MpcSolenoid = 0x24,
    /// EGS Compatibility - SPC Solenoid
    SpcSolenoid = 0x25,
    /// UN52 - Bootloader flash check toggle
    BootloaderFlashCheck = 0x50,
}

/// Mode which to operate local ID
pub enum IoctlAdjustmentTy {
    ReturnControlToEcu = 0x00,
    ReportCurrentState = 0x01,
    ShortTermAdjust = 0x07,
    LongTermAdjust = 0x08
}