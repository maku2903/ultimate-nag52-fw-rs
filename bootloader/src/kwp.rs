use core::{ops::Not, ptr::NonNull, sync::atomic::Ordering};

use atsamd_hal::{
    self,
    nvm::{self, Nvm},
    pac::{dsu::did::Seriesselect, Peripherals},
    trng::Trng,
};
pub use automotive_diag::kwp2000::*;
use cortex_m::{asm::delay, peripheral::SCB};

use crate::{
    bl_info::{self, app_crc, APP_ADDR_END, APP_ADDR_START},
    isotp::{BS_EGS, ST_MIN_EGS},
};

#[derive(Copy, Clone)]
pub enum PendingOperation {
    None,
    Reset,
    FlashErase {
        start: u32,
        total_sectors: u32,
        current: u32,
    },
    Flashing {
        blk_id: u8,
        current_addr: u32,
    },
}

#[derive(Copy, Clone)]
pub enum CompletedOperation {
    FlashErase(Result<(), nvm::Error>),
}

pub const P2_MAX_MS: u64 = 2500;

const DEFAULT_SEC_MODE: SecurityLevel = SecurityLevel::FullUnlocked;

#[repr(u8)]
#[derive(defmt::Format, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecurityLevel {
    Default = 1,
    Write = 3,
    Read = 5,
    FullUnlocked = 0xFE,
}

impl SecurityLevel {
    pub fn get_seed_key(&self, _trng: &Trng) -> SecuritySeedKey {
        todo!()
    }
}

#[repr(u8)]
#[derive(defmt::Format, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecuritySeedKey {
    Default(u16, u16),
    AppWrite(u16, u16),
    AppRead(u32, u32),
    FullUnlocked(u64, u64),
}

pub struct KwpServer {
    buf: [u8; 4096],
    flash_buf: [u8; 4096],
    mode: KwpSessionType,
    pending_operation: PendingOperation,
    completed_operation: Option<CompletedOperation>,
    nvm: Nvm,
    last_cmd_time: u64,
    sec_level: SecurityLevel,
    flash_size: u32,
    _rnd: Trng,
}

type ServerResult = core::result::Result<usize, KwpError>;

impl KwpServer {
    pub fn get_flash_size(nvm: &mut Nvm) -> u32 {
        let flash_reduction_size = if let Ok(eeprom) = nvm.smart_eeprom() {
            match eeprom {
                nvm::smart_eeprom::SmartEepromMode::Locked(smart_eeprom) => {
                    smart_eeprom.iter::<u8>().count()
                }
                nvm::smart_eeprom::SmartEepromMode::Unlocked(smart_eeprom) => {
                    smart_eeprom.iter::<u8>().count()
                }
            }
        } else {
            0
        } as u32;

        let flash_size = unsafe {
            let p = Peripherals::steal().dsu;
            match p.did().read().series().variant() {
                Some(Seriesselect::Same51) => {
                    match p.did().read().devsel().bits() {
                        0x00 | 0x04 => 1024 * 1024, // 1MB,
                        0x01 | 0x02 => 512 * 1024,  // 512KB
                        0x03 | 0x06 => 256 * 1024,  // 256KB
                        _ => 0,
                    }
                }
                Some(Seriesselect::Same54) => {
                    match p.did().read().devsel().bits() {
                        0x00 | 0x02 => 1024 * 1024, // 1MB,
                        0x01 | 0x03 => 512 * 1024,  // 512KB
                        _ => 0,
                    }
                }
                _ => return 0,
            }
        };
        flash_size - flash_reduction_size
    }

    pub fn new(mut nvm: Nvm, rnd: Trng) -> Self {
        let flash_size = Self::get_flash_size(&mut nvm);
        Self {
            mode: KwpSessionType::Normal,
            pending_operation: PendingOperation::None,
            completed_operation: None,
            buf: [0; 4096],
            flash_buf: [0; 4096],
            nvm,
            last_cmd_time: 0,
            sec_level: DEFAULT_SEC_MODE,
            flash_size,
            _rnd: rnd,
        }
    }
    pub fn check_addr(&mut self, v: u32, reading: bool) -> Result<(), KwpError> {
        let (min_sec_level, can_read) = match v {
            // Code space (Bootloader)
            0x00000000..0x00010000 => (SecurityLevel::FullUnlocked, true),
            // Code space (Application)
            //0x00010000..0x00100000 => (SecurityLevel::AppRead, true),
            x if (0x00010000..self.flash_size).contains(&x) => {
                let min = if reading {
                    SecurityLevel::Read
                } else {
                    SecurityLevel::Write
                };
                (min, true)
            }
            // CMCC
            0x03000000..0x04000000 => (SecurityLevel::FullUnlocked, true),
            // QSPI disabled (So not allowed to read)
            // RAM
            0x20000000..0x20040000 => (SecurityLevel::Read, true),
            // AHB-APB Bridge A
            0x40000000..0x40004000 => (SecurityLevel::FullUnlocked, true),
            // AHB-APB Bridge B
            0x41000000..0x4100C000 => (SecurityLevel::FullUnlocked, true),
            0x4100E000..0x41010000 => (SecurityLevel::FullUnlocked, true),
            0x41012000..0x4101E000 => (SecurityLevel::FullUnlocked, true),
            0x41020000..0x41022000 => (SecurityLevel::FullUnlocked, true),
            // AHB-APB Bridge C
            0x42000000..0x42003C00 => (SecurityLevel::FullUnlocked, true),
            // AHB-APB Bridge D
            0x43000000..0x43003000 => (SecurityLevel::FullUnlocked, true),
            // Other AHB-APB systems
            0x44000000..0x48000000 => (SecurityLevel::FullUnlocked, true),
            // System
            0xE0000000..0xE000F000 => (SecurityLevel::FullUnlocked, true),
            0xE00FF000..0xE0100000 => (SecurityLevel::FullUnlocked, true),
            _ => (SecurityLevel::Default, false),
        };
        if !can_read {
            Err(KwpError::RequestOutOfRange)
        } else if self.sec_level < min_sec_level {
            Err(KwpError::SecurityAccessDenied)
        } else {
            Ok(())
        }
    }

    pub fn update(&mut self, now_ms: u64) -> Option<&[u8]> {
        if now_ms - self.last_cmd_time > P2_MAX_MS && self.mode != KwpSessionType::Normal {
            defmt::debug!("Tester timeout. Going back to default mode");
            self.mode = KwpSessionType::Normal;
            self.pending_operation = PendingOperation::None;
            self.sec_level = DEFAULT_SEC_MODE;
        }
        match &mut self.pending_operation {
            PendingOperation::Reset => {
                delay(10_000);
                SCB::sys_reset();
            }
            PendingOperation::FlashErase {
                start,
                total_sectors,
                current,
            } => {
                let addr = (*start + (8192 * *current)) as *mut u32;
                match unsafe { self.nvm.erase_flash(addr, 1) } {
                    Ok(_) => {
                        *current += 1;
                        if *total_sectors == *current {
                            self.pending_operation = PendingOperation::None;
                            self.completed_operation = Some(CompletedOperation::FlashErase(Ok(())))
                        }
                    }
                    Err(e) => {
                        self.pending_operation = PendingOperation::None;
                        self.completed_operation = Some(CompletedOperation::FlashErase(Err(e)));
                    }
                }
                None
            }
            _ => None,
        }
    }

    pub fn make_nrc(&mut self, sid: u8, nrc: impl Into<u8>) -> usize {
        self.buf[0..3].copy_from_slice(&[0x7F, sid, nrc.into()]);
        3
    }

    pub fn make_positive_reply(&mut self, sid: u8, data: &[u8]) -> usize {
        self.buf[0] = sid + 0x40;
        self.buf[1..1 + data.len()].copy_from_slice(data);
        1 + data.len()
    }

    pub fn process_cmd<'a>(&'a mut self, cmd: &[u8], now_ms: u64) -> &'a [u8] {
        self.last_cmd_time = now_ms;
        let r = if let PendingOperation::FlashErase { .. } = self.pending_operation {
            Err(KwpError::BusyRepeatRequest)
        } else {
            match KwpCommand::try_from(cmd[0]).ok() {
                Some(KwpCommand::ECUReset) => self.ecu_reset(cmd),
                Some(KwpCommand::StartDiagnosticSession) => self.start_diag_session(cmd),
                Some(KwpCommand::ReadMemoryByAddress) => self.read_mem_by_address(cmd),
                Some(KwpCommand::RequestDownload) => self.start_download(cmd),
                Some(KwpCommand::TesterPresent) => self.tester_present(cmd),
                Some(KwpCommand::RequestTransferExit) => self.transfer_exit(cmd),
                Some(KwpCommand::StartRoutineByLocalIdentifier) => self.routine_start(cmd),
                Some(KwpCommand::RequestRoutineResultsByLocalIdentifier) => {
                    self.routine_results(cmd)
                }
                Some(KwpCommand::TransferData) => self.transfer_data(cmd),
                _ => Err(KwpError::ServiceNotSupported),
            }
        };
        let reply_len = r.unwrap_or_else(|nrc| self.make_nrc(cmd[0], nrc));
        defmt::debug!(
            "KWP Reponse length {} {:02X}",
            reply_len,
            self.buf[..reply_len]
        );
        &self.buf[..reply_len]
    }

    fn start_diag_session(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() != 2 && cmd.len() != 4 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            fn set_com_param(cmd: &[u8]) {
                if cmd.len() == 4 {
                    let bs = cmd[2];
                    let stmin = cmd[3];
                    ST_MIN_EGS.store(stmin, Ordering::Relaxed);
                    BS_EGS.store(bs, Ordering::Relaxed);
                }
            }

            match KwpSessionType::try_from(cmd[1]).ok() {
                Some(KwpSessionType::Reprogramming) => {
                    self.mode = KwpSessionType::Reprogramming;
                }
                Some(KwpSessionType::Normal) => {
                    self.mode = KwpSessionType::Normal;
                }
                _ => return Err(KwpError::SubFunctionNotSupportedInvalidFormat),
            }
            set_com_param(cmd);
            Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
        }
    }

    fn ecu_reset(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        if cmd.len() != 2 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if cmd[1] == 0x01 {
            self.pending_operation = PendingOperation::Reset;
            Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
        } else {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        }
    }

    fn tester_present(&mut self, cmd: &[u8]) -> ServerResult {
        Ok(self.make_positive_reply(cmd[0], &[]))
    }

    fn read_mem_by_address(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming
            && self.mode != KwpSessionType::ExtendedDiagnostics
        {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        if cmd.len() != 6 {
            // 1 byte for len and 4 bytes for addr
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            let len: usize = cmd[1] as usize;
            let addr = u32::from_le_bytes(cmd[2..6].try_into().unwrap());

            self.check_addr(addr, true)?;
            self.check_addr(addr + len as u32 - 1, true)?;

            unsafe {
                let mut buf = [0u8; 0xFF];
                let dest_ptr = buf.as_mut_ptr();

                let ptr = core::ptr::NonNull::new_unchecked(addr as *mut u8);
                ptr.copy_to_nonoverlapping(NonNull::new_unchecked(dest_ptr), len);
                Ok(self.make_positive_reply(cmd[0], &buf))
            }
        }
    }

    fn routine_start(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        // We want 2 bytes for number of 8192 blocks (LE)
        // 4 bytes for start address (LE)
        if cmd.len() < 2 {
            // At least 1 arg for LID
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if cmd[1] == 0xE0 {
            if cmd.len() != 8 {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }
            let start_addr = u32::from_le_bytes(cmd[2..6].try_into().unwrap());
            let num_blocks = u16::from_le_bytes(cmd[6..8].try_into().unwrap());

            // Mark app as erased now
            if let Err(e) = bl_info::mutate_bootloader_info(&mut self.nvm, |info| {
                info.flashing_not_done = 0xFF;
                info.application_crc = 0xFFFF_FFFF
            }) {
                defmt::error!("Bootloader info mutate error: {}", e);
                return Err(KwpError::GeneralReject);
            }

            // Do routine
            self.pending_operation = PendingOperation::FlashErase {
                start: start_addr,
                total_sectors: num_blocks as u32,
                current: 0,
            };
            Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
        } else if cmd[1] == 0xE1 {
            // Flash check routine [CRC32, Start Addr (4), End Addr (4)]
            if cmd.len() != 14 {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }
            let targ_crc = u32::from_le_bytes(cmd[2..6].try_into().unwrap());
            let start_addr = u32::from_le_bytes(cmd[6..10].try_into().unwrap());
            let end_addr = u32::from_le_bytes(cmd[10..14].try_into().unwrap());
            // Just check that addrs are valid
            self.check_addr(start_addr, false)?;
            self.check_addr(end_addr, false)?;
            // Check that start < end
            if end_addr <= start_addr {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            } else {
                let start = start_addr as *const u8;
                let slice = unsafe {
                    core::ptr::slice_from_raw_parts(start, (end_addr - start_addr) as usize)
                        .as_ref()
                        .unwrap()
                };
                let result = embedded_crc32c::crc32c(slice);
                if result == targ_crc {
                    // CRC32 OK, now make a CRC of the entire app flash region, and write it to the app
                    if let Err(e) = bl_info::mutate_bootloader_info(&mut self.nvm, |info| {
                        info.flashing_not_done = 0;
                        info.application_crc = app_crc()
                    }) {
                        defmt::error!("Bootloader info mutate error: {}", e);
                        return Err(KwpError::GeneralReject);
                    }

                    Ok(self.make_positive_reply(cmd[0], &[0xE1, 0x01]))
                } else {
                    defmt::error!(
                        "CRC Failed: Target: 0x{:08X}, actual: 0x{:08X}",
                        targ_crc,
                        result
                    );
                    Ok(self.make_positive_reply(cmd[0], &[0xE1, 0x00]))
                }
            }
        } else {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        }
    }

    fn routine_results(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        // We want 2 bytes for number of 8192 blocks (LE)
        // 4 bytes for start address (LE)
        if cmd.len() != 2 {
            // 1 arg (Always E0 = Flash Erase routine)
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if cmd[1] != 0xE0 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if let Some(completed) = &self.completed_operation {
            match completed {
                CompletedOperation::FlashErase(res) => {
                    if let Err(e) = res {
                        defmt::error!("Flash erase error: {}", e);
                        self.completed_operation = None;
                        Ok(self.make_positive_reply(cmd[0], &[0xE0, 0x01]))
                    } else {
                        self.completed_operation = None;
                        Ok(self.make_positive_reply(cmd[0], &[0xE0, 0x00]))
                    }
                }
            }
        } else {
            Err(KwpError::RoutineNotComplete)
        }
    }

    fn start_download(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        if cmd.len() != 10 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            // 0..2 -> Address
            // 3 -> Format (00 is only supported)
            // 4..8 -> Size
            let addr = u32::from_le_bytes(cmd[1..5].try_into().unwrap());
            let fmt = cmd[5];
            let size = u32::from_le_bytes(cmd[6..10].try_into().unwrap());
            if fmt != 0 || addr < APP_ADDR_START || addr + size > APP_ADDR_END {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            } else {
                // Valid params, lets start flashing
                const BLOCK_SIZE: u16 = 1024;
                let bs = [(BLOCK_SIZE >> 8) as u8, (BLOCK_SIZE & 0xFF) as u8];
                self.pending_operation = PendingOperation::Flashing {
                    blk_id: 0,
                    current_addr: addr,
                };
                Ok(self.make_positive_reply(cmd[0], &bs))
            }
        }
    }

    fn transfer_data(&mut self, cmd: &[u8]) -> ServerResult {
        if let PendingOperation::Flashing {
            blk_id,
            current_addr,
        } = &mut self.pending_operation
        {
            if cmd.len() > 2 {
                let req_blk_id = cmd[1];
                let data_size = cmd.len() - 2;
                if req_blk_id == *blk_id && data_size % 4 == 0 {
                    let addr = *current_addr as *mut u32;
                    // Copy to 4 byte aligned array
                    self.flash_buf[..cmd.len() - 2].copy_from_slice(&cmd[2..]);
                    // Write to aligned
                    unsafe {
                        let source: &[u32] = core::slice::from_raw_parts(
                            self.flash_buf.as_ptr() as *const u32,
                            data_size / 4,
                        );
                        if self
                            .nvm
                            .write_flash_from_slice(addr, source, nvm::WriteGranularity::Page)
                            .is_err()
                        {
                            Err(KwpError::TransferSuspended)?;
                        }
                        *current_addr += data_size as u32;
                        *blk_id += 1;
                        Ok(self.make_positive_reply(cmd[0], &[0x00]))
                    }
                } else {
                    // Mismatch
                    Err(KwpError::TransferSuspended)
                }
            } else {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            }
        } else {
            Err(KwpError::ConditionsNotCorrectRequestSequenceError)
        }
    }

    fn transfer_exit(&mut self, cmd: &[u8]) -> ServerResult {
        if let PendingOperation::Flashing { .. } = &mut self.pending_operation {
            self.pending_operation = PendingOperation::None;
            Ok(self.make_positive_reply(cmd[0], &[0]))
        } else {
            Err(KwpError::ConditionsNotCorrectRequestSequenceError)
        }
    }
}
