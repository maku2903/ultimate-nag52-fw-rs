use core::{cmp::min, ops::DerefMut};

use crate::atsamd_hal::{
    pac::Peripherals,
    prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin,
    qspi::{Command, OneShot, Qspi},
    usb::UsbBus,
};
use bsp::LedExtFlash;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, once_lock::OnceLock,
};
use embassy_time::Timer;
use static_cell::StaticCell;
use usbd_storage::{
    subclass::scsi::{Scsi, ScsiCommand},
    transport::{
        bbb::{BulkOnly, BulkOnlyError},
        TransportError,
    },
};

use crate::extern_flash::partition::{AllPartitions, Partition};

/// QSPI partitioning (TOTAL: 16MB)
///
/// 0x000000 - 0x0FFFFF (1MB)  - OTA Partition (2nd pending FW)
/// 0x100000 - 0x1FFFFF (1MB)  - Misc data (Maps, cal, ETC)
/// 0x200000 - 0xFFFFFF (14MB) - FAT32 FS (Read Only)
pub mod partition;

pub struct ExternFlash {
    led: bsp::LedExtFlash,
    qspi: Qspi<OneShot>,
}

impl ExternFlash {
    pub const W_PAGE_SIZE: usize = 256;
    pub const E_SECTOR_SIZE: usize = 4 * 1024;
    pub const E_BLK32_SIZE: usize = 32 * 1024;
    pub const E_BLK64_SIZE: usize = 64 * 1024;

    pub fn new(controller: Qspi<OneShot>, led: LedExtFlash) -> ExternFlash {
        ExternFlash {
            led,
            qspi: controller,
        }
    }

    pub async fn init(&mut self) {
        Timer::after_millis(5).await;
        self.led.set_high().unwrap();
        self.wait_ready();
        self.qspi.run_command(Command::EnableReset).unwrap();
        self.qspi.run_command(Command::Reset).unwrap();
        Timer::after_millis(1).await; // t_RST = 30us
        self.qspi.run_command(Command::WriteEnable).unwrap();
        self.qspi
            .write_command(Command::WriteStatus2, &[0x02])
            .unwrap();
        self.led.set_low().unwrap();
    }

    pub fn into_partitions(self) -> AllPartitions {
        static SHARED_FLASH: StaticCell<
            embassy_sync::mutex::Mutex<CriticalSectionRawMutex, ExternFlash>,
        > = StaticCell::new();
        let shared_flash = SHARED_FLASH.init(Mutex::new(self));
        AllPartitions {
            ota: Partition::new(0x000000, 0x100000, shared_flash),
            misc: Partition::new(0x100000, 0x200000, shared_flash),
            usb_fat32: Partition::new(0x200000, 0x1000000, shared_flash),
        }
    }

    pub fn write(&mut self, addr: u32, buf: &[u8]) {
        defmt::debug!("Writing @ {:08X}", addr);
        if buf.is_empty() {
            return;
        }
        self.led.set_high().unwrap();
        self.qspi.run_command(Command::WriteEnable).unwrap();
        self.qspi.write_memory(addr, buf);
        self.wait_ready();
        self.led.set_low().unwrap();
    }

    pub fn read(&mut self, addr: u32, buf: &mut [u8]) {
        if buf.is_empty() {
            return;
        }
        self.led.set_high().unwrap();
        self.wait_ready();
        self.qspi.read_memory(addr, buf);
        self.led.set_low().unwrap();
    }

    fn check_erased(&mut self, start: u32, len: u32) -> bool {
        let mut pos = 0;
        let mut ret = true;
        let mut x: [u8; 256] = [0; 256]; // Buffer for checking the sector
        while pos < len {
            let to_read = core::cmp::min(len - pos, 256) as usize;
            self.wait_ready();
            self.read(start + pos, &mut x[..to_read]);
            if !x.iter().all(|e| *e == 0xFF) {
                ret = false;
                break;
            }
            pos += to_read as u32;
        }
        ret
    }

    pub fn erase_region(&mut self, start_addr: u32, len: u32) {
        defmt::debug!("Erasing region @ {:08X} ({} bytes)", start_addr, len);
        self.led.set_high().unwrap();
        // For now, QSPI only supports the 4K or 64K erase commands
        let mut start = start_addr;
        let mut left = len;
        loop {
            // ping the watchdog before every erase as this can take a while!
            unsafe {
                Peripherals::steal()
                    .wdt
                    .clear()
                    .write(|w| w.clear().bits(0xA5));
            }
            if start % Self::E_BLK64_SIZE as u32 == 0 && left >= Self::E_BLK64_SIZE as u32 {
                defmt::info!("Doing 64K Erase");
                if !self.check_erased(start, Self::E_BLK64_SIZE as u32) {
                    // Not erased, we have to erase it
                    self.qspi.run_command(Command::WriteEnable).unwrap();
                    self.qspi.erase_command(Command::EraseBlock, start).unwrap();
                    self.wait_ready();
                }
                start += Self::E_BLK64_SIZE as u32;
                left -= Self::E_BLK64_SIZE as u32;
            } else {
                // 4K erase
                defmt::info!("Doing 4K Erase");
                if !self.check_erased(start, Self::E_SECTOR_SIZE as u32) {
                    // Not erased, we have to erase it
                    self.qspi.run_command(Command::WriteEnable).unwrap();
                    self.qspi
                        .erase_command(Command::EraseSector, start)
                        .unwrap();
                    self.wait_ready();
                }
                start += Self::E_SECTOR_SIZE as u32;
                left -= Self::E_SECTOR_SIZE as u32;
            }
            defmt::info!("Erase left: {}", left);
            if left == 0 {
                // Done!
                break;
            }
        }
        self.led.set_low().unwrap();
    }

    pub fn qspi(&mut self) -> &mut Qspi<OneShot> {
        &mut self.qspi
    }

    pub fn wait_ready(&mut self) {
        while self.flash_status(Command::ReadStatus) & 0x01 != 0 {}
        while self.flash_status(Command::ReadStatus2) & 0x80 != 0 {}
    }

    /// Returns the contents of the status register indicated by cmd.
    fn flash_status(&mut self, cmd: Command) -> u8 {
        let mut out = [0u8; 1];
        self.qspi.read_command(cmd, &mut out).ok().unwrap();
        out[0]
    }
}

/// Not necessarily `'static`. May reside in some special memory location
pub static USB_TRANSPORT_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
pub const USB_PACKET_SIZE: usize = 64; // 8,16,32,64
pub const MAX_LUN: u8 = 0;
pub static FAT_PARTITION: OnceLock<Mutex<CriticalSectionRawMutex, (Partition, ScsiState)>> =
    OnceLock::new();

#[derive(Default)]
pub struct ScsiState {
    storage_offset: usize,
    sense_key: Option<u8>,
    sense_key_code: Option<u8>,
    sense_qualifier: Option<u8>,
}

impl ScsiState {
    fn reset(&mut self) {
        *self = Self::default()
    }
}

pub fn reset_scsi_state() {
    if let Some(mut s) = FAT_PARTITION.try_get().map(|x| x.try_lock().ok()).flatten() {
        s.1.reset();
    }
}

#[inline]
#[link_section = ".data"]
pub fn process_scsi_command(
    mut command: usbd_storage::subclass::Command<ScsiCommand, Scsi<BulkOnly<UsbBus, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    if let Some(mut lock) = FAT_PARTITION.try_get().map(|x| x.try_lock().ok()).flatten() {
        let (partition, state) = lock.deref_mut();
        // We have partition available
        match command.kind {
            ScsiCommand::TestUnitReady { .. } => {
                command.pass();
            }
            ScsiCommand::Inquiry { .. } => {
                command.try_write_data_all(&[
                    0x00, // periph qualifier, periph device type (Normal, MMC)
                    0x80, // Removable
                    0x03, // SPC-2 compliance
                    0x02, // NormACA, HiSu, Response data format
                    0x20, // 36 bytes in total
                    0x00, // additional fields, none set
                    0x00, // additional fields, none set
                    0x00, // additional fields, none set
                    b'R', b'A', b'N', b'D', b'_', b'A', b'S', b'H', // 8-byte T-10 vendor id
                    b'U', b'L', b'T', b'I', b'M', b'A', b'T', b'E', b' ', b'N', b'A', b'G', b'5',
                    b'2', b' ', b' ', // 16-byte product identification
                    b'2', b'.', b'0', b'0', // 4-byte product revision
                ])?;
                command.pass();
            }
            ScsiCommand::RequestSense { .. } => unsafe {
                command.try_write_data_all(&[
                    0x70,                         // RESPONSE CODE. Set to 70h for information on current errors
                    0x00,                         // obsolete
                    state.sense_key.unwrap_or(0), // Bits 3..0: SENSE KEY. Contains information describing the error.
                    0x00,
                    0x00,
                    0x00,
                    0x00, // INFORMATION. Device-specific or command-specific information.
                    0x00, // ADDITIONAL SENSE LENGTH.
                    0x00,
                    0x00,
                    0x00,
                    0x00,                               // COMMAND-SPECIFIC INFORMATION
                    state.sense_key_code.unwrap_or(0),  // ASC
                    state.sense_qualifier.unwrap_or(0), // ASCQ
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                ])?;
                state.reset();
                command.pass();
            },
            ScsiCommand::ReadCapacity10 { .. } => {
                let n_blocks = (partition.size() / ExternFlash::E_SECTOR_SIZE) as u32;
                let mut data = [0u8; 8];
                let _ = &mut data[0..4].copy_from_slice(&u32::to_be_bytes(n_blocks - 1));
                let _ = &mut data[4..8]
                    .copy_from_slice(&u32::to_be_bytes(ExternFlash::E_SECTOR_SIZE as u32));
                command.try_write_data_all(&data)?;
                command.pass();
            }
            ScsiCommand::ReadCapacity16 { .. } => {
                let n_blocks = (partition.size() / ExternFlash::E_SECTOR_SIZE) as u32;
                let mut data = [0u8; 16];
                let _ = &mut data[0..8].copy_from_slice(&u32::to_be_bytes(n_blocks - 1));
                let _ = &mut data[8..12]
                    .copy_from_slice(&u32::to_be_bytes(ExternFlash::E_SECTOR_SIZE as u32));
                command.try_write_data_all(&data)?;
                command.pass();
            }
            ScsiCommand::ReadFormatCapacities { .. } => {
                let size = (partition.size() / ExternFlash::E_SECTOR_SIZE) as u32;
                let mut data = [0u8; 12];
                let _ = &mut data[0..4].copy_from_slice(&[
                    0x00, 0x00, 0x00, 0x08, // capacity list length
                ]);
                let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(size as u32)); // number of blocks
                data[8] = 0x01; //unformatted media
                let block_length_be = u32::to_be_bytes(ExternFlash::E_SECTOR_SIZE as u32);
                data[9] = block_length_be[1];
                data[10] = block_length_be[2];
                data[11] = block_length_be[3];

                command.try_write_data_all(&data)?;
                command.pass();
            }
            ScsiCommand::Read { lba, len } => unsafe {
                let lba = lba as usize;
                let len = len as usize;
                if state.storage_offset != (len * ExternFlash::E_SECTOR_SIZE) {
                    let start = (ExternFlash::E_SECTOR_SIZE * lba) + state.storage_offset;
                    let end = (ExternFlash::E_SECTOR_SIZE * lba) as usize
                        + (ExternFlash::E_SECTOR_SIZE * len) as usize;
                    let end = min(start + (USB_PACKET_SIZE * 4) as usize, end);
                    let max = end - start;
                    let mut read: [u8; USB_PACKET_SIZE * 4] = [0; USB_PACKET_SIZE * 4];

                    if let Err(e) = partition.try_read(start, &mut read[..max]) {
                        defmt::error!(
                            "QSPI Read failed {} Addr: {:08X}, len: {:08X}",
                            e,
                            start,
                            max
                        );
                        command.fail();
                        return Err(TransportError::Error(BulkOnlyError::InvalidState));
                    } else {
                        // Ok
                        let count = command.write_data(&read[..max])?;
                        state.storage_offset += count;
                        if state.storage_offset == (len * ExternFlash::E_SECTOR_SIZE) as usize {
                            command.pass();
                            state.storage_offset = 0;
                        }
                    }
                } else {
                    command.pass();
                    state.storage_offset = 0;
                }
            },
            ScsiCommand::Write { lba, len } => unsafe {
                // Base addr (Block size multiplier)
                let lba = lba as usize;
                // Length (Number of blocks)
                let len = len as usize;

                if state.storage_offset != (len * ExternFlash::E_SECTOR_SIZE) {
                    if state.storage_offset == 0 {
                        // Erase region
                        if let Err(e) = partition.try_erase(
                            lba * ExternFlash::E_SECTOR_SIZE,
                            len * ExternFlash::E_SECTOR_SIZE,
                        ) {
                            defmt::error!(
                                "Erase failed 0x{:08X} (0x{:08X} len): {}",
                                lba * ExternFlash::E_SECTOR_SIZE,
                                len * ExternFlash::E_SECTOR_SIZE,
                                e
                            );
                            command.fail();
                            return Err(TransportError::Error(BulkOnlyError::InvalidState));
                        }
                    }
                    // Start writing
                    let start = (ExternFlash::E_SECTOR_SIZE * lba) as usize + state.storage_offset;
                    let mut read: [u8; USB_PACKET_SIZE * 4] = [0; USB_PACKET_SIZE * 4];
                    let mut total = 0;
                    let mut cmd_fail = false;
                    loop {
                        match command.read_data(&mut read) {
                            Ok(0) => break,
                            Ok(count) => {
                                if let Err(e) = partition.try_write(start + total, &read[..count]) {
                                    defmt::error!(
                                        "QSPI Write failed to addr {:08X} {} ({} bytes)",
                                        start + total,
                                        e,
                                        count
                                    );
                                    cmd_fail = true;
                                    break;
                                } else {
                                    total += count;
                                }
                            }
                            Err(_) => break,
                        }
                    }

                    if cmd_fail {
                        command.fail();
                        return Err(TransportError::Error(BulkOnlyError::InvalidState));
                    } else {
                        state.storage_offset += total;
                        if state.storage_offset == (len * ExternFlash::E_SECTOR_SIZE) as usize {
                            command.pass();
                            state.storage_offset = 0;
                        }
                    }
                } else {
                    command.pass();
                    state.storage_offset = 0;
                }
            },
            ScsiCommand::ModeSense6 { .. } => {
                command.try_write_data_all(&[
                    0x03, // number of bytes that follow
                    0x00, // the media type is SBC
                    0x00, // not write-protected, no cache-control bytes support
                    0x00, // no mode-parameter block descriptors
                ])?;
                command.pass();
            }
            ScsiCommand::ModeSense10 { .. } => {
                command.try_write_data_all(&[0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?;
                command.pass();
            }
            ref unknown_scsi_kind => {
                unsafe {
                    state.sense_key.replace(0x05); // illegal request Sense Key
                    state.sense_key_code.replace(0x20); // Invalid command operation ASC
                    state.sense_qualifier.replace(0x00); // Invalid command operation ASCQ
                }
                command.fail();
            }
        }
    } else {
        // No partition
        //command.fail();
    }
    Ok(())
}
