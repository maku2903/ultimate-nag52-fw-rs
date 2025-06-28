use core::{cell::RefCell, ops::{DerefMut, Range, RangeInclusive}};

use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex};

use crate::extern_flash::{ExternFlash};


#[derive(defmt::Format, Clone, Copy)]
pub enum PartitionError {
    StartOutOfRange,
    EndOutOfRange,
    Busy,
    InvalidEraseSize
}

pub struct AllPartitions {
    pub ota: Partition,
    pub misc: Partition,
    pub usb_fat32: Partition
}

pub struct Partition {
    start_addr: usize,
    end_addr: usize,
    inner: &'static embassy_sync::mutex::Mutex<CriticalSectionRawMutex, ExternFlash>
}

impl Partition {
    pub (super) fn new(start: usize, end: usize, inner: &'static embassy_sync::mutex::Mutex<CriticalSectionRawMutex, ExternFlash>) -> Self {
        Self {
            start_addr: start,
            end_addr: end,
            inner,
        }
    }

    #[inline]
    pub fn size(&self) -> usize {
        self.end_addr - self.start_addr
    }

    #[inline]
    pub fn check_addr_range(&self, start: usize, len: usize) -> Result<(), PartitionError> {
        let f_len = self.size();
        if start > f_len {
            Err(PartitionError::StartOutOfRange)
        } else if (start+len) > f_len {
            Err(PartitionError::EndOutOfRange)
        } else {
            Ok(())
        }
    }

    pub async fn write(&self, addr: usize, data: &[u8]) -> Result<(), PartitionError> {
        self.check_addr_range(addr, data.len())?;
        let mut flash = self.inner.lock().await;
        flash.write((addr + self.start_addr) as u32, data);
        Ok(())
    }

    pub fn try_write(&self, addr: usize, data: &[u8]) -> Result<(), PartitionError> {
        self.check_addr_range(addr, data.len())?;
        if let Ok(mut flash) = self.inner.try_lock() {
            flash.write((addr + self.start_addr) as u32, data);
            Ok(())
        } else {
            Err(PartitionError::Busy)
        }
    }

    pub async fn read(&self, addr: usize, dest_buf: &mut [u8]) -> Result<(), PartitionError> {
        self.check_addr_range(addr, dest_buf.len())?;
        let mut flash = self.inner.lock().await;
        flash.read((addr + self.start_addr) as u32, dest_buf);
        Ok(())
    }

    pub fn try_read(&self, addr: usize, dest_buf: &mut [u8]) -> Result<(), PartitionError> {
        self.check_addr_range(addr, dest_buf.len())?;
        if let Ok(mut flash) = self.inner.try_lock() {
            flash.read((addr + self.start_addr) as u32, dest_buf);
            Ok(())
        } else {
            Err(PartitionError::Busy)
        }
    }

    pub fn try_erase(&self, addr: usize, len: usize) -> Result<(), PartitionError> {
        self.check_addr_range(addr, addr + len - 1)?;
        // Not aligned correctly
        if addr % ExternFlash::E_SECTOR_SIZE != 0 || len % ExternFlash::E_SECTOR_SIZE != 0 {
            return Err(PartitionError::InvalidEraseSize)
        }
        if let Ok(mut flash) = self.inner.try_lock() {
            flash.erase_region((addr + self.start_addr) as u32, len as u32);
            Ok(())
        } else {
            Err(PartitionError::Busy)
        }
    }

    pub unsafe fn with_qspi<F: FnOnce(&mut ExternFlash)>(&self, f: F) -> Result<(), PartitionError> {
        if let Ok(mut flash) = self.inner.try_lock() {
            f(flash.deref_mut());
            Ok(())
        } else {
            Err(PartitionError::Busy)
        }
    }
}
