//! Blocking ISOTP module for sending and receiving data

use core::{
    cmp::min,
    sync::atomic::{AtomicU8, Ordering},
};

use atsamd_hal::clock::v2::types::Can0;
use bsp::can_deps::Capacities;
use mcan::{
    embedded_can::Id,
    message::tx::{AnyMessage, ClassicFrameType, FrameType, Message, MessageBuilder},
    tx_buffers::DynTx,
};

use crate::{CAN_ID_TX, INSTANCE};

pub static ST_MIN_EGS: AtomicU8 = AtomicU8::new(10);
pub static BS_EGS: AtomicU8 = AtomicU8::new(0x20);

pub struct IsoTp {
    buffer: [u8; 4096],
    buffer_len: usize,
    buffer_pos: usize,
    mode: IsoTpMode,
}

impl Default for IsoTp {
    fn default() -> Self {
        Self {
            buffer: [0; 4096],
            buffer_len: 0,
            buffer_pos: 0,
            mode: IsoTpMode::Idle,
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum IsoTpMode {
    Rx {
        rx_count: u8,
    },
    Tx {
        pci: u8,
    },
    #[default]
    Idle,
}

impl IsoTp {
    pub fn on_packet_rx<'a>(
        &mut self,
        data: &[u8],
        tx: &mut mcan::tx_buffers::Tx<'a, Can0, Capacities>,
    ) -> Option<&[u8]> {
        if data.len() == 8 {
            match data[0] & 0xF0 {
                0x00 => {
                    // Single frame Rx
                    let len = data[0] & 0x0F;
                    if len < 8 {
                        self.buffer[..len as usize].copy_from_slice(&data[1..1 + len as usize]);
                        Some(&self.buffer[..len as usize])
                    } else {
                        None
                    }
                }
                0x10 => {
                    // Start Rx
                    let expected_size = ((data[0] & 0x0F) as usize) << 8 | data[1] as usize;
                    defmt::debug!("ISOTP Long Rx expects {} bytes", expected_size);
                    // Safety, since it is 12 bit number, it is impossible to exceed the size of buffer!
                    // Send flow control
                    let tx_fc = Message::new(MessageBuilder {
                        id: Id::Standard(CAN_ID_TX),
                        frame_type: FrameType::Classic(ClassicFrameType::Data(&[
                            0x30,
                            BS_EGS.load(Ordering::Relaxed),
                            ST_MIN_EGS.load(Ordering::Relaxed),
                            0,
                            0,
                            0,
                            0,
                            0,
                        ])),
                        store_tx_event: None,
                    })
                    .unwrap();
                    if tx.transmit_queued(tx_fc).is_ok() {
                        self.buffer_pos = 6;
                        self.buffer_len = expected_size;
                        self.mode = IsoTpMode::Rx { rx_count: 0 };
                        self.buffer[..6].copy_from_slice(&data[2..]);
                    }
                    None
                }
                0x20 => {
                    if let IsoTpMode::Rx { rx_count } = &mut self.mode {
                        // Only do this in Rx mode
                        let max_copy = min(7, self.buffer_len - self.buffer_pos);
                        self.buffer[self.buffer_pos..self.buffer_pos + max_copy]
                            .copy_from_slice(&data[1..1 + max_copy]);
                        self.buffer_pos += max_copy;
                        *rx_count = rx_count.wrapping_add(1);
                        if self.buffer_pos == self.buffer_len {
                            Some(&self.buffer[..self.buffer_pos])
                        } else {
                            let bs = BS_EGS.load(Ordering::Relaxed);
                            if bs != 0 && *rx_count == bs {
                                *rx_count = 0;
                                let tx_fc = Message::new(MessageBuilder {
                                    id: Id::Standard(CAN_ID_TX),
                                    frame_type: FrameType::Classic(ClassicFrameType::Data(&[
                                        0x30,
                                        bs,
                                        ST_MIN_EGS.load(Ordering::Relaxed),
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                    ])),
                                    store_tx_event: None,
                                })
                                .unwrap();
                                let _ = tx.transmit_queued(tx_fc);
                            }

                            None
                        }
                    } else {
                        None
                    }
                }
                0x30 => {
                    if let IsoTpMode::Tx { pci } = &mut self.mode {
                        if data[0] == 0x30 {
                            let bs = data[1];
                            let mut bs_count = bs;
                            let st_min = data[2];
                            let mut tx_complete = false;
                            let mut packet = [0u8; 8];
                            while bs == 0 || bs_count > 0 {
                                let max_tx = min(7, self.buffer_len - self.buffer_pos);
                                packet[0] = *pci;
                                packet[1..1 + max_tx].copy_from_slice(
                                    &self.buffer[self.buffer_pos..self.buffer_pos + max_tx],
                                );

                                let sf = Message::new(MessageBuilder {
                                    id: Id::Standard(CAN_ID_TX),
                                    frame_type: FrameType::Classic(ClassicFrameType::Data(&packet)),
                                    store_tx_event: None,
                                })
                                .unwrap();
                                if tx.transmit_queued(sf).is_ok() {
                                    self.buffer_pos += max_tx;
                                    if self.buffer_pos >= self.buffer_len {
                                        tx_complete = true;
                                        break;
                                    }
                                    if st_min != 0 {
                                        // Wait
                                        let start = INSTANCE.now();
                                        while INSTANCE.now() - start < st_min as u64 {
                                            core::hint::spin_loop();
                                        }
                                    } else {
                                        // Needed to let Tx msg clear buffer
                                        for _ in 0..1000 {
                                            core::hint::spin_loop();
                                        }
                                    }

                                    // Handle PCI Increment
                                    *pci += 1;
                                    if *pci == 0x30 {
                                        *pci = 0x20;
                                    }

                                    if bs != 0 {
                                        bs_count -= 1;
                                    }
                                } else {
                                    defmt::error!("CAN TX failed");
                                    tx_complete = true;
                                    break;
                                }
                            }
                            if tx_complete {
                                self.mode = IsoTpMode::Idle;
                            }
                        } else {
                            // Other 0x30s (0x31 and 32 are reject/aborts)
                            self.mode = IsoTpMode::Idle;
                        }
                    }
                    None
                }
                _ => {
                    // Unknown PCI
                    None
                }
            }
        } else {
            None
        }
    }

    /// Blocks until write completes
    pub fn write_payload<'a>(
        &mut self,
        data: &[u8],
        tx: &mut mcan::tx_buffers::Tx<'a, Can0, Capacities>,
    ) {
        if data.len() < 8 {
            let mut packet_buf: [u8; 8] = [0; 8];
            packet_buf[0] = data.len() as u8;
            packet_buf[1..1 + data.len()].copy_from_slice(data);
            let sf = Message::new(MessageBuilder {
                id: Id::Standard(CAN_ID_TX),
                frame_type: FrameType::Classic(ClassicFrameType::Data(&packet_buf)),
                store_tx_event: None,
            })
            .unwrap();
            if tx.transmit_queued(sf).is_err() {
                defmt::error!("Short packet Tx failed")
            }
        } else {
            // Larger than 8 bytes
            // Write the start frame
            let mut packet = [0u8; 8];
            packet[0] = 0x10 | (((data.len() >> 8) & 0x0F) as u8);
            packet[1] = (data.len() & 0xFF) as u8;
            packet[2..8].copy_from_slice(&data[..6]);
            let sf = Message::new(MessageBuilder {
                id: Id::Standard(CAN_ID_TX),
                frame_type: FrameType::Classic(ClassicFrameType::Data(&packet)),
                store_tx_event: None,
            })
            .unwrap();
            if tx.transmit_queued(sf).is_ok() {
                // Setup for sending
                let max = min(4095, data.len());
                self.buffer[..max].copy_from_slice(&data[..max]);
                self.buffer_len = max;
                self.buffer_pos = 6;
                self.mode = IsoTpMode::Tx { pci: 0x21 };
            }
        }
    }
}
