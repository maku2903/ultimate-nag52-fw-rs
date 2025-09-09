//! ISO 15765-2 ISOTP Layer for MCAN CAN devices
//!
//! Assumptions
//! * Padding of each frame to 8 bytes
//! * Non extended CAN or extended ISO-TP addressing

use mcan::{
    core::CanId,
    embedded_can::Id,
    message::tx::{AnyMessage, ClassicFrameType, FrameType, MessageBuilder},
    messageram::Capacities,
    tx_buffers::DynTx,
};
use rtic_sync::{
    arbiter::Arbiter,
    signal::{Signal, SignalReader, SignalWriter},
};

use futures::FutureExt;

use crate::isotp_endpoints::SharedIsoTpBuf;

#[inline(always)]
fn make_fc_ok(stmin: u8, bs: u8) -> [u8; 8] {
    [0x30, bs, stmin, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
}

#[inline(always)]
fn make_fc_reject() -> [u8; 8] {
    [0x32, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
}

fn write<'a, ID: CanId + 'a, C: Capacities>(
    id: Id,
    data: [u8; 8],
    tx: &mut mcan::tx_buffers::Tx<'a, ID, C>,
) -> nb::Result<(), mcan::tx_buffers::Error> {
    let msg = C::TxMessage::new(MessageBuilder {
        id,
        frame_type: FrameType::Classic(ClassicFrameType::Data(&data)),
        store_tx_event: None,
    })
    .unwrap();
    tx.transmit_queued(msg)
}

pub fn make_isotp_endpoint<'a, ID: CanId + 'a, C: Capacities, const N: usize>(
    tx_id: Id,
    rx_id: Id,
    can_tx: &'a Arbiter<mcan::tx_buffers::Tx<'a, ID, C>>,
    ready_signal: &'a Signal<IsotpCtsMsg>,
    rx_signal: &'a Signal<SharedIsoTpBuf<N>>,
) -> (
    IsoTpInterruptHandler<'a, ID, C, N>,
    IsotpConsumer<'a, ID, C, N>,
) {
    let (tx_tx_signal, rx_tx_signal) = ready_signal.split();
    let (tx_rx_ready, rx_rx_ready) = rx_signal.split();
    (
        IsoTpInterruptHandler {
            rx_id,
            tx_id,
            rx_ready: tx_rx_ready,
            rx_state: IsoTpMode::Idle,
            can_tx,
            tx_clear_to_send: tx_tx_signal,
        },
        IsotpConsumer {
            tx_id,
            rx_ready: rx_rx_ready,
            can_tx,
            rx_clear_to_send: rx_tx_signal,
        },
    )
}

/// Part of the ISOTP handler that is used for MCAN interrupts:
///
/// Usage with RTIC:
/// ```
/// #[task(priority = 1, binds=CAN0, local=[can0_interrupts, can0_fifo0, isotp_isr])]
/// fn can0(cx: can0::Context) {
///     for interrupt in cx.local.can0_interrupts.iter_flagged() {
///         match interrupt {
///             Interrupt::RxFifo0NewMessage => {
///                 for msg in cx.local.can0_fifo0.into_iter() {
///                     if msg.id() == Id::Standard(cx.local.isotp_isr.rx_id) {
///                         const BS: u8 = 8;
///                         const STMIN: u8 = 20;
///                         cx.local.isotp_isr.on_frame_rx(msg.data(), STMIN, BS);
///                     }
///                 }
///             }
///             _ => {}
///         }
///     }
/// }
/// ```
pub struct IsoTpInterruptHandler<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> {
    pub rx_id: Id,
    tx_id: Id,
    rx_ready: SignalWriter<'a, SharedIsoTpBuf<N>>,
    rx_state: IsoTpMode<N>,
    can_tx: &'a Arbiter<mcan::tx_buffers::Tx<'a, ID, C>>,
    tx_clear_to_send: SignalWriter<'a, IsotpCtsMsg>,
}

impl<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> IsoTpInterruptHandler<'a, ID, C, N> {
    /// Called when each frame is received
    pub fn on_frame_rx(&mut self, data: &[u8], ecu_stmin: u8, ecu_bs: u8) {
        if data.len() == 8 {
            let pci = data[0] & 0xF0;
            match pci {
                0x00 if data[0] < 8 => {
                    // Single frame handling
                    let size = data[0] as usize;
                    let buf = &data[1..1 + size];
                    let mut tmp = SharedIsoTpBuf::new();
                    tmp.data[..size].copy_from_slice(buf);
                    tmp.size = size;
                    self.rx_ready.write(tmp);
                }
                0x10 => {
                    // Start of a large payload
                    if let Some(mut can_tx) = self.can_tx.try_access() {
                        // Accept
                        let size = (((data[0] & 0x0F) as u16) << 8 | (data[1] as u16)) as usize;

                        let mut shared_buffer = SharedIsoTpBuf::new();
                        shared_buffer.data[..6].copy_from_slice(&data[2..]);
                        shared_buffer.size = size;

                        if write(self.tx_id, make_fc_ok(ecu_stmin, ecu_bs), &mut can_tx).is_ok() {
                            self.rx_state = IsoTpMode::Rx {
                                stmin: ecu_stmin,
                                bs: ecu_bs,
                                buf: shared_buffer,
                                rx_count: 0,
                                targ_size: size,
                            };
                        }
                    }
                }
                0x20 => {
                    if let IsoTpMode::Rx {
                        stmin,
                        bs,
                        buf,
                        rx_count,
                        targ_size,
                    } = &mut self.rx_state
                    {
                        // Multi-frame
                        *rx_count += 1;
                        let max = core::cmp::min(7, *targ_size - buf.size);
                        buf.data[buf.size..buf.size + max].copy_from_slice(&data[1..1 + max]);
                        buf.size += max;
                        if buf.size == *targ_size {
                            // Completed reception of  data
                            self.rx_ready.write(*buf);
                            self.rx_state = IsoTpMode::Idle;
                        } else if *bs != 0 && rx_count == bs {
                            // Try to transmit a FC OK CAN message
                            if self
                                .can_tx
                                .try_access()
                                .and_then(|mut tx| {
                                    write(self.tx_id, make_fc_ok(*stmin, *bs), &mut tx).ok()
                                })
                                .is_none()
                            {
                                // Error
                                self.rx_state = IsoTpMode::Idle;
                            } else {
                                // Ok
                                *rx_count = 0;
                            }
                        }
                    }
                }
                0x30 => {
                    // FC from other end
                    if data[0] == 0x30 {
                        self.tx_clear_to_send.write(IsotpCtsMsg::Ok {
                            stmin: data[2],
                            bs: data[1],
                        });
                    } else {
                        // Reject for 0x31 and 0x32
                        self.tx_clear_to_send.write(IsotpCtsMsg::Reject);
                    }
                }
                _ => {}
            }
        }
    }
}

#[derive(Clone, Copy)]
/// Used by the ISR Handler to determine
/// what state to be in
pub enum IsoTpMode<const N: usize> {
    /// In reception mode.
    /// Timing parameters are from the
    /// other ends flow-control msg
    Rx {
        stmin: u8,
        bs: u8,
        buf: SharedIsoTpBuf<N>,
        rx_count: u8,
        targ_size: usize,
    },
    /// No message reception in progress
    Idle,
}

#[derive(Clone, Copy)]
/// Message sent to the Thread handler
pub enum IsotpCtsMsg {
    /// Thread handler can continue with sending
    Ok { stmin: u8, bs: u8 },
    /// Receiver rejected
    Reject,
}

/// ISOTP Transmission error
pub enum IsoTpTxErr {
    /// Receiver rejected the request
    Rejected,
    /// Timeout waiting for receipient flow control
    Timeout,
    /// Internal CAN error
    McanErr(nb::Error<mcan::tx_buffers::Error>),
}

impl From<nb::Error<mcan::tx_buffers::Error>> for IsoTpTxErr {
    fn from(value: nb::Error<mcan::tx_buffers::Error>) -> Self {
        Self::McanErr(value)
    }
}

/// ISOTP thread part
///
/// This is designed to be used in an async context:
///
/// ```
/// let rx =  isotp_thread.read_payload().await;
/// // Do something
/// let tx_result = isotp_thread.write_payload(&[0x7F, 0x21, 0x12]).await;
/// ```
pub struct IsotpConsumer<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> {
    tx_id: Id,
    rx_ready: SignalReader<'a, SharedIsoTpBuf<N>>,
    can_tx: &'a Arbiter<mcan::tx_buffers::Tx<'a, ID, C>>,
    rx_clear_to_send: SignalReader<'a, IsotpCtsMsg>,
}

impl<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> IsotpConsumer<'a, ID, C, N> {
    /// Attempts to write an ISOTP payload to the CAN network
    pub async fn write_payload<M: embedded_hal_async::delay::DelayNs>(
        &mut self,
        mono: &mut M,
        buf: &[u8],
    ) -> Result<(), IsoTpTxErr> {
        if buf.len() < 8 {
            let mut tx_buf = [0; 8];
            tx_buf[0] = buf.len() as u8;
            tx_buf[1..1 + buf.len()].copy_from_slice(buf);
            write(self.tx_id, tx_buf, &mut *self.can_tx.access().await)?;
            Ok(())
        } else {
            // We can send
            let mut tx_buf = [0; 8];
            tx_buf[0] = 0x10u8 | ((buf.len() >> 8) & 0x0F) as u8;
            tx_buf[1] = (buf.len() & 0xFF) as u8;
            tx_buf[2..].copy_from_slice(&buf[..6]);
            write(self.tx_id, tx_buf, &mut *self.can_tx.access().await)?;
            // Wait for clear to send a block
            let mut pci = 0x21;
            let mut buf_pos = 6;
            'outer: loop {
                futures::select_biased! {
                    _ = mono.delay_ms(1000).fuse() => {
                        break Err(IsoTpTxErr::Timeout)
                    },
                    fc_result = self.rx_clear_to_send.wait().fuse() => {
                        match fc_result {
                            IsotpCtsMsg::Ok { stmin, bs } => {
                                let mut bs_count = bs;
                                while bs == 0 || bs_count != 0 {
                                    // Send frame
                                    tx_buf[0] = pci;
                                    let max_copy = core::cmp::min(7, buf.len() - buf_pos);
                                    tx_buf[1..1 + max_copy]
                                        .copy_from_slice(&buf[buf_pos..buf_pos + max_copy]);
                                    write(self.tx_id, tx_buf, &mut *self.can_tx.access().await)?;
                                    buf_pos += max_copy;
                                    if buf_pos == buf.len() {
                                        // Tx complete
                                        break 'outer Ok(());
                                    }

                                    if stmin == 0 {
                                        mono.delay_us(100).await;
                                    } else {
                                        mono.delay_ms(stmin as u32).await;
                                    }
                                    pci = 0x20 | (pci + 1) & 0x0F;
                                    bs_count = bs_count.wrapping_sub(1);
                                }
                            }
                            IsotpCtsMsg::Reject => {
                                write(
                                    self.tx_id,
                                    make_fc_reject(),
                                    &mut *self.can_tx.access().await,
                                )?;
                                break 'outer Err(IsoTpTxErr::Rejected);
                            }
                        }
                    }
                }
            }
        }
    }

    /// Reads a received ISOTP payload
    pub async fn read_payload(&mut self) -> SharedIsoTpBuf<N> {
        self.rx_ready.wait().await
    }
}
