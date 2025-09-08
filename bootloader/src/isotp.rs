use atsamd_hal::{clock::v2::pclk, fugit::ExtU64, rtic_time::Monotonic};
use bsp::can_deps::Capacities;
use mcan::{
    embedded_can::{Id, StandardId},
    message::tx::{AnyMessage, ClassicFrameType, FrameType, Message, MessageBuilder},
    tx_buffers::DynTx,
};
use rtic_sync::{
    arbiter::Arbiter,
    signal::{Signal, SignalReader, SignalWriter},
};

use crate::Mono;

#[inline(always)]
fn make_fc_ok(stmin: u8, bs: u8) -> [u8; 8] {
    [0x30, bs, stmin, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
}

#[inline(always)]
fn make_fc_reject() -> [u8; 8] {
    [0x32, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
}

fn write(
    id: StandardId,
    data: [u8; 8],
    tx: &mut mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>,
) -> Result<(), ()> {
    let msg = Message::new(MessageBuilder {
        id: Id::Standard(id),
        frame_type: FrameType::Classic(ClassicFrameType::Data(&data)),
        store_tx_event: None,
    })
    .unwrap();
    tx.transmit_queued(msg).map_err(|_| ())
}

pub fn make_isotp_endpoint<'a>(
    tx_id: StandardId,
    rx_id: StandardId,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    ready_signal: &'static Signal<IsotpCtsMsg>,
    rx_signal: &'static Signal<(usize, [u8; 4096])>,
) -> (IsoTpInterruptHandler, IsotpConsumer) {
    let (tx_tx_signal, rx_tx_signal) = ready_signal.split();
    let (tx_rx_ready, rx_rx_ready) = rx_signal.split();
    (
        IsoTpInterruptHandler {
            rx_id,
            tx_id,
            rx_ready: tx_rx_ready,
            rx_state: IsoTpMode::Idle,
            can_tx: can_tx,
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

pub struct IsoTpInterruptHandler {
    pub rx_id: StandardId,
    tx_id: StandardId,
    rx_ready: SignalWriter<'static, (usize, [u8; 4096])>,
    rx_state: IsoTpMode,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    tx_clear_to_send: SignalWriter<'static, IsotpCtsMsg>,
}

impl IsoTpInterruptHandler {
    pub fn on_frame_rx(&mut self, data: &[u8], ecu_stmin: u8, ecu_bs: u8) {
        if data.len() == 8 {
            let pci = data[0] & 0xF0;
            match pci {
                0x00 if data[0] < 8 => {
                    let size = data[0] as usize;
                    let buf = &data[1..1 + size];
                    let mut tmp = [0; 4096];
                    tmp[..size].copy_from_slice(buf);
                    self.rx_ready.write((size, tmp));
                }
                0x10 => {
                    if let Some(mut can_tx) = self.can_tx.try_access() {
                        // Accept
                        let size = (((data[0] & 0x0F) as u16) << 8 | (data[1] as u16)) as usize;
                        let mut buf = [0; 4096];
                        buf[..6].copy_from_slice(&data[2..]);
                        if write(self.tx_id, make_fc_ok(ecu_stmin, ecu_bs), &mut can_tx).is_ok() {
                            self.rx_state = IsoTpMode::Rx {
                                stmin: ecu_stmin,
                                bs: ecu_bs,
                                buf: buf,
                                rx_count: 0,
                                buf_pos: 6,
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
                        buf_pos,
                        targ_size,
                    } = &mut self.rx_state
                    {
                        *rx_count += 1;
                        let max = core::cmp::min(7, *targ_size - *buf_pos);
                        buf[*buf_pos..*buf_pos + max].copy_from_slice(&data[1..1 + max]);
                        *buf_pos += max;
                        if buf_pos == targ_size {
                            // Done
                            self.rx_ready.write((*buf_pos, *buf));
                            self.rx_state = IsoTpMode::Idle;
                        } else if *bs != 0 && rx_count == bs {
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
                    if data[0] == 0x30 {
                        self.tx_clear_to_send.write(IsotpCtsMsg::Ok {
                            stmin: data[2],
                            bs: data[1],
                        });
                    } else {
                        // Reject
                        self.tx_clear_to_send.write(IsotpCtsMsg::Reject);
                    }
                }
                _ => {}
            }
        }
    }
}

#[derive(Clone, Copy)]
pub enum IsoTpMode {
    Rx {
        stmin: u8,
        bs: u8,
        buf: [u8; 4096],
        rx_count: u8,
        buf_pos: usize,
        targ_size: usize,
    },
    Idle,
}

#[derive(Clone, Copy)]
pub enum IsotpCtsMsg {
    Ok { stmin: u8, bs: u8 },
    Reject,
}

pub struct IsotpConsumer {
    tx_id: StandardId,
    rx_ready: SignalReader<'static, (usize, [u8; 4096])>,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    rx_clear_to_send: SignalReader<'static, IsotpCtsMsg>,
}

impl IsotpConsumer {
    pub async fn write_payload(&mut self, buf: &[u8]) -> Result<(), ()> {
        if buf.len() < 8 {
            let mut tx_buf = [0; 8];
            tx_buf[0] = buf.len() as u8;
            tx_buf[1..1 + buf.len()].copy_from_slice(&buf);
            write(self.tx_id, tx_buf, &mut *self.can_tx.access().await)
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
                match Mono::timeout_after(1000u64.millis(), self.rx_clear_to_send.wait()).await {
                    Err(_) => {
                        break Err(());
                    }
                    Ok(fc_result) => {
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
                                        Mono::delay(100u64.micros()).await;
                                    } else {
                                        Mono::delay((stmin as u64).micros()).await;
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
                                break 'outer Err(());
                            }
                        }
                    }
                }
            }
        }
    }

    pub async fn read_payload(&mut self) -> (usize, [u8; 4096]) {
        self.rx_ready.wait().await
    }
}
