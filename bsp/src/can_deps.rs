use atsamd_hal::{can::Dependencies, pac};
use mcan::{message::{rx, tx}, rx_fifo::{Fifo0, RxFifo}};

use crate::{CanRx, CanTx};
use mcan::generic_array::typenum::{U0, U1, U32, U64};
use atsamd_hal::clock::v2::types::Can0;
pub struct Capacities;

impl mcan::messageram::Capacities for Capacities {
    type StandardFilters = U1;
    type ExtendedFilters = U1;
    type RxBufferMessage = rx::Message<64>;
    type DedicatedRxBuffers = U0;
    type RxFifo0Message = rx::Message<64>;
    type RxFifo0 = U64;
    type RxFifo1Message = rx::Message<64>;
    type RxFifo1 = U64;
    type TxMessage = tx::Message<64>;
    type TxBuffers = U32;
    type DedicatedTxBuffers = U0;
    type TxEventFifo = U32;
}

pub type RxFifo0 =
    RxFifo<'static, Fifo0, Can0, <Capacities as mcan::messageram::Capacities>::RxFifo0Message>;

pub type Can0Tx = mcan::tx_buffers::Tx<'static, Can0, Capacities>;

pub type Can0TxEventFifo = mcan::tx_event_fifo::TxEventFifo<'static, Can0>;

pub type Can0Aux<GclkId> = mcan::bus::Aux<
    'static,
    Can0,
    Dependencies<Can0, GclkId, CanRx, CanTx, pac::Can0>,
>;