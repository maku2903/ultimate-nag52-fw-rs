use packed_struct::{derive::PackedStruct, PackedStruct};

use super::TleChannel;

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="1")]
pub struct MsgId {
    #[packed_field(bits="0..8")]
    pub msg_id: u8,
}

impl MsgId {
    pub fn new(msg_id: u8) -> Self {
        Self {
            msg_id
        }
    }
}

#[derive(PackedStruct, PartialEq, Eq)]
#[packed_struct(bit_numbering="lsb0", size_bytes="1")]
pub struct MsgIdWithChannel {
    #[packed_field(bits="0..3")]
    pub channel_id: u8,
    #[packed_field(bits="3..7")]
    pub msg_id: u8,
}

impl MsgIdWithChannel {
    pub fn new(msg_id: u8, channel: TleChannel) -> Self {
        Self {
            channel_id: channel as u8,
            msg_id
        }
    }
}

pub trait TleHeader: PackedStruct{}

impl TleHeader for MsgId{}
impl TleHeader for MsgIdWithChannel{}

pub trait TleMessage: PackedStruct {
    const MSG_ID: u8;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="4")]
pub struct TleMessagePayload {
    #[packed_field(bits="0..24")]
    pub payload: [u8; 3],
    #[packed_field(bits="24..31")]
    pub msg_id: u8,
    #[packed_field(bits="31")]
    pub r_w: bool
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct ICVersion {
    #[packed_field(bits="8..16")]
    pub version: u8,
    #[packed_field(bits="16..24")]
    pub ic_manf_id: u8,
}

impl TleMessage for ICVersion {
    const MSG_ID: u8 = 0;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct ControlMethodFaultMaskConfig {
    #[packed_field(bits="4..6")]
    pub diag_tmr: u8,
    #[packed_field(bits="6")]
    pub fme: bool,
    #[packed_field(bits="7")]
    pub fmr: bool,

    #[packed_field(bits="8")]
    pub fm7: bool,
    #[packed_field(bits="9")]
    pub fm6: bool,
    #[packed_field(bits="10")]
    pub fm5: bool,
    #[packed_field(bits="11")]
    pub fm4: bool,
    #[packed_field(bits="12")]
    pub fm3: bool,
    #[packed_field(bits="13")]
    pub fm2: bool,
    #[packed_field(bits="14")]
    pub fm1: bool,
    #[packed_field(bits="15")]
    pub fm0: bool,

    #[packed_field(bits="16")]
    pub cm7: bool,
    #[packed_field(bits="17")]
    pub cm6: bool,
    #[packed_field(bits="18")]
    pub cm5: bool,
    #[packed_field(bits="19")]
    pub cm4: bool,
    #[packed_field(bits="20")]
    pub cm3: bool,
    #[packed_field(bits="21")]
    pub cm2: bool,
    #[packed_field(bits="22")]
    pub cm1: bool,
    #[packed_field(bits="23")]
    pub cm0: bool,
}

impl TleMessage for ControlMethodFaultMaskConfig {
    const MSG_ID: u8 = 1;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct DiagConfiguration0_3 {
    #[packed_field(bits="0..4")]
    sb_retry3: u8,
    #[packed_field(bits="4..6")]
    sb3: u8,

    #[packed_field(bits="6..10")]
    sb_retry2: u8,
    #[packed_field(bits="10..12")]
    sb2: u8,

    #[packed_field(bits="12..16")]
    sb_retry1: u8,
    #[packed_field(bits="16..18")]
    sb1: u8,

    #[packed_field(bits="18..22")]
    sb_retry0: u8,
    #[packed_field(bits="22..24")]
    sb0: u8,
}

impl TleMessage for DiagConfiguration0_3 {
    const MSG_ID: u8 = 2;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct DiagConfiguration4_7 {
    #[packed_field(bits="0..4")]
    sb_retry7: u8,
    #[packed_field(bits="4..6")]
    sb7: u8,

    #[packed_field(bits="6..10")]
    sb_retry6: u8,
    #[packed_field(bits="10..12")]
    sb6: u8,

    #[packed_field(bits="12..16")]
    sb_retry5: u8,
    #[packed_field(bits="16..18")]
    sb5: u8,

    #[packed_field(bits="18..22")]
    sb_retry4: u8,
    #[packed_field(bits="22..24")]
    sb4: u8,
}

impl TleMessage for DiagConfiguration4_7 {
    const MSG_ID: u8 = 3;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct DiagnosticRead0_3 {
    #[packed_field(bits="0")]
    ol_on3: bool,
    #[packed_field(bits="1")]
    ol_off3: bool,
    #[packed_field(bits="2")]
    sb_tst3: bool,
    #[packed_field(bits="3")]
    sb3: bool,
    #[packed_field(bits="4")]
    off_tst3: bool,
    #[packed_field(bits="5")]
    sg3: bool,

    #[packed_field(bits="6")]
    ol_on2: bool,
    #[packed_field(bits="7")]
    ol_off2: bool,
    #[packed_field(bits="8")]
    sb_tst2: bool,
    #[packed_field(bits="9")]
    sb2: bool,
    #[packed_field(bits="10")]
    off_tst2: bool,
    #[packed_field(bits="11")]
    sg2: bool,

    #[packed_field(bits="12")]
    ol_on1: bool,
    #[packed_field(bits="13")]
    ol_off1: bool,
    #[packed_field(bits="14")]
    sb_tst1: bool,
    #[packed_field(bits="15")]
    sb1: bool,
    #[packed_field(bits="16")]
    off_tst1: bool,
    #[packed_field(bits="17")]
    sg1: bool,

    #[packed_field(bits="18")]
    ol_on0: bool,
    #[packed_field(bits="19")]
    ol_off0: bool,
    #[packed_field(bits="20")]
    sb_tst0: bool,
    #[packed_field(bits="21")]
    sb0: bool,
    #[packed_field(bits="22")]
    off_tst0: bool,
    #[packed_field(bits="23")]
    sg0: bool,
}

impl TleMessage for DiagnosticRead0_3 {
    const MSG_ID: u8 = 4;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct DiagnosticRead4_7 {
    #[packed_field(bits="0")]
    ol_on7: bool,
    #[packed_field(bits="1")]
    ol_off7: bool,
    #[packed_field(bits="2")]
    sb_tst7: bool,
    #[packed_field(bits="3")]
    sb7: bool,
    #[packed_field(bits="4")]
    off_tst7: bool,
    #[packed_field(bits="5")]
    sg7: bool,

    #[packed_field(bits="6")]
    ol_on6: bool,
    #[packed_field(bits="7")]
    ol_off6: bool,
    #[packed_field(bits="8")]
    sb_tst6: bool,
    #[packed_field(bits="9")]
    sb6: bool,
    #[packed_field(bits="10")]
    off_tst6: bool,
    #[packed_field(bits="11")]
    sg6: bool,

    #[packed_field(bits="12")]
    ol_on5: bool,
    #[packed_field(bits="13")]
    ol_off5: bool,
    #[packed_field(bits="14")]
    sb_tst5: bool,
    #[packed_field(bits="15")]
    sb5: bool,
    #[packed_field(bits="16")]
    off_tst5: bool,
    #[packed_field(bits="17")]
    sg5: bool,

    #[packed_field(bits="18")]
    ol_on4: bool,
    #[packed_field(bits="19")]
    ol_off4: bool,
    #[packed_field(bits="20")]
    sb_tst4: bool,
    #[packed_field(bits="21")]
    sb4: bool,
    #[packed_field(bits="22")]
    off_tst4: bool,
    #[packed_field(bits="23")]
    sg4: bool,
}

impl TleMessage for DiagnosticRead4_7 {
    const MSG_ID: u8 = 5;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct PwmOffset0_3 {
    #[packed_field(bits="0..5")]
    offset3: u8,
    #[packed_field(bits="5..10")]
    offset2: u8,
    #[packed_field(bits="10..15")]
    offset1: u8,
    #[packed_field(bits="15..20")]
    offset0: u8,
    
}

impl TleMessage for PwmOffset0_3 {
    const MSG_ID: u8 = 6;
}


#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct PwmOffset4_7 {
    #[packed_field(bits="0..5")]
    offset7: u8,
    #[packed_field(bits="5..10")]
    offset6: u8,
    #[packed_field(bits="10..15")]
    offset5: u8,
    #[packed_field(bits="15..20")]
    offset4: u8,
    
}

impl TleMessage for PwmOffset4_7 {
    const MSG_ID: u8 = 7;
}


#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct MainPeriodSet {
    #[packed_field(bits="0..14", endian="lsb")]
    divider_n: u16,
    #[packed_field(bits="14..16")]
    divider_m: u8,
    #[packed_field(bits="16")]
    sam: bool,
}

impl TleMessage for MainPeriodSet {
    const MSG_ID: u8 = 8;
}


#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct ControlVariableSet {
    #[packed_field(bits="0..12", endian="lsb")]
    ki: u16,
    #[packed_field(bits="12..24", endian="lsb")]
    kp: u16,
}

impl TleMessage for ControlVariableSet {
    const MSG_ID: u8 = 9;
}


#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct CurrentDitherAmplitudeSet {
    #[packed_field(bits="0..11", endian="lsb")]
    pub current_setpoint: u16,
    #[packed_field(bits="11..21", endian="lsb")]
    pub dither_step_size: u16,
    #[packed_field(bits="23")]
    pub en : bool
}

impl TleMessage for CurrentDitherAmplitudeSet {
    const MSG_ID: u8 = 0b0011;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct DitherPeriodSet {
    #[packed_field(bits="0..5")]
    number_of_steps: u8,
}

impl TleMessage for DitherPeriodSet {
    const MSG_ID: u8 = 11;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct MaxMinCurrentRead {
    #[packed_field(bits="0..11", endian="lsb")]
    min: u16,
    #[packed_field(bits="11..22", endian="lsb")]
    max: u16,
    #[packed_field(bits="22")]
    valid: bool,
}

impl TleMessage for MaxMinCurrentRead {
    const MSG_ID: u8 = 12;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct AvgCurrentRead {
    #[packed_field(bits="0..20", endian="lsb")]
    avg: u32,
    #[packed_field(bits="20")]
    valid: bool,
}

impl TleMessage for AvgCurrentRead {
    const MSG_ID: u8 = 13;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct AutoZeroTrigger {
    #[packed_field(bits="0..8")]
    az_off_val: u8,
    #[packed_field(bits="8..16")]
    az_on_val: u8,
    #[packed_field(bits="16")]
    az_off: bool,
    #[packed_field(bits="17")]
    an_on: bool,
}

impl TleMessage for AutoZeroTrigger {
    const MSG_ID: u8 = 14;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct PwmDutyCycle {
    #[packed_field(bits="0..19", endian="lsb")]
    pwm: u32,
}

impl TleMessage for PwmDutyCycle {
    const MSG_ID: u8 = 15;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct CurrentProfileSetup1 {
    #[packed_field(bits="0..4")]
    count1: u8,
    #[packed_field(bits="4..8")]
    count2: u8,
    #[packed_field(bits="8..12")]
    count3: u8,

    #[packed_field(bits="12..16")]
    threshold1: u8,
    #[packed_field(bits="16..20")]
    threshold2: u8,
    #[packed_field(bits="20..24")]
    threshold3: u8,
}

impl TleMessage for CurrentProfileSetup1 {
    const MSG_ID: u8 = 16;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct CurrentProfileSetup2 {
    #[packed_field(bits="0..2")]
    zone3_set: u8,
    #[packed_field(bits="4..8")]
    timeout: u8,
}

impl TleMessage for CurrentProfileSetup2 {
    const MSG_ID: u8 = 17;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct CurrentProfileFeedback {
    #[packed_field(bits="0")]
    pass: bool,
    #[packed_field(bits="1")]
    timeout: bool,
    #[packed_field(bits="2")]
    detect_interrupt: bool,
}

impl TleMessage for CurrentProfileFeedback {
    const MSG_ID: u8 = 18;
}

#[derive(PackedStruct, Default)]
#[packed_struct(bit_numbering="lsb0", size_bytes="3")]
pub struct ReadGenericFlags {
    #[packed_field(bits="0")]
    reset_latch_bit: bool,
    #[packed_field(bits="1")]
    enable_latch_bit: bool,
    #[packed_field(bits="2")]
    phase_sync_occurred: bool,
    #[packed_field(bits="3")]
    overvoltage_occurred: bool,
}

impl TleMessage for ReadGenericFlags {
    const MSG_ID: u8 = 19;
}