pub struct SolenoidInfo {
    pub inductance_mh: u8,
    pub resistance_ohm: f32,
}


const SOL_INFO_SHIFT_VALVES: SolenoidInfo = SolenoidInfo {
    inductance_mh: 20,
    resistance_ohm: 3.9
};

const SOL_INFO_P_VALVES: SolenoidInfo = SolenoidInfo {
    inductance_mh: 28,
    resistance_ohm: 5.0
};

const SOL_INFO_TCC_VALVE: SolenoidInfo = SolenoidInfo {
    inductance_mh: 5,
    resistance_ohm: 2.5
};