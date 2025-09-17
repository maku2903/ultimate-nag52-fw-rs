/// Userpage information

#[repr(C, packed(1))]
/// Structure held in user page of the TCU
pub struct TcuUserPage {
    pub manf_day: u8,
    pub manf_week: u8,
    pub manf_month: u8,
    pub manf_year: u8,
    pub can_baud: u32,
    pub _reserved: [u8; 504],
}

static_assertions::const_assert!(size_of::<TcuUserPage>() == 512);
