pub mod can_isotp;
pub mod usb_isotp;

#[derive(Copy, Clone)]
pub struct SharedIsoTpBuf<const N: usize> {
    pub (crate) size: usize,
    pub (crate) data: [u8; N],
}

impl<const N: usize> SharedIsoTpBuf<N> {
    fn new() -> Self {
        const {
            assert!(N <= 4096);
        }
        Self {
            size: 0,
            data: [0; N],
        }
    }

    pub fn payload(&self) -> &[u8] {
        &self.data[..self.size]
    }
}
