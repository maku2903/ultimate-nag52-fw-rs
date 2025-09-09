#![cfg_attr(feature = "mcu", no_std)]

//pub mod ioctl;
//pub use automotive_diag::*;

#[cfg(feature = "mcu")]
pub mod isotp_endpoints;
