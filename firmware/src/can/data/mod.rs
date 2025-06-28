pub mod custom_can;
pub mod egs_51;
pub mod egs_52;
pub mod egs_53;
pub mod hfm;
pub mod slave_mode;


pub trait SignalFrame {
	const CAN_ID: usize;
}