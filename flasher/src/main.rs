use std::{fs, path::PathBuf, str::FromStr};

use clap::{Parser, ValueEnum};
use probe_rs::probe::list::Lister;

#[derive(Debug, Clone, Copy)]
#[derive(ValueEnum)]
pub enum Protocol {
    Swd,
    Usb,
    Can
}

#[derive(clap::Parser)]
pub struct Flasher {
    #[arg(short, long)]
    pub protocol: Protocol,
    /// Binary file to flash to the TCU
    #[arg(short, long)]
    pub file: PathBuf
}

fn main() {
    let args = Flasher::parse();
    let binary_bytes = fs::read(args.file).unwrap();

}

fn with_probe_rs() {
    let lister = Lister::new();
    let probes = lister.list_all();
    probes.get(0)
}
