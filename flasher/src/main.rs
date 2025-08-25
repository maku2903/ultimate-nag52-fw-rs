use std::{fs, path::PathBuf, thread::sleep, time::{Duration, Instant}};

use clap::*;
use clap_num::maybe_hex;
use console::style;
use ecu_diagnostics::{channel::{IsoTPChannel, IsoTPSettings}, dynamic_diag::{DiagServerBasicOptions, DiagServerEmptyLogger, DynamicDiagSession, TimeoutConfig}, hardware::{Hardware, HardwareScanner}, kwp2000::{Kwp2000Protocol, KwpCommand, KwpError, KwpSessionType}, DiagError};
use indicatif::{HumanBytes, HumanDuration, MultiProgress, ProgressBar, ProgressStyle};
use object::{Object, ObjectSection};
use color_eyre::eyre::{Report, Result};

use crate::usb_diag_compat::UsbDiagIface;

mod usb_diag_compat;

#[cfg(target_os="linux")]
use ecu_diagnostics::hardware::socketcan::SocketCanScanner;

#[derive(Debug, Clone, ValueEnum)]
pub enum Interface {
    Usb,
    Can,
    CanFast,
}

#[derive(Subcommand, Clone)]
pub enum Command {
    /// Flash an application binary to the TCU
    Flash { file: PathBuf },
    /// Read / Dump memory from the TCU to a binary file
    Read {
        #[clap(value_parser=maybe_hex::<u32>)]
        start_address: u32,
        #[clap(value_parser=maybe_hex::<u32>)]
        end_address: u32,
        output_file: PathBuf
    }
}

#[derive(clap::Parser, Clone)]
pub struct Flasher {
    #[command(subcommand)]
    pub command: Command,
    #[cfg(not(target_os="windows"))]
    #[clap(value_enum)]
    pub interface: Interface,
    /// Interface name. If using socketcan (can/can-fast) then it would
    /// be a socketcan interface name, otherwise its the USB/COM port name
    pub iface_name: String,
}

pub const EGS_DIAG_SETTINGS: DiagServerBasicOptions = DiagServerBasicOptions {
    send_id: 0x07E1,
    recv_id: 0x07E9,
    timeout_cfg: TimeoutConfig {
        read_timeout_ms: 2000,
        write_timeout_ms: 2000,
    },
};

fn launch_server_usb(port: &str) -> Result<DynamicDiagSession, Report> {
    let serial = UsbDiagIface::new(port)?;

    let channel = Box::new(serial) as Box<dyn IsoTPChannel>;

    let egs_isotp_opts: IsoTPSettings = IsoTPSettings {
        block_size: 0,
        st_min: 0,
        extended_addresses: None,
        pad_frame: true,
        can_speed: 500_000,
        can_use_ext_addr: false,
    };

    let server = DynamicDiagSession::new_over_iso_tp(
        Kwp2000Protocol::default(), 
        channel, 
        egs_isotp_opts, 
        EGS_DIAG_SETTINGS, 
        None, 
        DiagServerEmptyLogger{}
    )?;
    
    Ok(server)
}

#[cfg(target_os="linux")]
fn launch_server_isotp(can_iface_name: &str, fast: bool) -> Result<DynamicDiagSession, Report> {
    let mut socket_can = SocketCanScanner::new().open_device_by_name(can_iface_name)?;
    let channel = socket_can.create_iso_tp_channel()?;

    let egs_isotp_opts: IsoTPSettings = IsoTPSettings {
        block_size: if fast {0} else {0x20},
        st_min: if fast {0} else {10},
        extended_addresses: None,
        pad_frame: true,
        can_speed: 500_000,
        can_use_ext_addr: false,
    };

    let server = DynamicDiagSession::new_over_iso_tp(
        Kwp2000Protocol::default(), 
        channel, 
        egs_isotp_opts, 
        EGS_DIAG_SETTINGS, 
        None, 
        DiagServerEmptyLogger{}
    )?;
    
    Ok(server)
}

fn next_spinner(mp: &MultiProgress, last_bar: Option<ProgressBar>, stage: u32, out_of: u32) -> ProgressBar {
    if let Some(last_bar) = last_bar {
        let old_msg = last_bar.message();
        last_bar.finish_with_message(format!("{old_msg} {}", style("✔").green()));
    }

    let next_bar = mp.add(ProgressBar::new_spinner());
    let spinner_style = ProgressStyle::with_template("{prefix:.bold.dim} {spinner} {wide_msg}")
        .unwrap()
        .tick_chars("⠁⠂⠄⡀⢀⠠⠐⠈ ");
    next_bar.set_style(spinner_style);
    next_bar.set_prefix(format!("[{stage}/{out_of}]"));
    next_bar.enable_steady_tick(Duration::from_millis(100));
    next_bar
}

fn read(mp: &MultiProgress, dest_file: PathBuf, start_addr: u32, end_addr: u32, server: DynamicDiagSession, fast_mode: bool) -> Result<(), Report> {
    let spinner = next_spinner(&mp, None, 1, 3);
    spinner.set_message("Enter programming mode");
    // Now start the command chain
    if fast_mode {
        server.send_byte_array_with_response(&[KwpCommand::StartDiagnosticSession.into(), KwpSessionType::Reprogramming.into(), 0, 0])?;
    } else {
        server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    }
    let spinner = next_spinner(&mp, Some(spinner), 2, 3);
    spinner.set_message(format!("Reading memory ({})", HumanBytes((end_addr-start_addr) as u64)));
    let mut v: Vec<u8> = Vec::new();
    let total_bytes = end_addr - start_addr;

    let pb = mp.add(ProgressBar::new(total_bytes as u64).with_message("Reading"))
        .with_style(ProgressStyle::with_template("{percent}% [{bar:40.cyan/blue}] {msg} {decimal_bytes_per_sec} ETA: {eta}")
        .unwrap()
        .progress_chars("##-")
    );

    while (v.len() as u32) < total_bytes {
        let max = std::cmp::min(255, total_bytes - v.len() as u32);
        let mut req = vec![0x23, max as u8];
        req.extend_from_slice(&(start_addr + v.len() as u32).to_le_bytes());
        let resp = server.send_byte_array_with_response(&req)?;
        v.extend_from_slice(&resp[1..]);
        pb.set_position(v.len() as u64);
    }
    pb.finish_with_message(format!("{}", style("✔").green()));
    let spinner = next_spinner(&mp, Some(spinner), 3, 3);
    spinner.set_message("Writing memory to file");
    std::fs::write(dest_file, v)?;
    spinner.finish_with_message(format!("{} {}", spinner.message(), style("✔").green()));
    Ok(())
}

fn flash(mp: &MultiProgress, file: PathBuf, server: DynamicDiagSession, fast_mode: bool) -> Result<(), Report> {
    let binary_bytes = fs::read(file).unwrap();
    let elf = object::File::parse(&*binary_bytes).unwrap();
    let vec_table = elf.section_by_name(".vector_table").unwrap();
    let txt = elf.section_by_name(".text").unwrap();
    let rodata = elf.section_by_name(".rodata").unwrap();
    let end_addr = rodata.address();

    let x = rodata.size();

    let len = end_addr+x-vec_table.address();
    
    let start_address = vec_table.address();

    let mut array = vec![0u8; len as usize];
    array[0..vec_table.size() as usize].copy_from_slice(&vec_table.data().unwrap());

    let offset = (txt.address() - vec_table.address()) as usize;
    array[offset..offset+txt.size() as usize].copy_from_slice(&txt.data().unwrap());
    

    let offset = (rodata.address() - vec_table.address()) as usize;
    array[offset..offset+rodata.size() as usize].copy_from_slice(&rodata.data().unwrap());
    // Resize to the next page offset (512 bytes)
    let new_size = (array.len() + 511) & !511;
    array.resize(new_size, 0);

    let mut num_pages = array.len() / 8192;
    if array.len() % 8192 != 0 {
        num_pages += 1;
    }
    assert!(start_address % 8192 == 0);
    let spinner = next_spinner(&mp, None, 1, 6);
    spinner.set_message("Enter programming mode");
    // Now start the command chain
    if fast_mode {
        server.send_byte_array_with_response(&[KwpCommand::StartDiagnosticSession.into(), KwpSessionType::Reprogramming.into(), 0, 0])?;
    } else {
        server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    }
    let spinner = next_spinner(&mp, Some(spinner), 2, 6);
    spinner.set_message(format!("Erasing flash ({})", HumanBytes((num_pages*8192) as u64)));

    let mut erase_cmd = [0; 8];
    erase_cmd[0] = 0x31;
    erase_cmd[1] = 0xE0;
    erase_cmd[2..6].copy_from_slice(&(start_address as u32).to_le_bytes());
    erase_cmd[6..8].copy_from_slice(&(num_pages as u16).to_le_bytes());
    server.send_byte_array_with_response(&erase_cmd)?;
    loop {
        match server.send_byte_array_with_response(&[KwpCommand::RequestRoutineResultsByLocalIdentifier.into(), 0xE0]) {
            Ok(res) => {
                if res[2] == 0x00 {
                    break;
                } else {
                    return Err(Report::msg("Flash erase failed"))
                }
            },
            Err(DiagError::ECUError { code, def }) => {
                if code == KwpError::RoutineNotComplete as u8 {
                    // Waiting
                    sleep(Duration::from_millis(1000));
                } else {
                    return Err(DiagError::ECUError { code, def }.into())
                }
            },
            Err(e) => return Err(e.into())
        }
    }
    // Flash erase completed
    let spinner = next_spinner(&mp, Some(spinner), 3, 6);
    spinner.set_message("Preparing download");
    let mut download_req = vec![KwpCommand::RequestDownload.into()];
    download_req.extend_from_slice(&(start_address as u32).to_le_bytes());
    download_req.push(0x00); // Fmt
    download_req.extend_from_slice(&(array.len() as u32).to_le_bytes());
    server.send_byte_array_with_response(&download_req)?;
    let mut counter: u8 = 0;
    const MAX_COPY: usize = 1024;
    let mut block = [0; MAX_COPY+2];
    let mut addr = 0;

    let spinner = next_spinner(&mp, Some(spinner), 4, 6);
    spinner.set_message(format!("Transfering data  ({})", HumanBytes(array.len() as u64)));
    let pb = mp.add(ProgressBar::new(array.len() as u64).with_message("Flashing"))
        .with_style(ProgressStyle::with_template("{percent}% [{bar:40.cyan/blue}] {msg} {decimal_bytes_per_sec} ETA: {eta}")
        .unwrap()
        .progress_chars("##-")
    );
    while addr < array.len() {
        let max_copy = core::cmp::min(MAX_COPY, array.len()-addr);
        // Start uploading data (512 byte chunks)!
        block[0] = KwpCommand::TransferData.into();
        block[1] = counter;
        block[2..2+max_copy].copy_from_slice(&array[addr..addr+max_copy]);
        pb.set_position(addr as u64);
        server.send_byte_array_with_response(&block)?;
        addr += max_copy;
        counter = counter.wrapping_add(1);
    }
    pb.finish_with_message(format!("{}", style("✔").green()));
    let spinner = next_spinner(&mp, Some(spinner), 5, 6);
    spinner.set_message("Verifying flashed data");
    // Start flash check routine
    let targ_crc = embedded_crc32c::crc32c(&array);
    let mut buf = vec![0x31, 0xE1];
    let start = start_address as u32;
    let end = start_address as u32 + array.len() as u32;

    buf.extend_from_slice(&targ_crc.to_le_bytes());
    buf.extend_from_slice(&start.to_le_bytes());
    buf.extend_from_slice(&end.to_le_bytes());
    let response = server.send_byte_array_with_response(&buf)?;
    if response[2] == 0x00 {
        return Err(Report::msg("Flash CRC compare failed"));
    }

    // Reset ECU
    let spinner = next_spinner(&mp, Some(spinner), 6, 6);
    spinner.set_message("Resetting ECU");
    server.send_byte_array_with_response(&[KwpCommand::ECUReset.into(), 0x01])?;
    spinner.finish_with_message(format!("{} {}", spinner.message(), style("✔").green()));
    Ok(())
}


fn main() -> Result<()> {
    env_logger::init();
    color_eyre::install()?;
    let start_timer = Instant::now();
    let args = Flasher::parse();
    let mut fast_mode = false;
    #[cfg(target_os="linux")]
    let server = match args.interface {
        Interface::Usb => launch_server_usb(&args.iface_name),
        Interface::Can => {
            fast_mode = false;
            launch_server_isotp(&args.iface_name, false)
        },
        Interface::CanFast => {
            fast_mode = true;
            launch_server_isotp(&args.iface_name, true)
        }
    }?;
    #[cfg(not(target_os="linux"))]
    let server = launch_server_usb(&args.iface_name)?;

    let mp = MultiProgress::new();
    let res = match args.command {
        Command::Flash { file } => {
            flash(&mp, file, server, fast_mode)
        },
        Command::Read { start_address, end_address, output_file } => {
            read(&mp, output_file, start_address, end_address, server, fast_mode)
        },
    };
    if res.is_err() {
        mp.clear()?;
    }
    res?;

    println!("{}", 
        style(format!("Completed in {}", HumanDuration(start_timer.elapsed()))).bold().green()
    );
    Ok(())
}

