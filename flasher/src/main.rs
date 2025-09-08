use std::{collections::BTreeMap, fs, path::PathBuf, thread::sleep, time::{Duration, Instant}};

use chrono::{Datelike, Utc};
use clap::{builder::styling::Color, *};
use clap_num::maybe_hex;
use console::style;
use ecu_diagnostics::{channel::{IsoTPChannel, IsoTPSettings}, dynamic_diag::{DiagServerBasicOptions, DiagServerEmptyLogger, DynamicDiagSession, TimeoutConfig}, hardware::{Hardware, HardwareScanner}, kwp2000::{Kwp2000Protocol, KwpCommand, KwpError, KwpSessionType}, DiagError};
use elf::{abi::PT_LOAD, endian::{self, LittleEndian}};
use indicatif::{HumanBytes, HumanDuration, MultiProgress, ProgressBar, ProgressStyle};
use color_eyre::{eyre::{Report, Result}, owo_colors::OwoColorize};
use object::{elf::FileHeader32, read::elf::{FileHeader, ProgramHeader}, Endianness};
use serialport::SerialPortType;

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
    /// Analyze firmware binary for SRAM/Flash usage
    Analyze { file: PathBuf },
    /// Read out ECU identification
    Ident,
    /// Burn production date into the ECU
    /// THIS CAN ONLY BE PERFORMED ONCE!
    BurnDate,
    /// Set chip security bits
    SetSecurity {
        #[clap(long, short, action)]
        enable: bool 
    },
    /// Flash an application binary to the TCU
    Flash {
        #[clap(long, short)]
        bootloader: Option<PathBuf>,
        #[clap(long, short)]
        application: PathBuf,
    },
    /// Read / Dump memory from the TCU to a binary file
    Read {
        #[clap(value_parser=maybe_hex::<u32>)]
        start_address: u32,
        #[clap(value_parser=maybe_hex::<u32>)]
        end_address: u32,
        output_file: PathBuf
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ElfSegment {
    phys_addr: u32,
    virt_addr: u32,
    size: u32,
    offset_in_elf: u32
}

#[derive(clap::Parser, Clone)]
pub struct Flasher {
    #[command(subcommand)]
    pub command: Command,
    #[cfg(not(target_os="windows"))]
    #[clap(value_enum)]
    pub interface: Interface,
    #[cfg(not(target_os="windows"))]
    #[arg(required_if_eq_any([
        ("interface","Can"),
        ("interface","can"),
        ("interface","Can-fast"),
        ("interface","can-fast")
    ]))]
    pub can_iface: Option<String>
}

pub const EGS_DIAG_SETTINGS: DiagServerBasicOptions = DiagServerBasicOptions {
    send_id: 0x07E1,
    recv_id: 0x07E9,
    timeout_cfg: TimeoutConfig {
        read_timeout_ms: 5000,
        write_timeout_ms: 5000,
    },
};

fn launch_server_usb() -> Result<DynamicDiagSession, Report> {
    println!("Waiting for device to be available...");
    let port: String;
    'outer: loop {
        let ports = serialport::available_ports()?;
        for p in ports {
            if let SerialPortType::UsbPort(usb_inf) = p.port_type {
                if usb_inf.vid == 0x16c0 && usb_inf.pid == 0x27de {
                    port = p.port_name;
                    break 'outer;
                }
            }
        }
        std::thread::sleep(Duration::from_millis(10));

    }

    let serial = UsbDiagIface::new(&port)?;

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

fn create_server(fast_mode: &mut bool, args: &Flasher) -> Result<DynamicDiagSession, Report> {
    #[cfg(target_os="linux")]
    let server = match args.interface {
        Interface::Usb => launch_server_usb(),
        Interface::Can => {
            *fast_mode = false;
            launch_server_isotp(&args.can_iface.clone().unwrap(), false)
        },
        Interface::CanFast => {
            *fast_mode = true;
            launch_server_isotp(&args.can_iface.clone().unwrap(), true)
        }
    }?;
    #[cfg(not(target_os="linux"))]
    let server = launch_server_usb()?;
    Ok(server)
}

#[cfg(target_os="linux")]
fn launch_server_isotp(can_iface_name: &str, fast: bool) -> Result<DynamicDiagSession, Report> {
    use ecu_diagnostics::dynamic_diag::DiagServerAdvancedOptions;

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
    let adv_opts = DiagServerAdvancedOptions  {
        global_tp_id: 0,
        tester_present_interval_ms: 2000,
        tester_present_require_response: true,
        global_session_control: false,
        tp_ext_id: None,
        command_cooldown_ms: 0,
    };

    let server = DynamicDiagSession::new_over_iso_tp(
        Kwp2000Protocol::default(), 
        channel, 
        egs_isotp_opts, 
        EGS_DIAG_SETTINGS, 
        Some(adv_opts), 
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

fn read(mp: &MultiProgress, dest_file: &PathBuf, start_addr: u32, end_addr: u32, server: DynamicDiagSession, fast_mode: bool) -> Result<(), Report> {
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
        let max = std::cmp::min(250, total_bytes - v.len() as u32);
        pb.set_message(format!("0x{:08X}-0x{:08X}", start_addr + v.len()as u32, start_addr + v.len() as u32 + max));
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

fn analyze(file: PathBuf)  -> Result<(), Report> {
    println!("Analyzing {file:?}");
    /*
    let binary_bytes = fs::read(file)?;
    let elf = object::File::parse(&*binary_bytes)?;
    let mut flash_bytes: usize = 0;
    let mut ram_bytes: usize = 0;
    for section in elf.sections() {
        if section.kind() == SectionKind::Other || section.kind() == SectionKind::Metadata || section.kind() == SectionKind::OtherString {
            continue;
        }
        if section.address() < 0x100000 {
            // Flash
            flash_bytes += section.size() as usize;
        } else if section.address() >= 0x20000000 && section.address() <= 0x2003FFFF {
            // RAM
            ram_bytes += section.size() as usize;
        }
    }
    let pb_flash = ProgressBar::new(1024*1024)
        .with_style(ProgressStyle::with_template("Flash usage: [{bar:40.cyan/blue}] {percent}% ({decimal_bytes}/{total_bytes})")
        .unwrap()
        .progress_chars("##-"));
    pb_flash.set_position(flash_bytes as u64);
    pb_flash.abandon();
    let pb_ram = ProgressBar::new(256*1024)
        .with_style(ProgressStyle::with_template("  RAM usage: [{bar:40.cyan/blue}] {percent}% ({decimal_bytes}/{total_bytes})")
        .unwrap()
        .progress_chars("##-"));
    pb_ram.set_position(ram_bytes as u64);
    pb_ram.abandon();
    */
    Ok(())
}

fn flash(mp: &MultiProgress, file: &PathBuf, server: &mut DynamicDiagSession, fast_mode: bool, is_bootloader: bool) -> Result<(), Report> {
    let binary_bytes = fs::read(file)?;
    let binary = FileHeader32::<Endianness>::parse(&*binary_bytes)?;
    let endian = binary.endian()?;
    
    let mut segments = Vec::new();
    
    for segment in binary.program_headers(binary.endian()?, &*binary_bytes)? {
        let p_paddr: u64 = segment.p_paddr(endian).into();
        let p_vaddr: u64 = segment.p_vaddr(endian).into();
        let flags = segment.p_flags(endian);
        let segment_data = segment.data(endian, &*binary_bytes).map_err(|_| Report::msg("Failed to access data for ELF segment"))?;
        if !segment_data.is_empty() { 
            if segment.p_type(endian) == PT_LOAD {
                let (segment_offset, segment_filesize) = segment.file_range(endian);
                segments.push(ElfSegment {
                    phys_addr: p_paddr as u32,
                    virt_addr: p_vaddr as u32,
                    size: segment_filesize as u32,
                    offset_in_elf: segment_offset as u32,
                });
            } else {
                println!("{segment:?}");
            }
        }
    }

    segments.sort_by(|x, y| x.phys_addr.cmp(&y.phys_addr));
    let start_address = segments[0].phys_addr;
    let last = segments.last().unwrap();
    let end_addr = last.phys_addr + last.size;
    let to_flash = end_addr-start_address;
    println!("{} bytes ({:08X} - {:08X}) {}", end_addr-start_address, start_address, end_addr, to_flash);
    assert!(start_address % 8192 == 0);
    let mut array = vec![0xFFu8; to_flash as usize];
    for seg in segments {
        let offset = seg.phys_addr as usize - start_address as usize;
        array[offset..offset + seg.size as usize].copy_from_slice(&binary_bytes[seg.offset_in_elf as usize..seg.size as usize + seg.offset_in_elf as usize]);
    }
    let mut num_pages = array.len() / 8192;
    if array.len() % 8192 != 0 {
        num_pages += 1;
    }
    let spinner = next_spinner(&mp, None, 1, 6);
    spinner.set_message("Enter programming mode");
    // Now start the command chain
    if fast_mode {
        server.send_byte_array_with_response(&[KwpCommand::StartDiagnosticSession.into(), KwpSessionType::Reprogramming.into(), 0, 0])?;
    } else {
        server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    }
    let spinner = next_spinner(&mp, Some(spinner), 2, 6);
    spinner.set_message(format!("Erasing flash ({} from 0x{:08X})", HumanBytes((num_pages*8192) as u64), start_address));

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

fn ident(server: DynamicDiagSession) -> Result<(), Report> {
    let mut map: BTreeMap<&'static str, Option<String>> = BTreeMap::new();
    if let Ok(ident) = server.kwp_read_daimler_identification() {
        map.insert("ECU Production date", Some(format!("{}/{}/20{}", ident.ecu_production_day, ident.ecu_production_month, ident.ecu_production_year)));
        map.insert("ECU Software date", Some(format!("Week {} of 20{}", ident.ecu_sw_build_week, ident.ecu_sw_build_year)));
    } else {
        map.insert("ECU Production date", None);
        map.insert("ECU Software date", None);
    }

    if let Ok(ident) = server.kwp_read_ecu_serial_number() {
        let mut res = String::new();
        for b in ident {
            res.push_str(&format!("{b:02X?}"));
        }
        map.insert("ECU Serial Number", Some(res));
    } else {
        map.insert("ECU Serial Number", None);
    }

    let ident = server.kwp_read_daimler_mmc_identification()?;
    map.insert("HW Version", Some(format!("{}", ident.hw_version)));
    map.insert("SW Version", Some(format!("{}", ident.sw_version)));

    let s = if ident.diag_info.is_boot_sw() {
        style("Bootloader").bold().red()
    } else {
        style("Application")
    };
    map.insert("Software type", Some(format!("{s}")));

    let dbg = if ident.diag_info.is_production_ecu() {
        style("No")
    } else {
        style("Yes").bold().red()
    };

    map.insert("Debug mode SW", Some(format!("{dbg}")));

    println!("{}", style("Identification information").bold().bright_blue());
    for (k, v) in map {
        println!("{: <20}: {}",
        style(k).bold(),
        v.map(|x| style(x).green()).unwrap_or(style("REFUSED".into()).bold().red())
        );
    }
    Ok(())
}

fn burn_date(server: DynamicDiagSession) -> Result<(), Report> {
    server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    let date = Utc::now();
    let mut req = [KwpCommand::StartRoutineByLocalIdentifier as u8, 0x24, 0,0,0,0];
    req[2] = date.day() as u8; // Day
    req[3] = date.iso_week().week() as u8; // Week
    req[4] = date.month() as u8; // Month
    req[5] = (date.year() % 100) as u8; // Year
    server.send_byte_array_with_response(&req)?;
    println!("Burnt production date: {}/{}/{} (Week {})", req[2], req[4], req[5],  req[3]);
    Ok(())
}

fn set_security_lock(server: DynamicDiagSession, en: bool) -> Result<(), Report> {
    server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    server.send_byte_array_with_response(&[KwpCommand::StartRoutineByLocalIdentifier.into(), 0xFE, en as u8])?;
    Ok(())
}

fn main() -> Result<()> {
    env_logger::init();
    color_eyre::install()?;
    let start_timer = Instant::now();
    let args = Flasher::parse();

    if let Command::Analyze { file } = &args.command {
        analyze(file.clone())?;
        return Ok(());
    }

    let mut fast_mode = false;
    let mut server = create_server(&mut fast_mode, &args)?;

    let mut mp = MultiProgress::new();
    let res = match &args.command {
        Command::Flash { bootloader, application } => {
            let has_bootloader = bootloader.is_some();
            if let Some(loader) = bootloader {
                println!("{}", 
                    style("Flashing bootloader (Stage 1/2)").bold().green()
                );
                flash(&mp, loader, &mut server, fast_mode, true)?;
            }
            drop(mp);
            if has_bootloader {
                println!("{}", 
                    style("Flashing application (Stage 2/2)").bold().green()
                );
                drop(server);
                std::thread::sleep(Duration::from_millis(250));
                server = create_server(&mut fast_mode, &args)?;
                std::thread::sleep(Duration::from_millis(250));
            } else {
                println!("{}", 
                    style("Flashing application").bold().green()
                );
            }
            mp = MultiProgress::new();
            flash(&mp, application, &mut server, fast_mode, false)
        },
        Command::Read { start_address, end_address, output_file } => {
                read(&mp, output_file, *start_address, *end_address, server, fast_mode)
            },
        Command::Ident => {
                ident(server)
            }
        Command::BurnDate => {
                burn_date(server)
            }
        Command::SetSecurity { enable } => {
            set_security_lock(server, *enable)
        },
        Command::Analyze { .. } => { unreachable!() }
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

