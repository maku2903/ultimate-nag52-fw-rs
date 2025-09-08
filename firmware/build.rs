use candb_codegen::codegen_all_dbs;
use chrono::Datelike;
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const MEM_BYTES: &[u8] = include_bytes!("memory.x");
const MEM_F_NAME: &'static str = "memory.x";

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(MEM_BYTES)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={}", MEM_F_NAME);
    println!("cargo:rerun-if-changed=build.rs");

    // Can data parsing
    println!("cargo::rerun-if-changed=can_data/custom_can.txt");
    println!("cargo::rerun-if-changed=can_data/egs51.txt");
    println!("cargo::rerun-if-changed=can_data/egs52.txt");
    println!("cargo::rerun-if-changed=can_data/egs53.txt");
    println!("cargo::rerun-if-changed=can_data/hfm.txt");
    println!("cargo::rerun-if-changed=can_data/slave_mode.txt");

    codegen_all_dbs(PathBuf::from("can_data/"), PathBuf::from("src/can/data/"));

    // Generate our build timestamps for bootloader info header
    let time = chrono::Utc::now();
    println!("cargo::rustc-env=BUILD_YEAR={}", time.year() - 2000);
    println!("cargo::rustc-env=BUILD_MONTH={}", time.month());
    println!("cargo::rustc-env=BUILD_WEEK={}", time.iso_week().week());
    println!("cargo::rustc-env=BUILD_DAY={}", time.day());

    // Grab rust version info
    let v = rustc_version::version().unwrap();
    println!("cargo::rustc-env=RUSTC_VER_MAJOR={}", v.major);
    println!("cargo::rustc-env=RUSTC_VER_MINOR={}", v.minor);
    println!("cargo::rustc-env=RUSTC_VER_PATCH={}", v.patch);
}
