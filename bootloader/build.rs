use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use chrono::Datelike;

const MEM_BYTES: &[u8] = include_bytes!("memory.x");
const MEM_F_NAME: &str = "memory.x";

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(MEM_BYTES)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={}", MEM_F_NAME);
    println!("cargo:rerun-if-changed=build.rs");

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
