//! Generates the optional compile-time image expression.

use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=UHD_EMBEDDED_IMAGE");
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    let source = env::var_os("UHD_EMBEDDED_IMAGE").map(PathBuf::from);
    let generated = match source {
        Some(path) => {
            println!("cargo:rerun-if-changed={}", path.display());
            format!("Some(include_bytes!({:?}) as &'static [u8])", path)
        }
        None => "None".to_owned(),
    };
    fs::write(out.join("embedded.rs"), generated).expect("write embedded image expression");
}
