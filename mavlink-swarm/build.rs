//! Generates Rust bindings for dialect/swarm.xml at build time. The included MAVLink XMLs
//! (common.xml, standard.xml, minimal.xml) are vendored next to it, copied from rust-mavlink
//! 0.19.1, because bindgen resolves `<include>` relative to the dialect file's directory.

use std::env;
use std::path::Path;

use mavlink_bindgen::XmlDefinitions;

fn main() {
    let dialect_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("dialect");
    let out_dir = env::var("OUT_DIR").unwrap();

    let result = mavlink_bindgen::generate(XmlDefinitions::Files(vec![dialect_dir.join("swarm.xml")]), out_dir)
        .expect("failed to generate MAVLink bindings for dialect/swarm.xml");

    mavlink_bindgen::emit_cargo_build_messages(&result);
    // emit_cargo_build_messages only watches swarm.xml itself; the included files matter too.
    println!("cargo:rerun-if-changed={}", dialect_dir.display());
}
