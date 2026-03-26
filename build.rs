fn main() {
    // Tell Cargo to re-run this script if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");

    // Place memory.x on the linker search path
    println!("cargo:rustc-link-search={}", std::env::var("OUT_DIR").unwrap());
    std::fs::copy("memory.x", format!("{}/memory.x", std::env::var("OUT_DIR").unwrap())).unwrap();
}
