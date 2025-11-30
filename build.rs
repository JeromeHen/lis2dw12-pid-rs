fn main() {
    // Only run build script if building for host (e.g., x86_64)
    let target = std::env::var("TARGET").unwrap();
    if target.contains("thumb") {
        // Skip build script for embedded targets
        return;
    }

    // Build the original C driver into a static lib so it can be used by tests
    cc::Build::new()
        .file("lis2dw12-pid/lis2dw12_reg.c")
        .include("lis2dw12-pid")
        .compile("lis2dw12_reg");
}
