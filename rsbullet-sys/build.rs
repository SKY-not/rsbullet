use std::path::{Path, PathBuf};

fn lib_exists(dir: &Path, name: &str) -> bool {
    // Windows static: Foo.lib；Unix static: libFoo.a
    let win = dir.join(format!("{}.lib", name));
    let unix = dir.join(format!("lib{}.a", name));
    win.exists() || unix.exists()
}

fn main() {
    let mut cfg = cmake::Config::new("bullet3");
    cfg.profile("Release")
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_BULLET2_DEMOS", "OFF")
        .define("BUILD_CPU_DEMOS", "OFF")
        .define("BUILD_OPENGL3_DEMOS", "OFF")
        .define("BUILD_EXTRAS", "OFF")
        .define("BUILD_UNIT_TESTS", "OFF")
        .define("BUILD_PYBULLET", "OFF");

    // cfg.generator("Ninja");

    // cfg.define("CMAKE_MSVC_RUNTIME_LIBRARY", "MultiThreaded$<$<CONFIG:Debug>:Debug>");

    let dst = cfg.build();
    let libdir: PathBuf = Path::new(&format!("{}/lib", dst.display())).into();

    println!("cargo:rustc-link-search=native={}", libdir.display());

    for &lib in &[
        "BulletSoftBody",
        "BulletDynamics",
        "BulletCollision",
        "LinearMath",
        "BulletInverseDynamicsUtils",
        "BulletInverseDynamics",
        "Bullet3Common",
        "cbullet",
    ] {
        if lib_exists(&libdir, lib) {
            println!("cargo:rustc-link-lib=static={}", lib);
        }
    }

    #[cfg(all(not(target_env = "msvc"), not(target_os = "macos")))]
    println!("cargo:rustc-link-lib=dylib=stdc++");

    #[cfg(target_os = "macos")]
    println!("cargo:rustc-link-lib=c++");
}
