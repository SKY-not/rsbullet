use std::{path::PathBuf, thread::sleep, time::Duration};

use rsbullet::{Mode, PhysicsClient};

fn main() {
    let mut client = PhysicsClient::connect(Mode::Gui).expect("failed to start Bullet GUI");

    let data_path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "..",
        "rsbullet-sys",
        "bullet3",
        "data",
    ]
    .iter()
    .collect();
    client
        .set_additional_search_path(&data_path)
        .expect("failed to register Bullet data path");

    client
        .load_urdf("r2d2.urdf", None)
        .expect("failed to load URDF");

    sleep(Duration::from_secs(10));
}
