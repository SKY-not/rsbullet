use std::{thread::sleep, time::Duration};

use rsbullet::{Mode, PhysicsClient};

fn main() {
    let mut client = PhysicsClient::connect(Mode::Gui).expect("failed to start Bullet GUI");

    client
        .set_additional_search_path(&PhysicsClient::bullet_data_path())
        .expect("failed to register Bullet data path");

    client
        .load_urdf("r2d2.urdf", None)
        .expect("failed to load URDF");

    sleep(Duration::from_secs(10));
}
