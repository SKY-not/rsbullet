use std::{thread::sleep, time::Duration};

use rsbullet::{BulletResult, Mode, PhysicsClient, UrdfOptions};

fn main() -> BulletResult<()> {
    let mut client = PhysicsClient::connect(Mode::Gui)?;
    client
        .set_default_search_path()?
        .set_real_time_simulation(true)?
        .set_gravity([0., 0., -10.])?;

    let plane = client.load_urdf("plane.urdf", None::<UrdfOptions>)?;
    let cube = client.load_urdf("cube.urdf", Some([0., 0., 3.]))?;

    client
        .set_collision_filter_group_mask(cube, 1, 0, 0)?
        .set_collision_filter_pair(plane, cube, -1, -1, true)?;

    while client.is_connected() {
        client.step_simulation()?;
        sleep(Duration::from_secs_f64(1. / 240.));
    }
    Ok(())
}
