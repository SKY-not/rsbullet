use std::{f64::consts::PI, thread::sleep, time::Duration};

use nalgebra as na;
use rsbullet::*;

fn main() -> BulletResult<()> {
    let mut client = PhysicsClient::connect(Mode::Gui)?;
    client
        .set_default_search_path()?
        .set_gravity([0., 0., -10.])?
        .set_real_time_simulation(true)?;

    client.load_urdf("plane.urdf", None::<()>)?;
    let cube = client.load_urdf("cube.urdf", Some([0., 0., 1.]))?;

    let cid = client.create_constraint(&ConstraintCreateOptions {
        parent_body: cube,
        joint_type: JointType::Fixed,
        joint_axis: [0.; 3],
        child_frame: na::Isometry3::translation(0., 0., 1.),
        ..Default::default()
    })?;
    println!("Constraint ID: {cid}");
    println!("Constraint Info: {:?}", client.get_constraint_unique_id(0));

    let mut a = PI;
    loop {
        a += 0.01;
        if a > PI {
            a -= PI;
        }
        sleep(Duration::from_secs_f64(0.01));
        let translation = [a, 0., 1.].into();
        let rotation = na::UnitQuaternion::from_euler_angles(a, 0., 0.);
        client.set_gravity([0., 0., -10.])?.change_constraint(
            cid,
            &ConstraintUpdate {
                child_frame: Some(na::Isometry3::from_parts(translation, rotation)),
                max_force: Some(50.),
                ..Default::default()
            },
        )?;
    }
}
