use rsbullet::{BulletResult, Mode, PhysicsClient};

fn main() -> BulletResult<()> {
    let mut client = PhysicsClient::connect(Mode::Gui)?;
    client
        .set_default_search_path()?
        .set_gravity([0., 0., -10.])?;

    client.load_urdf("plane.urdf", None::<()>)?;
    client.load_urdf("cube.urdf", Some([0., 0., 1.]))?;

    loop {
        client.step_simulation()?;
        let pts = client.get_contact_points(None, None, None, None)?;
        let mut total_normal_force = 0.0;
        let mut total_lateral_friction_forces = [0.; 3];
        for pt in pts {
            total_normal_force += pt.normal_force;
            total_lateral_friction_forces[0] +=
                pt.linear_friction_force_1 + pt.linear_friction_force_2;
            total_lateral_friction_forces[1] +=
                pt.linear_friction_force_1 + pt.linear_friction_force_2;
            total_lateral_friction_forces[2] +=
                pt.linear_friction_force_1 + pt.linear_friction_force_2;
        }

        println!("total_normal_force = {}", total_normal_force);
        println!(
            "total_lateral_friction_forces = {:?}",
            total_lateral_friction_forces
        );
    }
}
