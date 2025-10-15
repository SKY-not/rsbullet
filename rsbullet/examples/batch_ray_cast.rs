use std::f64::consts::PI;

use rsbullet::{
    BulletResult, DebugLineOptions, DebugVisualizerOptions, Mode, PhysicsClient,
    RayTestBatchOptions,
};

fn main() -> BulletResult<()> {
    let mut client = PhysicsClient::connect(Mode::Gui)?;
    client
        .set_default_search_path()?
        .configure_debug_visualizer(&DebugVisualizerOptions::Flag(
            rsbullet::DebugVisualizerFlag::CovEnableGui,
            false,
        ))?;

    client.load_urdf("r2d2.urdf", Some([3., 3., 1.]))?;

    let num_rays = 1024;
    let ray_len = 13.;
    let ray_hit_color = [1., 0., 0.];
    let ray_miss_color = [0., 1., 0.];
    let replace_lines = true;

    let mut ray_from: Vec<[f64; 3]> = Vec::with_capacity(num_rays);
    let mut ray_to: Vec<[f64; 3]> = Vec::with_capacity(num_rays);
    let mut ray_ids: Vec<i32> = Vec::with_capacity(num_rays);

    for i in 0..num_rays {
        ray_from.push([0., 0., 1.]);
        let ray = [
            ray_len * f64::sin(2. * PI * i as f64 / num_rays as f64),
            ray_len * f64::cos(2. * PI * i as f64 / num_rays as f64),
            1.,
        ];

        ray_to.push(ray.into());
        if replace_lines {
            ray_ids.push(
                client
                    .add_user_debug_line(&DebugLineOptions {
                        from: ray_from[i],
                        to: ray_to[i],
                        color: Some(ray_miss_color),
                        ..Default::default()
                    })
                    .unwrap(),
            );
        }
    }
    let num_steps = 327_680;
    for _ in 0..num_steps {
        client.step_simulation()?;
        let results = client.ray_test_batch(&RayTestBatchOptions {
            ray_from_positions: &ray_from,
            ray_to_positions: &ray_to,
            num_threads: Some(4),
            ..Default::default()
        })?;
        if !replace_lines {
            client.remove_all_user_debug_items()?;
        }
        for (i, hit) in results.iter().enumerate() {
            let (to, color) = if hit.hit_fraction < 1.0 {
                (hit.hit_position_world, ray_hit_color)
            } else {
                (ray_to[i], ray_miss_color)
            };

            client.add_user_debug_line(&DebugLineOptions {
                from: ray_from[i],
                to,
                color: Some(color),
                replace_item_unique_id: if replace_lines {
                    Some(ray_ids[i])
                } else {
                    None
                },
                ..Default::default()
            })?;
        }
    }

    Ok(())
}
