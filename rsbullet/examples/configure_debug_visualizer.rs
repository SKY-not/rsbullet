use rsbullet::{BulletResult, DebugVisualizerOptions, Mode, PhysicsClient};

fn main() -> BulletResult<()> {
    let mut client = PhysicsClient::connect(Mode::Gui)?;
    client
        .set_default_search_path()?
        .set_gravity([0., 0., -10.])?;
    client.load_urdf("r2d2.urdf", Some([0., 0., 1.]))?;
    client.load_urdf("plane.urdf", None::<()>)?;

    client
        .configure_debug_visualizer(&DebugVisualizerOptions::ShadowMapWorldSize(5))?
        .configure_debug_visualizer(&DebugVisualizerOptions::ShadowMapResolution(8192))?;
    let mut t = 0.;
    const DELTA_T: f64 = 1. / 240.;
    const RADIUS: f64 = 5.;
    loop {
        t += DELTA_T;
        client.configure_debug_visualizer(&DebugVisualizerOptions::LightPosition([
            (RADIUS * t.sin()) as f32,
            (RADIUS * t.cos()) as f32,
            1.,
        ]))?;
    }
}
