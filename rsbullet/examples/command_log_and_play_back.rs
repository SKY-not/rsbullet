use std::{thread::sleep, time::Duration};

use rsbullet::{BulletResult, LoggingType, Mode, PhysicsClient};

fn main() -> BulletResult<()> {
    let mut client = PhysicsClient::connect(Mode::Gui)?;
    client.set_default_search_path()?;
    let log = client.start_state_logging(LoggingType::AllCommands, "commandLog.bin", None::<()>)?;
    client.load_urdf("plane.urdf", None::<()>)?;
    client.load_urdf("r2b2.urdf", Some([0., 0., 1.]))?;

    client.stop_state_logging(log)?;
    client.reset_simulation()?;
    client.start_state_logging(LoggingType::AllCommands, "commandLog.bin", None::<()>)?;

    while client.is_connected() {
        sleep(Duration::from_secs_f64(1. / 240.));
    }

    Ok(())
}
