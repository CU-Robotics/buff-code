use buff_rust::locomotion::swerve_controller::*;
use std::thread;

fn main() {
    env_logger::init();

    rosrust::init("buff_control");

    let mut controller = SwerveController::new();

    controller.spin();
}
