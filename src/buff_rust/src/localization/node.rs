use buff_rust::localization::{estimator::*, swerve::*};
use rosrust::ros_info;
use std::thread;

fn main() {
    // launch programs in threads here or as scripts in run
    env_logger::init();

    rosrust::init("buff_localization");

    let robot_desc = format!("/buffbot/self");
    let filepath = rosrust::param(&robot_desc)
        .unwrap()
        .get::<String>()
        .unwrap();

    // let mut layer = HidLayer::new();

    // let can_handle = thread::spawn(move || {
    //     CAN_spin();
    // });

    // layer.spin();

    // can_handle.join().unwrap();
}
