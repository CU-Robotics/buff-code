use buff_rust::locomotion::controllers::*;
use std::thread;

fn main() {
    env_logger::init();

    rosrust::init("buff_locomotion");

    let mut controller = BuffLocomotion::new();

    controller.spin();
}
