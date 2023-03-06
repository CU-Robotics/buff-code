use buff_rust::locomotion::locomotion::*;

fn main() {
    env_logger::init();

    rosrust::init("buff_locomotion");

    let mut controller = BuffLocomotion::new();

    controller.spin();
}
