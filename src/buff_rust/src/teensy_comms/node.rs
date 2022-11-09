use buff_rust::teensy_comms::buff_hid::*;
use std::thread;

fn main() {
    env_logger::init();

    rosrust::init("buff_comms");

    let mut layer = HidLayer::new();

    layer.spin();
}
