use buff_rust::teensy_comms::{buff_hid::*, device::*};
use std::thread;

fn main() {
    env_logger::init();

    rosrust::init("buff_comms");

    let mut layer = HidLayer::new();

    let can_handle = thread::spawn(move || {
        CANPipeline::spin();
    });

    // let recv_handle = thread::spawn(move || {
    //     dr16_spin();
    // });

    layer.spin();
    can_handle.join().unwrap();
    // recv_handle.join().unwrap();
}
