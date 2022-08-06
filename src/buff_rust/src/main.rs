mod buff_hid;
mod remote_control;

use rosrust::ros_info;
use std::time::Duration;
use std::{sync::Arc, thread, thread::sleep, time::Instant};

use rosrust_msg::{std_msgs, std_msgs::Float64MultiArray};

fn main() {
    // launch programs in threads here or as scripts in run
    env_logger::init();

    rosrust::init("buff_rust");

    let mut layer = buff_hid::HidLayer::new();

    let hid_output = Arc::clone(&layer.output_queue);

    let hid_handle = thread::spawn(move || {
        layer.spin();
    });

    let mut rc = remote_control::RemoteInput::new();

    let rc_handle = thread::spawn(move || {
        rc.spin();
    });

    hid_handle.join().unwrap();
    rc_handle.join().unwrap();
}
