mod buff_hid;
mod localization;
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

    let hid_imu = Arc::clone(&layer.imu_input);
    let hid_dr16 = Arc::clone(&layer.dr16_input);
    let hid_output_queue = Arc::clone(&layer.output_queue);

    let mut imu = localization::IMU::new(hid_imu);
    let mut rc = remote_control::RemoteInput::new(hid_dr16);

    let hid_handle = thread::spawn(move || {
        layer.spin();
    });

    let imu_handle = thread::spawn(move || {
        imu.spin();
    });

    let rc_handle = thread::spawn(move || {
        rc.spin();
    });

    hid_handle.join().unwrap();
    imu_handle.join().unwrap();
    rc_handle.join().unwrap();
}
