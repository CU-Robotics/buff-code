use buff_rust::hid::buff_hid;
use buff_rust::localization::estimator::*;
use buff_rust::swerve_controller;
use rosrust::ros_info;
use std::thread;

// use std::sync::mpsc;
// use std::sync::mpsc::{Receiver, Sender};

fn main() {
    // launch programs in threads here or as scripts in run
    env_logger::init();

    rosrust::init("buff_rust");

    ros_info!("Buff-Rust is up");

    let robot_desc = format!("/buffbot/self");
    let filepath = rosrust::param(&robot_desc)
        .unwrap()
        .get::<String>()
        .unwrap();

    let mut layer = buff_hid::HidLayer::new();

    let motor_inpipe = layer.dtable.get_motor_inputs();
    let motor_outpipe = layer.dtable.get_motor_outputs();
    let dr16_input = layer.dtable.get_remote_input();
    let imu_input = layer.dtable.get_imu_input();

    let mut rstate = RobotStateEstimator::new(filepath, motor_inpipe, dr16_input, imu_input);

    // let robot_state = rstate.get_state_copy();

    // let mut swrv_ctrl = swerve_controller::SwerveController::new(motor_outpipe);

    let rs_handle = thread::spawn(move || {
        rstate.spin();
    });

    let hid_handle = thread::spawn(move || {
        layer.spin();
    });

    // let ctrl_handle = thread::spawn(move || {
    //     swrv_ctrl.spin();
    // });

    rs_handle.join().unwrap();
    hid_handle.join().unwrap();
    // ctrl_handle.join().unwrap();
}
