mod buff_hid;
mod device;
mod localization;
mod swerve_controller;

use rosrust::ros_info;
use std::thread;

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

    let mut rstate = localization::RobotState::new(filepath, motor_inpipe);

    // let mut swrv_ctrl = swerve_controller::SwerveController::new(motor_outpipe);

    let rs_handle = thread::spawn(move || {
        rstate.spin();
    });

    // let hid_handle = thread::spawn(move || {
    //     layer.spin();
    // });

    // let ctrl_handle = thread::spawn(move || {
    //     swrv_ctrl.spin();
    // });

    rs_handle.join().unwrap();
    // hid_handle.join().unwrap();
    // ctrl_handle.join().unwrap();
}
