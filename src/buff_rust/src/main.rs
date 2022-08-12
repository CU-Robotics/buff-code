mod buff_hid;
mod device;
mod swerve_controller;

use rosrust::ros_info;
use std::thread;

fn main() {
    // launch programs in threads here or as scripts in run
    env_logger::init();

    rosrust::init("buff_rust");

    ros_info!("Buff-Rust is up");

    // let robot_desc = format!("/buffbot/self");
    // let filepath = rosrust::param(&robot_desc)
    //     .unwrap()
    //     .get::<String>()
    //     .unwrap();

    // let dt = device::DeviceTable::from_yaml(filepath);

    let mut layer = buff_hid::HidLayer::new();

    let motor_outpipe = layer.dtable.get_motors();

    let mut swrv_ctrl = swerve_controller::SwerveController::new(motor_outpipe);

    let hid_handle = thread::spawn(move || {
        layer.spin();
    });

    let ctrl_handle = thread::spawn(move || {
        swrv_ctrl.spin();
    });

    hid_handle.join().unwrap();
    ctrl_handle.join().unwrap();
}
