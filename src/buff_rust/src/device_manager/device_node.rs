use buff_rust::device_manager::devices::*;
use std::thread;

fn main() {
    env_logger::init();

    rosrust::init("buff_devices");

    let can_pipeline = CANPipeline::new();
    let mut dr16_pipeline = DR16Pipeline::new();

    let can_handle = thread::spawn(move || {
        can_pipeline.spin();
    });

    let dr16_handle = thread::spawn(move || {
        dr16_pipeline.spin();
    });

    can_handle.join().unwrap();
    dr16_handle.join().unwrap();
}
