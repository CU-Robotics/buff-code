#![allow(unused_imports)]
use buff_rust::buff_util::buff_utils::*;
use buff_rust::device_manager::device::*;
use buff_rust::locomotion::controllers::*;
use buff_rust::teensy_comms::buff_hid::*;

use rosrust_msg::std_msgs;
use std::{
    env,
    sync::{Arc, RwLock},
    time::Instant,
};

#[cfg(test)]
pub mod live_control_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn test_pid() {
        let motor_id = 2;

        let timestamp = Instant::now();
        env_logger::init();

        rosrust::init("buff_control_functional_test");

        let byu = BuffYamlUtil::new("penguin");

        let mut layer = HidLayer::new();
        let can_pipeline = CANPipeline::new();
        let mut dr16_pipeline = DR16Pipeline::new();

        let vc_motor = byu.load_string_list("velocity_control_index")[motor_id].clone();
        let vc_gain = byu.load_float_matrix("velocity_control_gains")[motor_id].clone();

        let fb_clone = Arc::new(RwLock::new(vec![0f64; 3]));

        let fb_vec = fb_clone.clone();
        let feedback_sub = rosrust::subscribe(
            format!("{}_feedback", vc_motor).as_str(),
            1,
            move |msg: std_msgs::Float64MultiArray| {
                *fb_clone.write().unwrap() = msg.data;
            },
        )
        .unwrap();

        let command_pub = rosrust::publish(format!("{}_command", vc_motor).as_str(), 1).unwrap();

        let mut pid = PidController::new(vc_gain);

        layer.init_comms();

        let speed = 5000.0;

        while rosrust::is_ok() {
            can_pipeline.publish_can_packet();
            dr16_pipeline.publish_messages();

            layer.read();
            layer.write();

            let mut msg = std_msgs::Float64::default();
            msg.data = pid.update(speed - fb_vec.read().unwrap()[1]);
            command_pub.send(msg).unwrap();

            if timestamp.elapsed().as_millis() > 1000 {
                break;
            }
        }

        drop(feedback_sub);
    }
}
