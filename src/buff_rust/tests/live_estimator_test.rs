#![allow(unused_imports)]
use buff_rust::localization::estimators::*;
use buff_rust::locomotion::controllers::*;
use buff_rust::teensy_comms::buff_hid::*;
use buff_rust::utilities::loaders::*;

use rosrust_msg::std_msgs;
use std::{
    env,
    sync::{Arc, RwLock},
    time::Instant,
};

#[cfg(test)]
pub mod live_estimator_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    // #[test]
    // pub fn test_ke_estimator() {
    //     let timestamp = Instant::now();
    //     env_logger::init();

    //     rosrust::init("buff_KEE_functional_test");

    //     let mut layer = HidLayer::new();
    //     let can_pipeline = CANPipeline::new();
    //     let mut dr16_pipeline = DR16Pipeline::new();
    //     let mut kee = KinematicEncoderEstimator::new();

    //     layer.init_comms();

    //     while rosrust::is_ok() {
    //         can_pipeline.publish_feedback_packet();
    //         can_pipeline.publish_can_packet();
    //         dr16_pipeline.publish_messages();

    //         layer.read();
    //         layer.write();

    //         kee.update();
    //         kee.publish_state();

    //         if timestamp.elapsed().as_millis() > 20000 {
    //             break;
    //         }
    //     }
    // }

    // #[test]
    // pub fn test_accel_pid() {
    //     let motor_id = 0;

    //     env_logger::init();

    //     rosrust::init("buff_KEE_functional_test");

    //     let timestamp = Instant::now();

    //     let byu = BuffYamlUtil::new("penguin");

    //     let mut layer = HidLayer::new();
    //     let can_pipeline = CANPipeline::new();

    //     let gain = byu.load_float_matrix("motor_control_gains")[motor_id].clone();
    //     let motor_index = byu.load_string_list("motor_index").clone();
    //     let n = motor_index.len();

    //     let command_pub = rosrust::publish("motor_commands", 1).unwrap();

    //     let mut pid = PidController::new(gain);
    //     let mut kee = KinematicEncoderEstimator::new();

    //     layer.init_comms();

    //     let mut accel = 1.0;
    //     let mut ctr = 0;

    //     while rosrust::is_ok() {
    //         can_pipeline.publish_can_packet();
    //         can_pipeline.publish_feedback_packet();

    //         layer.read();
    //         layer.write();

    //         let acc = kee.accel_from_encoder_speed();

    //         let mut msg = std_msgs::Float64MultiArray::default();
    //         msg.data = vec![0f64; n];
    //         println!("error vec {} {} {}", accel, acc[motor_id], accel - acc[motor_id]);
    //         msg.data[motor_id] = pid.update(accel - acc[motor_id]);
    //         command_pub.send(msg).unwrap();

    //         if timestamp.elapsed().as_millis() > 5000 {
    //             break;
    //         }
    //         ctr += 1;
    //         if ctr > 5 {
    //             ctr = 0;
    //             accel *= -1.0;
    //         }
    //     }

    // }
}
