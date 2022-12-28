#![allow(unused_imports)]
use buff_rust::utilities::loaders::*;
use buff_rust::device_manager::devices::*;
use buff_rust::locomotion::locomotion::*;
use buff_rust::teensy_comms::buff_hid::*;

use rosrust_msg::std_msgs;
use std::{
    env,
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

#[cfg(test)]
pub mod live_step_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn test_pid() {
        let motor_id = 4;

        let timestamp = Instant::now();
        env_logger::init();

        rosrust::init("buff_control_functional_test");

        let byu = BuffYamlUtil::default();
        // motor config info (only needed for name indexing)
        let rate = byu.load_u128("pid_control_rate");

        let mut layer = HidLayer::new();
        let can_pipeline = CANPipeline::new();
        let mut controller = BuffLocomotion::new();

        layer.init_comms();

        let mut speed = vec![0f64; controller.motor_index.len()];
        let mut ctr = 0;

        let pubr = rosrust::publish("motor_references", 1).unwrap();

        let mut micros;

        while rosrust::is_ok() {
            let mut msg = std_msgs::Float64MultiArray::default();
            msg.data = speed.clone();
            pubr.send(msg).unwrap();

            can_pipeline.publish_can_packet();
            can_pipeline.publish_feedback_packet();

            layer.read();
            layer.write();

            controller.update_motor_controllers(speed.clone());

            ctr += 1;
            if ctr >= 200 {
                speed[motor_id] = 2000.0; // += 5000.0 * (timestamp.elapsed().as_millis() as f64).sin(); //
                                          // speed[motor_id] %= 8000.0;
                ctr = 0;
            }

            micros = timestamp.elapsed().as_micros();
            if micros > (1e6 as u128 / rate) * 500 {
                break;
            }
            if micros < 1e6 as u128 / rate {
                sleep(Duration::from_micros(
                    ((1e6 as u128 / rate) - micros) as u64,
                ));
            }
        }
    }
}
