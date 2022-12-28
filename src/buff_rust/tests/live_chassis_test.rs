#![allow(unused_imports)]
use buff_rust::utilities::loaders::*;
use buff_rust::device_manager::devices::*;
use buff_rust::localization::estimators::*;
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
pub mod live_state_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn test_chassis() {
        let timestamp = Instant::now();
        env_logger::init();

        rosrust::init("buff_control_functional_test");

        let byu = BuffYamlUtil::default();
        let rate = byu.load_u128("pid_control_rate");
        // motor config info (only needed for name indexing)

        let mut layer = HidLayer::new();
        let can_pipeline = CANPipeline::new();
        let mut dr16_pipeline = DR16Pipeline::new();
        let mut controller = BuffLocomotion::new();
        let mut kee = KinematicEncoderEstimator::new();

        layer.init_comms();

        let mut millis;
        let mut references;

        let mut ctr = 0;

        while rosrust::is_ok() {
            ctr += 1;
            if ctr < 200 {
                continue;
            }

            dr16_pipeline.publish_messages();
            can_pipeline.publish_can_packet();
            can_pipeline.publish_feedback_packet();

            layer.read();
            layer.write();

            kee.update();
            kee.publish_state();

            references = controller.update_state_control();
            controller.update_motor_controllers(references);

            millis = timestamp.elapsed().as_millis();
            if millis > 5000 {
                break;
            }
            if millis < 1e3 as u128 / rate {
                sleep(Duration::from_millis(
                    ((1e3 as u128 / rate) - millis) as u64,
                ));
            }
        }
    }
}
