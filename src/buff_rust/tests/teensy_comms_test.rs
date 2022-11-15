#![allow(unused_imports)]
use buff_rust::teensy_comms::buff_hid::*;
use rosrust_msg::std_msgs;
use std::{
    env,
    sync::{Arc, RwLock},
    time::Instant,
};

#[cfg(test)]
pub mod hid_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn hid_connection() {
        /*
            test the hidlayers ability to connect to a teensy
            this function will require information from
            a robot setup yaml. We default tests to the
            penguin.
        */

        let can_message_count = Arc::new(RwLock::new(0.0f64));
        let s1_message_count = Arc::new(RwLock::new(0.0f64));
        let s2_message_count = Arc::new(RwLock::new(0.0f64));

        env_logger::init();

        rosrust::init("buff_comms_functional_test");

        let param_id = format!("/buffbot/robot_name");
        rosrust::param(&param_id)
            .unwrap()
            .set::<String>(&"penguin".to_string())
            .unwrap();

        let can_msg_cnt = can_message_count.clone();
        let can_sub = rosrust::subscribe("can_raw", 1, move |_msg: std_msgs::UInt8MultiArray| {
            *can_msg_cnt.write().unwrap() += 1.0;
        })
        .unwrap();

        let s1_msg_cnt = s1_message_count.clone();
        let s1_sub =
            rosrust::subscribe("sensor1_raw", 1, move |_msg: std_msgs::UInt8MultiArray| {
                *s1_msg_cnt.write().unwrap() += 1.0;
            })
            .unwrap();

        let s2_msg_cnt = s1_message_count.clone();
        let s2_sub =
            rosrust::subscribe("sensor2_raw", 1, move |_msg: std_msgs::UInt8MultiArray| {
                *s2_msg_cnt.write().unwrap() += 1.0;
            })
            .unwrap();

        let mut layer = HidLayer::new();

        layer.init_comms();

        while rosrust::is_ok() {
            layer.read();
            layer.write();

            if layer.timestamp.elapsed().as_millis() > layer.nc_timeout {
                println!("HID Connection test timeout");
                break;
            }
        }

        drop(can_sub);
        drop(s1_sub);
        drop(s2_sub);

        println!(
            "Received {} can messages",
            *can_message_count.read().unwrap()
        );
        println!(
            "Received {} sensor1 messages",
            *s1_message_count.read().unwrap()
        );
        println!(
            "Received {} sensor2 messages",
            *s2_message_count.read().unwrap()
        );
        println!("Topics:\ncan_raw\nsensor1\nsensor2");
        println!(
            "Topic counts: {} {} {}",
            *can_message_count.read().unwrap() / 10.0,
            *s1_message_count.read().unwrap() / 10.0,
            *s1_message_count.read().unwrap() / 10.0
        );
        println!(
            "Topic rates (Hz): {} {} {}",
            *can_message_count.read().unwrap() / 10.0,
            *s1_message_count.read().unwrap() / 10.0,
            *s1_message_count.read().unwrap() / 10.0
        );
    }
}
