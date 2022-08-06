use crate::buff_hid::HidBuffer;
use rosrust::ros_info;
use rosrust_msg::{std_msgs, std_msgs::Float64MultiArray};
use std::time::Duration;
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::Instant,
};

pub struct IMU_6DOF {
    pub timestamp: Instant,
    pub input: Arc<RwLock<Vec<f64>>>,
}

impl IMU_6DOF {
    pub fn new(input: Arc<RwLock<Vec<f64>>>) -> IMU_6DOF {
        IMU_6DOF {
            timestamp: Instant::now(),
            input: input,
        }
    }
}

pub struct IMU {
    input: IMU_6DOF,
}

impl IMU {
    pub fn new(hid_input: Arc<RwLock<Vec<f64>>>) -> IMU {
        IMU {
            input: IMU_6DOF::new(hid_input),
        }
    }

    pub fn spin(&mut self) {
        let timestamp = Instant::now();

        if (timestamp.elapsed().as_millis() as u64) < 25 {
            sleep(Duration::from_millis(
                25 - timestamp.elapsed().as_millis() as u64,
            ));
        }
    }
}
