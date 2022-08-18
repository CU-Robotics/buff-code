use crate::hid::device::MotorTable;
// use rosrust::ros_info;
// use rosrust_msg::{std_msgs, std_msgs::Float64MultiArray};
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

// use crate::localization::estimator::*;
// use crate::localization::swerve::*;

pub struct SwerveController {
    pub motors: Arc<RwLock<MotorTable>>,
    // pub robot_state: Arc<RwLock<SwerveState>>,
}

impl SwerveController {
    pub fn new(motortable: Arc<RwLock<MotorTable>>) -> SwerveController {
        SwerveController {
            motors: motortable,
            // robot_state: state,
        }
    }

    pub fn set_motor(&mut self, idx: usize, power: f64) {
        let mut motors = self.motors.write().unwrap();
        motors.data[idx] = vec![power];
    }

    pub fn spin(&mut self) {
        let mut ctr = 0;
        let mut f = 1.0;
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            // self.set_motor(ctr, f);
            ctr += 1;

            if ctr >= 8 {
                ctr = 0;
                if f > 0.0 {
                    f = 0.0;
                } else if f == 0.0 {
                    f = -1.0;
                } else {
                    f = 1.0;
                }
            }

            let micros = timestamp.elapsed().as_micros();

            if micros < 1000 {
                sleep(Duration::from_micros(1000 - micros as u64));
            }
        }
    }
}
