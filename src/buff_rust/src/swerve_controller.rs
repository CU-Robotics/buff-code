use crate::device::MotorTable;
// use rosrust::ros_info;
// use rosrust_msg::{std_msgs, std_msgs::Float64MultiArray};
use std::{
    thread::sleep,
    time::{Duration, Instant},
};

pub struct SwerveController {
    pub motors: MotorTable,
}

impl SwerveController {
    pub fn new(motortable: MotorTable) -> SwerveController {
        SwerveController { motors: motortable }
    }

    pub fn set_motor(&mut self, idx: u8, power: f32) {
        let motor;
        match idx {
            0 => motor = self.motors.fl_drive.as_ref().unwrap(),
            1 => motor = self.motors.fr_drive.as_ref().unwrap(),
            2 => motor = self.motors.rr_drive.as_ref().unwrap(),
            3 => motor = self.motors.rl_drive.as_ref().unwrap(),

            4 => motor = self.motors.fl_steer.as_ref().unwrap(),
            5 => motor = self.motors.fr_steer.as_ref().unwrap(),
            6 => motor = self.motors.rr_steer.as_ref().unwrap(),
            7 => motor = self.motors.rl_steer.as_ref().unwrap(),

            _ => return,
        }
        let mut output = motor.write().unwrap();
        *output = f32::to_le_bytes(power).to_vec();
    }

    pub fn spin(&mut self) {
        let mut ctr = 0;
        let mut f = 1.0;
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.set_motor(ctr, f);
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
