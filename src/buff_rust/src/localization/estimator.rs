use crate::hid::device::*;
use crate::localization::swerve::*;
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

pub struct RobotStateEstimator {
    timestamp: Instant,

    robot_state: Body3D,

    motors: Arc<RwLock<MotorTable>>,
    imu_input: Arc<RwLock<RawInput>>,
    dr16_input: Arc<RwLock<RawInput>>,

    tf_pub: Option<rosrust::Publisher<tf2_msgs::TFMessage>>,
}

impl RobotStateEstimator {
    pub fn new(
        filepath: String,
        motortable: Arc<RwLock<MotorTable>>,
        dr16_queue: Arc<RwLock<RawInput>>,
        imu_queue: Arc<RwLock<RawInput>>,
    ) -> RobotStateEstimator {
        let dbg = format!("/buffbot/buff_rust/debug");
        let debug = rosrust::param(&dbg).unwrap().get::<bool>().unwrap();

        let mut tf_publisher = None;

        if debug {
            tf_publisher = Some(rosrust::publish("tf", 1).unwrap());
        }

        RobotStateEstimator {
            timestamp: Instant::now(),
            imu_input: imu_queue,
            dr16_input: dr16_queue,
            motors: motortable,
            tf_pub: tf_publisher,
            robot_state: Body3D::from_urdf(filepath, debug),
        }
    }

    pub fn broadcast_state(&self) {
        let msg = tf2_msgs::TFMessage {
            transforms: self
                .robot_state
                .frames
                .iter()
                .map(|x| x.as_tfmessages(self.timestamp.elapsed().as_micros() as u32))
                .flatten()
                .collect(),
        };

        if let Some(ros_pub) = &self.tf_pub {
            ros_pub.send(msg).expect("joint states Failed to publish");
        }
    }

    pub fn spin(&mut self) {
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            {
                let motors = self.motors.read().unwrap();
                let data = motors.data.clone();
                let names = motors.names.clone();
                let timestamp = motors.timestamp.clone();
                drop(motors);

                self.robot_state.set_motors(names, data, timestamp);
            }

            {
                // println!("esty Taking buffer ->");
                let dr16 = self.dr16_input.read().unwrap();
                let data = dr16.data.clone();
                let timestamp = dr16.timestamp.clone();
                drop(dr16);
                // println!("esty droping buffer ->");

                self.robot_state.set_control_input(data, timestamp);
            }

            {
                // println!("esty Taking buffer ->");
                let imu = self.imu_input.read().unwrap();
                let data = imu.data.clone();
                let timestamp = imu.timestamp.clone();
                drop(imu);
                // println!("esty droping buffer ->");

                self.robot_state.set_reference_input(data, timestamp);
            }

            self.robot_state.integrate_step();
            self.broadcast_state();

            if (timestamp.elapsed().as_millis() as u64) < 100 {
                sleep(Duration::from_millis(
                    100 - timestamp.elapsed().as_millis() as u64,
                ));
            }
        }
    }
}
