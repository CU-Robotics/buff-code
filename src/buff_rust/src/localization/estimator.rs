use crate::localization::swerve::*;
use nalgebra::Vector3;
use std::{
    // sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

pub struct RobotStateEstimator {
    timestamp: Instant,

    robot_state: Body3D,

    // motors: Arc<RwLock<MotorTable>>,
    // imu_input: Arc<RwLock<RawInput>>,
    // dr16_input: Arc<RwLock<RawInput>>,
    // remote_control: Arc<RwLock<Vec<f64>>>,
    // inertial_reference: Arc<RwLock<Vec<f64>>>,
    // motor_reference: Arc<RwLock<Vec<f64>>>,
    // can_sub: rosrust::Subscriber,
    // imu_sub: rosrust::Subscriber,
    inertial_feedback: Vector3<f64>,
    acceleration_feedback: Vector3<f64>,
    motor_states: Vec<Vec<f64>>,
    tf_pub: rosrust::Publisher<tf2_msgs::TFMessage>,
}

impl RobotStateEstimator {
    pub fn new(filepath: String) -> RobotStateEstimator {
        let tf_publisher = rosrust::publish("tf", 1).unwrap();

        // let sub = rosrust::subscribe("imu_raw", 5, move |msg: std_msgs::UInt8MultiArray| {
        //     let mut proc_msg = std_msgs::Float64MultiArray::default();
        //     proc_msg.data = msg
        //         .data
        //         .chunks_exact(4)
        //         .map(|chunk| f32::from_be_bytes(chunk.try_into().unwrap_or([0, 0, 0, 0])) as f64)
        //         .collect();

        //     publisher.send(proc_msg);
        // })
        // .unwrap();

        let body = Body3D::from_urdf(filepath);

        RobotStateEstimator {
            timestamp: Instant::now(),

            tf_pub: tf_publisher,

            inertial_feedback: Vector3::new(0.0, 0.0, 0.0),
            acceleration_feedback: Vector3::new(0.0, 0.0, 0.0),
            motor_states: vec![vec![0f64; 3]; body.frames.len()],

            robot_state: body,
        }
    }

    // pub fn get_remote_control(&self) -> Arc<RwLock<Vec<f64>>> {
    //     self.remote_control.clone()
    // }

    // pub fn get_inertial_reference(&self) -> Arc<RwLock<Vec<f64>>> {
    //     self.inertial_reference.clone()
    // }

    // pub fn get_motor_reference(&self) -> Arc<RwLock<Vec<f64>>> {
    //     self.motor_reference.clone()
    // }

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
        self.tf_pub
            .send(msg)
            .expect("joint states Failed to publish");
    }

    pub fn spin(&mut self) {
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            // {
            //     let motors = self.motors.read().unwrap();
            //     let data = motors.data.clone();
            //     let names = motors.names.clone();
            //     let timestamp = motors.timestamp.clone();
            //     drop(motors);

            //     *self.motor_reference.write().unwrap() = data.iter().map(|d| d[1]).collect();
            //     self.robot_state.set_motors(names, data, timestamp);
            // }

            // {
            //     // println!("esty Taking buffer ->");
            //     let dr16 = self.dr16_input.read().unwrap();
            //     let data = dr16.data.clone();
            //     let timestamp = dr16.timestamp.clone();
            //     drop(dr16);
            //     // println!("esty droping buffer ->");

            //     *self.remote_control.write().unwrap() = vec![
            //         data[0], data[1], 0.0, 0.0, 0.0, data[2], 0.0, data[3], data[4],
            //     ];
            // }

            // {
            //     // println!("esty Taking buffer ->");
            //     let imu = self.imu_input.read().unwrap();
            //     let data = imu.data.clone();
            //     let timestamp = imu.timestamp.clone();
            //     drop(imu);
            //     // println!("esty droping buffer ->");

            //     *self.inertial_reference.write().unwrap() = data;
            // }

            // self.robot_state.integrate_step();
            self.broadcast_state();

            if (timestamp.elapsed().as_millis() as u64) < 100 {
                sleep(Duration::from_millis(
                    100 - timestamp.elapsed().as_millis() as u64,
                ));
            }
        }
    }
}
