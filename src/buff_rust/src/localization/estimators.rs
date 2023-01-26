use rosrust_msg::std_msgs;
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

use crate::utilities::loaders::*;

pub struct RobotStateEstimator {
    pub rate: u128,
    pub kinematic_matrix: Vec<Vec<f64>>,
    pub state: Vec<f64>,
    pub motor_hist: Vec<f64>,
    pub motor_feedback: Vec<Arc<RwLock<Vec<f64>>>>,
    pub subscribers: Vec<rosrust::Subscriber>,
    pub state_publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
}

impl RobotStateEstimator {
    pub fn new() -> RobotStateEstimator {
        let byu = BuffYamlUtil::default();
        // motor config info (only needed for name indexing)
        let rate = byu.load_u128("publish_rate");
        let k_mat = byu.load_float_matrix("kinematic_matrix");
        // let state_ref = byu.load_string("velocity_state_feedback");

        let n = k_mat[0].len();
        let motor_feedback = vec![Arc::new(RwLock::new(vec![0f64; 3])); n];

        let subs = (0..n)
            .map(|i| {
                let ref_clone = motor_feedback[i].clone();
                rosrust::subscribe(
                    format!("motor_{}_feedback", i).as_str(),
                    1,
                    move |msg: std_msgs::Float64MultiArray| {
                        *ref_clone.write().unwrap() = msg.data;
                    },
                )
                .unwrap()
            })
            .collect();

        let publ = rosrust::publish("robot_state", 1).unwrap();

        RobotStateEstimator {
            rate: rate,
            kinematic_matrix: k_mat,
            state: vec![0f64; 12],
            motor_hist: vec![0f64; n],
            motor_feedback: motor_feedback,
            subscribers: subs,
            state_publisher: publ,
        }
    }

    pub fn get_motor_angles(&mut self) -> Vec<f64> {
        self.motor_feedback
            .iter()
            .map(|fb| fb.read().unwrap()[0])
            .collect()
    }

    pub fn get_motor_speeds(&mut self) -> Vec<f64> {
        self.motor_feedback
            .iter()
            .map(|fb| fb.read().unwrap()[1])
            .collect()
    }

    // pub fn accel_from_encoder_speed(&mut self) -> Vec<f64> {
    //     let vel = self.get_motor_speeds();
    //     let accel = vel
    //         .iter()
    //         .zip(self.motor_hist.iter())
    //         .map(|(v, h)| (v - h) / (self.rate as f64))
    //         .collect();
    //     self.motor_hist = vel;
    //     accel
    // }

    // pub fn publish_accel(&mut self) {
    //     let mut msg = std_msgs::Float64MultiArray::default();
    //     msg.data = self.accel_from_encoder_speed();
    //     self.accel_publisher.send(msg).unwrap();
    // }

    pub fn update_kinematics(&mut self) -> Vec<f64> {
        let v = self.get_motor_speeds();

        self.kinematic_matrix
            .iter()
            .map(|a| a.iter().zip(v.iter()).map(|(a, x)| a * x).sum::<f64>())
            .collect()
    }

    // pub fn update_inertial(&mut self) -> Vec<f64> {
    //     // let v = self.get_inertias();

    //     self.kinematic_matrix
    //         .iter()
    //         .map(|a| a.iter().zip(v.iter()).map(|(a, x)| a * x).sum::<f64>())
    //         .collect()
    // }

    pub fn update(&mut self) {}

    pub fn publish_state(&self) {
        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = self.state.clone();
        self.state_publisher.send(msg).unwrap();
    }

    pub fn spin(&mut self) {
        let mut micros;
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.update();
            self.publish_state();

            micros = timestamp.elapsed().as_micros();
            if micros < 1e6 as u128 / self.rate {
                sleep(Duration::from_micros(
                    ((1e6 as u128 / self.rate) - micros) as u64,
                ));
            } else {
                println!("RSE overtime {}", micros);
            }
        }
    }
}
