use rosrust_msg::std_msgs;
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

use crate::utilities::loaders::*;

pub struct KinematicEncoderEstimator {
    pub rate: u128,
    pub kinematic_matrix: Vec<Vec<f64>>,
    pub state: Vec<f64>,
    pub motor_hist: Vec<f64>,
    pub motor_feedback: Arc<RwLock<Vec<f64>>>,
    pub subscribers: Vec<rosrust::Subscriber>,
    pub state_publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
    pub accel_publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
}

impl KinematicEncoderEstimator {
    pub fn new() -> KinematicEncoderEstimator {
        let byu = BuffYamlUtil::default();
        // motor config info (only needed for name indexing)
        let rate = byu.load_u128("pid_control_rate");
        let k_mat = byu.load_float_matrix("kinematic_matrix");
        let state_ref = byu.load_string("velocity_state_feedback");

        let n = k_mat[0].len();

        let mut subs = vec![];

        let ref_clone = Arc::new(RwLock::new(vec![0f64; 3 * n]));
        let motor_feedback = ref_clone.clone();
        subs.push(
            rosrust::subscribe(
                "motor_feedback",
                1,
                move |msg: std_msgs::Float64MultiArray| {
                    *ref_clone.write().unwrap() = msg.data;
                },
            )
            .unwrap(),
        );

        let publ = rosrust::publish(state_ref.as_str(), 1).unwrap();
        let pubr = rosrust::publish("motor_acceleration", 1).unwrap();

        KinematicEncoderEstimator {
            rate: rate,
            kinematic_matrix: k_mat,
            state: vec![0f64; 12],
            motor_hist: vec![0f64; n],
            motor_feedback: motor_feedback,
            subscribers: subs,
            state_publisher: publ,
            accel_publisher: pubr,
        }
    }

    pub fn get_motor_angles(&mut self) -> Vec<f64> {
        self.motor_feedback
            .read()
            .unwrap()
            .chunks(3)
            .map(|fb| fb[0])
            .collect()
    }

    pub fn get_motor_speeds(&mut self) -> Vec<f64> {
        self.motor_feedback
            .read()
            .unwrap()
            .chunks(3)
            .map(|fb| fb[1])
            .collect()
    }

    pub fn accel_from_encoder_speed(&mut self) -> Vec<f64> {
        let vel = self.get_motor_speeds();
        let accel = vel
            .iter()
            .zip(self.motor_hist.iter())
            .map(|(v, h)| (v - h) / (self.rate as f64))
            .collect();
        self.motor_hist = vel;
        accel
    }

    pub fn publish_accel(&mut self) {
        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = self.accel_from_encoder_speed();
        self.accel_publisher.send(msg).unwrap();
    }

    pub fn update(&mut self) {
        // self.publish_accel();
        let x = self.get_motor_speeds();

        self.state = self
            .kinematic_matrix
            .iter()
            .map(|a| a.iter().zip(x.iter()).map(|(a, x)| a * x).sum::<f64>())
            .collect();
    }

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
                println!("KEE overtime {}", micros);
            }
        }
    }
}
