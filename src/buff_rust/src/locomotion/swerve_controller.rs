// use crate::hid::device::MotorTable;
use nalgebra::abs;
use rosrust::ros_info;
use rosrust_msg::std_msgs;
use std::{
    fs::OpenOptions,
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};
// use crate::localization::estimator::*;
// use crate::localization::swerve::*;

pub struct SwerveController {
    pub motor_names: Vec<String>,
    pub remote_control: Arc<RwLock<Vec<f64>>>,
    pub inertial_feedback: Arc<RwLock<Vec<f64>>>,
    pub motor_feedback: Vec<Arc<RwLock<Vec<f64>>>>,
    pub motor_error: Vec<Vec<f64>>,
    pub motor_gains: Vec<Vec<Vec<f64>>>,
    pub publishers: Vec<rosrust::Publisher<std_msgs::Float64>>,
    pub subscribers: Vec<rosrust::Subscriber>,
    pub timestamp: Instant,
    // pub robot_state: Arc<RwLock<SwerveState>>,
}

impl SwerveController {
    pub fn new() -> SwerveController {
        let can_desc = format!("/buffbot/can");

        let motor_info = rosrust::param(&can_desc)
            .unwrap()
            .get::<Vec<Vec<String>>>()
            .unwrap();

        let names: Vec<String> = motor_info.iter().map(|motor| motor[0].clone()).collect();

        let rmt_ctrl = Arc::new(RwLock::new(vec![0f64; 10]));
        let inr_ref = Arc::new(RwLock::new(vec![0f64; 6]));
        let motor_err = vec![vec![0.0, 0.0]; names.len()];
        let mut motor_ref = vec![];
        // let mref_clone = motor_ref.clone();

        let mut pubs = vec![];
        let mut subs = vec![];
        let mut gains = vec![];

        names.iter().enumerate().for_each(|(i, x)| {
            pubs.push(rosrust::publish(format!("{}_command", x).as_str(), 1).unwrap());

            motor_ref.push(Arc::new(RwLock::new(vec![0f64; 3])));
            let ref_clone = motor_ref[i].clone();
            subs.push(
                rosrust::subscribe(
                    format!("{}_state", x).as_str(),
                    1,
                    move |msg: std_msgs::Float64MultiArray| {
                        *ref_clone.write().unwrap() = msg.data;
                    },
                )
                .unwrap(),
            );

            gains.push(
                rosrust::param(format!("/buffbot/{}_gains", x).as_str())
                    .unwrap()
                    .get::<Vec<Vec<f64>>>()
                    .unwrap(),
            );
        });

        let rmt = rmt_ctrl.clone();

        subs.push(
            rosrust::subscribe(
                "receiver_raw",
                1,
                move |msg: std_msgs::Float64MultiArray| {
                    *rmt.write().unwrap() = msg.data;
                },
            )
            .unwrap(),
        );

        SwerveController {
            motor_names: names,
            remote_control: rmt_ctrl,
            inertial_feedback: inr_ref,
            motor_feedback: motor_ref,
            motor_error: motor_err,
            motor_gains: gains,
            publishers: pubs,
            subscribers: subs,
            timestamp: Instant::now(),
            // robot_state: state,
        }
    }

    pub fn set_motor_power(&mut self, name: String, power: f64) {
        let idx = self.motor_names.iter().position(|n| *n == name).unwrap();
        let mut msg = std_msgs::Float64::default();
        msg.data = power;
        self.publishers[idx].send(msg).unwrap();
    }

    pub fn get_motor_angle(&self, name: String) -> f64 {
        let idx = self.motor_names.iter().position(|n| *n == name).unwrap();
        self.motor_feedback[idx].read().unwrap()[0].clone()
    }

    pub fn get_motor_speed(&self, name: String) -> f64 {
        let idx = self.motor_names.iter().position(|n| *n == name).unwrap();
        self.motor_feedback[idx].read().unwrap()[1].clone()
    }

    pub fn get_motor_idx(&self, name: String) -> usize {
        self.motor_names.iter().position(|n| *n == name).unwrap()
    }

    // pub fn swerve_IK(&self, xy: Vector3<f64>, omega: f64, r: Vector2<f64>) -> Vector2<f64> {
    //     let module_velocity = xy + Vector2::new(0.0, omega);
    // }

    // pub fn set_remote_control(&mut self) {
    //     let control = self.remote_control.read().unwrap().clone();

    //     self.set_motor("fl_steer".to_string(), 0.0);
    //     self.set_motor("fr_steer".to_string(), 0.0);
    //     self.set_motor("rr_steer".to_string(), 0.0);
    //     self.set_motor("rl_steer".to_string(), 0.0);

    //     self.set_motor("fl_drive".to_string(), 0.0);
    //     self.set_motor("fr_drive".to_string(), 0.0);
    //     self.set_motor("rr_drive".to_string(), 0.0);
    //     self.set_motor("rl_drive".to_string(), 0.0);
    // }

    pub fn set_motor_pos(&mut self, name: String, setpoint: f64) {
        let mut error = setpoint - self.get_motor_angle(name.clone());

        if error > std::f64::consts::PI {
            error = (2.0 * std::f64::consts::PI) - error;
        } else if error < -std::f64::consts::PI {
            error += 2.0 * std::f64::consts::PI;
        }

        let midx = self.get_motor_idx(name.clone());
        let derr = error - self.motor_error[midx][0];

        self.motor_error[midx][0] = error;
        self.motor_error[midx][1] += (error + self.motor_error[midx][1]).clamp(-1.0, 1.0);

        let p = self.motor_gains[midx][0][0];
        let i = self.motor_gains[midx][0][1];
        let d = self.motor_gains[midx][0][2];

        let speed = (p * error) + (i * self.motor_error[midx][1]) + (d * derr);

        // ros_info!("speed {}", speed);
        self.set_motor_speed(name, speed);
    }

    pub fn set_motor_speed(&mut self, name: String, setpoint: f64) {
        let mut error = setpoint - self.get_motor_speed(name.clone());
        let midx = self.get_motor_idx(name.clone());
        let derr = error - self.motor_error[midx][0];

        self.motor_error[midx][0] = error;
        self.motor_error[midx][1] = (error + self.motor_error[midx][1]).clamp(-1.0, 1.0);

        let p = self.motor_gains[midx][1][0];
        let i = self.motor_gains[midx][1][1];
        let d = self.motor_gains[midx][1][2];

        let power = (p * error) + (i * self.motor_error[midx][1]) + (d * derr);

        // ros_info!("power {}", power);
        self.set_motor_power(name, power);
    }

    pub fn write_sample(&mut self, fname: String) {
        // pub fn write_sample(&mut self, fname: String, motor_name: String) {
        // write the data sample to a file

        //let midx = self.get_motor_idx(name.clone());
        //let rmt = self.remote_control.read().unwrap();
        //let motor_states = self.motor_feedback[midx].read().unwrap();

        // let mut file = File::open(fname).unwrap();

        // println!("We should be printing in file");

        let mut file = OpenOptions::new()
            .append(true)
            .open(fname)
            .expect("cannot open file");

        file.write_all("We did it Joe!".as_bytes())
            .expect("write failed");

        // let data = "we did it joe!";

        // fs::write(fname, data).expect("Unable to write file");

        // file.write(rmt).unwrap();
        // file.write(motor_states).unwrap();
    }

    pub fn spin(&mut self) {
        // let mut ctr = 0;
        let mut f: f64 = 0.0;
        let mut timestamp;

        let mut switch_value = 3.0;

        let max_rpm = 6.28;
        let left_joystick_vpos_max = 680.0;
        let left_joystick_vpos_min = -552.0;
        let right_joystick_vpos_max = 660.0;
        let right_joystick_vpos_min = -756.0;

        let mut record = false;

        while rosrust::is_ok() {
            timestamp = Instant::now();
            let rmt = self.remote_control.read().unwrap();
            switch_value = rmt[5];
            drop(rmt);

            println!("switch position: {}", switch_value);

            // up: 1.0, middle: 3.0, down: 2.0
            if switch_value == 3.0 {
                record = true;
            } else if switch_value == 2.0 {
                record = false;
                println!("switch position");
                self.write_sample("/home/mdyse/buff-code/data/control_log.txt".to_string());
            } else {
                // erase file
            }

            // let mut left_joystick_vpos =
            //     rmt[3].clamp(left_joystick_vpos_min, left_joystick_vpos_max);
            // let mut right_joystick_vpos =
            //     rmt[1].clamp(right_joystick_vpos_min, right_joystick_vpos_max);

            // let mut motor1_rpm = max_rpm * (left_joystick_vpos / left_joystick_vpos_max);

            // println!("{}", motor1_rpm);

            // self.set_motor_speed("feeder".to_string(), -120.0 * 36.0);
        }
    }
}
