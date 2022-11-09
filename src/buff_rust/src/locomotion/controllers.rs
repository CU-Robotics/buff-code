// use crate::hid::device::MotorTable;
use nalgebra::{abs, Matrix4x3};
use rosrust::ros_info;
use rosrust_msg::std_msgs;
use std::{
    fs::OpenOptions,
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

pub struct PidController {
    /*
        A controller implementation for a PID filter,
        tracks the error and stores the gain. The struct impl
        an update funtion that returns the lastest output.
    */
    pub gain: Vec<f64>,
    pub acc_error: f64,
    pub prev_error: f64,
}

impl PidController {
    pub fn new(gain: Vec<f64>) -> PidController {
        PidController {
            gain: gain,
            acc_error: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn update(&mut self, error: f64) -> f64 {
        /*
            Apply a PID filter to the reference and feedback.
            @param
                the most recently measured error
            @return
                the dot product of [P, I, D] (gains) and [e, de, sum(e)] (current derivitive and integral of error)
        */
        // update the controllers tracked values
        self.prev_error = error;
        self.acc_error += error;

        // compute and return output
        let error_vec = vec![error, error - self.prev_error, self.acc_error];
        error_vec
            .iter()
            .zip(self.gain.iter())
            .map(|(x, y)| x * y)
            .sum()
    }
}

pub struct StateController {
    /*
        uses a reference and transform matrix
        to derive a reference for each motor controller.
    */
    pub state_matrix: Vec<Vec<f64>>,
}

impl StateController {
    pub fn new(state_tf: Vec<Vec<f64>>) -> StateController {
        StateController {
            state_matrix: state_tf,
        }
    }

    pub fn update(&mut self, reference_control: Vec<f64>) -> Vec<f64> {
        /*
            Update the controllers to track the reference input.
            Use the inertial feedback to improve the tracking and estimate.
            @param
                reference_control: could be the remote input or autonomous descision making
                inertial_feedback: measured chassis motion
            @return
                control law applied to the the state reference
        */
        self.state_matrix
            .iter()
            .map(|A| {
                A.iter()
                    .zip(reference_control.iter())
                    .map(|(a, r)| a * r)
                    .sum::<f64>()
            })
            .collect()
    }
}

pub struct BuffLocomotion {
    /*
        Handles remote control/autonomous input to
        drive the defined motors. The state transform
        finds a motor refence for the given control input.
        The motor references are then given to motor
        controllers.
    */
    pub motor_names: Vec<String>,
    pub remote_control: Arc<RwLock<Vec<f64>>>,
    pub inertial_feedback: Arc<RwLock<Vec<f64>>>,
    pub motor_feedback: Vec<Arc<RwLock<Vec<f64>>>>,
    pub publishers: Vec<rosrust::Publisher<std_msgs::Float64>>,
    pub subscribers: Vec<rosrust::Subscriber>,
    pub state_controller: StateController,
    pub velocity_controllers: Vec<PidController>,
    pub vc_index: Vec<String>,
    pub state_reference: String,
    pub control_index: Vec<String>,
    pub timestamp: Instant,
    // pub robot_state: Arc<RwLock<SwerveState>>,
}

impl BuffLocomotion {
    pub fn new() -> BuffLocomotion {
        let motor_info = rosrust::param("/buffbot/can")
            .unwrap()
            .get::<Vec<Vec<String>>>()
            .unwrap();

        let vc_ind = rosrust::param("/buffbot/velocity_controllers")
            .unwrap()
            .get::<Vec<String>>()
            .unwrap();

        let pc_ind = rosrust::param("/buffbot/position_controllers")
            .unwrap()
            .get::<Vec<String>>()
            .unwrap();

        let state_tf = rosrust::param("/buffbot/state_controller/control_law")
            .unwrap()
            .get::<Vec<Vec<f64>>>()
            .unwrap();

        let state_ref = rosrust::param("/buffbot/state_controller/input")
            .unwrap()
            .get::<String>()
            .unwrap();

        let cntrl_index = rosrust::param("/buffbot/state_controller/output")
            .unwrap()
            .get::<Vec<String>>()
            .unwrap();

        let names: Vec<String> = motor_info.iter().map(|motor| motor[0].clone()).collect();

        let rmt_ctrl = Arc::new(RwLock::new(vec![0f64; 10]));
        let inr_ref = Arc::new(RwLock::new(vec![0f64; 6]));
        let mut motor_ref = vec![];

        let mut pubs = vec![];
        let mut subs = vec![];
        let mut gains = vec![];

        names.iter().enumerate().for_each(|(i, x)| {
            pubs.push(rosrust::publish(format!("{}_command", x).as_str(), 1).unwrap());

            let ref_clone = Arc::new(RwLock::new(vec![0f64; 3]));
            motor_ref.push(ref_clone.clone());
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
            rosrust::subscribe(&state_ref, 1, move |msg: std_msgs::Float64MultiArray| {
                *rmt.write().unwrap() = msg.data;
            })
            .unwrap(),
        );

        let vc = vc_ind
            .iter()
            .map(|x| {
                PidController::new(gains[names.iter().position(|n| n == x).unwrap()][0].clone())
            })
            .collect();

        let sc = StateController::new(state_tf);

        BuffLocomotion {
            motor_names: names,
            remote_control: rmt_ctrl,
            inertial_feedback: inr_ref,
            motor_feedback: motor_ref,
            publishers: pubs,
            subscribers: subs,
            state_controller: sc,
            velocity_controllers: vc,
            vc_index: vc_ind,
            state_reference: state_ref,
            control_index: cntrl_index,
            timestamp: Instant::now(),
            // robot_state: state,
        }
    }

    pub fn spin(&mut self) {
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            // Update the state
            let references = self
                .state_controller
                .update(self.remote_control.read().unwrap().clone());

            // Update the velocity controllers & publish
            self.vc_index.iter().enumerate().for_each(|(i, n)| {
                // find index of reference for controller
                let j = self.control_index.iter().position(|cn| cn == n).unwrap() as usize;
                // find index of motor command publisher
                let mi = self.motor_names.iter().position(|mn| mn == n).unwrap() as usize;
                // get the error
                let error = references[j] - self.motor_feedback[mi].read().unwrap()[1];
                // publish
                let mut msg = std_msgs::Float64::default();
                msg.data = self.velocity_controllers[i].update(error);
                self.publishers[mi].send(msg).unwrap();
            })
        }
    }
}
