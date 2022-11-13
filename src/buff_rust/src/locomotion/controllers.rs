use crate::buff_rust::buff_utils::*;
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
        self.acc_error += error;

        let error_vec = vec![error, error - self.prev_error, self.acc_error];

        self.prev_error = error;

        error_vec
            .iter()
            .zip(self.gain.iter())
            .map(|(x, y)| x * y)
            .sum()
    }
}

pub struct StateController {
    /*
        uses a reference and gain matrix
        to derive a reference for each motor controller.
    */
    pub gain_matrix: Vec<Vec<f64>>,
}

impl StateController {
    pub fn new(k: Vec<Vec<f64>>) -> StateController {
        StateController { gain_matrix: k }
    }

    pub fn update(&self, state: Vec<f64>) -> Vec<f64> {
        /*
            Update the controllers to track the reference input.
            Use the inertial feedback to improve the tracking and estimate.
            @param
                reference_control: could be the remote input or autonomous descision making
                inertial_feedback: measured chassis motion
            @return
                control law applied to the the state reference
        */
        self.gain_matrix
            .iter()
            .map(|k| k.iter().zip(state.iter()).map(|(k, s)| k * s).sum::<f64>())
            .collect()
    }
}

pub struct BuffLocomotion {
    /*
        Handles remote control/autonomous input to
        drive the defined motors. The state gain
        finds a motor refence for the given control input.
        The motor references are then given to motor
        controllers.
    */
    pub rate: u128,
    pub vc_index: Vec<String>,
    pub motor_index: Vec<String>,

    pub state_input: String,
    pub state_controller: StateController,
    pub velocity_controllers: Vec<PidController>,

    pub remote_control: Arc<RwLock<Vec<f64>>>,
    pub inertial_feedback: Arc<RwLock<Vec<f64>>>,
    pub motor_feedback: Vec<Arc<RwLock<Vec<f64>>>>,

    pub publishers: Vec<rosrust::Publisher<std_msgs::Float64>>,
    pub subscribers: Vec<rosrust::Subscriber>,

    pub timestamp: Instant,
}

impl BuffLocomotion {
    /*
        The main Locomotive node in buff-code.
        This struct handles all the ros subscribing/publishing
        as well as the config for state controllers and
        motor controllers.

        BuffLocomotion uses a state description to determine
        approximate references for individual motors to track.
        There are state controllers and motor controllers, the
        motor controllers are SISO implementations, the state
        controllers implement a control law u = -kx.
    */
    pub fn new() -> BuffLocomotion {
        let byu = BuffYamlUtil::default();
        // motor config info (only needed for name indexing)
        let motor_ind = byu.load_string_list("motor_index");
        // velocity controller indexer
        let vc_ind = byu.load_string_list("velocity_control_index");
        // position controller indexer
        let pc_ind = byu.load_string_list("position_control_index");
        // veloctiy controller gains
        let vc_gains = byu.load_float_matrix("velocity_control_gains");
        // state controller gain matrix
        let k = byu.load_float_matrix("state_control_law");
        // state control input id
        let state_ref = byu.load_string("state_control_input");

        // remote control/IMU feedback ros topic buffer
        let rmt_ctrl = Arc::new(RwLock::new(vec![0f64; 10]));
        let inr_ref = Arc::new(RwLock::new(vec![0f64; 6]));

        let mut pubs = vec![];
        let mut subs = vec![];
        let mut motor_ref = vec![];

        // Use the list of motors to create the publishers and subscribers
        // Also save the gains for each motor
        motor_ind.iter().enumerate().for_each(|(i, x)| {
            pubs.push(rosrust::publish(format!("{}_command", x).as_str(), 1).unwrap());

            let ref_clone = Arc::new(RwLock::new(vec![0f64; 3]));
            motor_ref.push(ref_clone.clone());
            subs.push(
                rosrust::subscribe(
                    format!("{}_feedback", x).as_str(),
                    1,
                    move |msg: std_msgs::Float64MultiArray| {
                        *ref_clone.write().unwrap() = msg.data;
                    },
                )
                .unwrap(),
            );
        });

        let rmt = rmt_ctrl.clone();

        // Create the remote control subscriber (will also need IMU)
        subs.push(
            rosrust::subscribe(&state_ref, 1, move |msg: std_msgs::Float64MultiArray| {
                *rmt.write().unwrap() = msg.data;
            })
            .unwrap(),
        );

        // Init Velocity controllers
        let vc = vc_gains
            .into_iter()
            .map(|x| PidController::new(x))
            .collect();

        // init state controller
        let sc = StateController::new(k);

        BuffLocomotion {
            rate: 50,
            vc_index: vc_ind,
            motor_index: motor_ind,
            remote_control: rmt_ctrl,

            state_input: state_ref,
            state_controller: sc,
            velocity_controllers: vc,

            inertial_feedback: inr_ref,
            motor_feedback: motor_ref,

            publishers: pubs,
            subscribers: subs,

            timestamp: Instant::now(),
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
                // find index of motor controller
                let j = self.motor_index.iter().position(|mn| mn == n).unwrap() as usize;
                // get the error
                let error = references[j] - self.motor_feedback[j].read().unwrap()[1];
                // publish
                let mut msg = std_msgs::Float64::default();
                msg.data = self.velocity_controllers[i].update(error);
                self.publishers[j].send(msg).unwrap();
            });

            let micros = timestamp.elapsed().as_micros();
            if micros < 1e6 as u128 / self.rate {
                sleep(Duration::from_micros(
                    ((1e6 as u128 / self.rate) - micros) as u64,
                ));
            } else {
                println!("dr16 manager overtime {}", micros);
            }
        }
    }
}
