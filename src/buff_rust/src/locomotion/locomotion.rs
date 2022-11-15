use rosrust_msg::std_msgs;
use std::{
    sync::{Arc, RwLock},
    time::Instant,
};

use crate::buff_rust::buff_utils::*;
use crate::locomotion::controllers::*;

pub struct BuffLocomotion {
    /*
        Handles remote control/autonomous input to
        drive the defined motors. The state gain
        finds a motor refence for the given control input.
        The motor references are then given to motor
        controllers.
    */
    pub orate: u128,
    pub irate: u128,

    pub vc_index: Vec<String>,
    pub pc_index: Vec<String>,
    pub motor_index: Vec<String>,

    pub state_input: String,
    pub state_controller: StateController,

    pub state_feedback: Arc<RwLock<Vec<f64>>>,
    pub inertial_feedback: Arc<RwLock<Vec<f64>>>,

    pub velocity_controllers: Vec<PidController>,
    pub position_controllers: Vec<CascadedPidController>,

    pub motor_feedback: Vec<Arc<RwLock<Vec<f64>>>>,

    pub subscribers: Vec<rosrust::Subscriber>,
    pub publishers: Vec<rosrust::Publisher<std_msgs::Float64>>,

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
        // position controller gains
        let pc_gains = byu.load_float_matrix("position_control_gains");
        // state controller gain matrix
        let k = byu.load_float_matrix("state_control_law");
        // state control input id
        let state_ref = byu.load_string("state_feedback");

        let mut pubs = vec![];
        let mut subs = vec![];
        let mut motor_ref = vec![];

        // Use the list of motors to create the publishers and subscribers
        // Also save the gains for each motor
        motor_ind.iter().for_each(|x| {
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

        // remote control/IMU feedback ros topic buffer
        let inr_ref = Arc::new(RwLock::new(vec![0f64; 6]));

        let state = Arc::new(RwLock::new(vec![0f64; k[0].len()]));
        let rmt = state.clone();

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

        // Init Velocity controllers
        let pc = pc_gains
            .into_iter()
            .map(|x| CascadedPidController::new(x))
            .collect();

        // init state controller
        let sc = StateController::new(k);

        BuffLocomotion {
            orate: 10,
            irate: 1000,
            vc_index: vc_ind,
            pc_index: pc_ind,
            motor_index: motor_ind,
            state_feedback: state,

            state_input: state_ref,
            state_controller: sc,
            velocity_controllers: vc,
            position_controllers: pc,

            inertial_feedback: inr_ref,
            motor_feedback: motor_ref,

            publishers: pubs,
            subscribers: subs,

            timestamp: Instant::now(),
        }
    }

    pub fn update_motor_controllers(&mut self, references: Vec<f64>) {
        // Update the velocity controllers & publish
        self.vc_index.iter().enumerate().for_each(|(i, n)| {
            // find index of motor controller
            // get the error
            // publish
            let j = self.motor_index.iter().position(|mn| mn == n).unwrap() as usize;
            let error = references[j] - self.motor_feedback[j].read().unwrap()[1];
            let mut msg = std_msgs::Float64::default();
            msg.data = self.velocity_controllers[i].update(error);
            self.publishers[j].send(msg).unwrap();
        });

        self.pc_index.iter().enumerate().for_each(|(i, n)| {
            let j = self.motor_index.iter().position(|mn| mn == n).unwrap() as usize;
            let error = references[j] - self.motor_feedback[j].read().unwrap()[0];
            let mut msg = std_msgs::Float64::default();
            msg.data = self.position_controllers[i]
                .update(error, self.motor_feedback[j].read().unwrap()[1]);
            self.publishers[j].send(msg).unwrap();
        });
    }

    pub fn update_state_reference(&self) -> Vec<f64> {
        self.state_controller
            .update(self.state_feedback.read().unwrap().clone())
    }

    pub fn spin(&mut self) {
        let mut timestamp_m = Instant::now();
        let mut timestamp_s = Instant::now();
        let mut micros;

        let mut references = vec![0f64; self.motor_index.len()];

        while rosrust::is_ok() {
            // if micros > rate as microseconds
            micros = timestamp_m.elapsed().as_micros();
            if micros > 1e6 as u128 / self.irate {
                // update the motor signals
                timestamp_m = Instant::now();
                self.update_motor_controllers(references.clone());
            }

            micros = timestamp_s.elapsed().as_micros();
            if micros > 1e6 as u128 / self.orate {
                // Update the state
                timestamp_s = Instant::now();
                references = self.update_state_reference();
            }
        }
    }
}
