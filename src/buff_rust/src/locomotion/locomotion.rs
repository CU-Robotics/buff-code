use rosrust_msg::std_msgs;
use std::{
    sync::{Arc, RwLock},
    time::Instant,
};

use crate::buff_util::buff_utils::*;
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

    pub motor_index: Vec<String>,

    pub state_controller: StateController,

    pub state_feedback: Arc<RwLock<Vec<f64>>>,
    pub state_reference: Arc<RwLock<Vec<f64>>>,

    pub motor_controllers: Vec<PidController>,

    pub motor_feedback: Arc<RwLock<Vec<f64>>>,

    pub subscribers: Vec<rosrust::Subscriber>,
    pub publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,

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
        // veloctiy controller gains
        let gains = byu.load_float_matrix("motor_control_gains");
        // state controller gain matrix
        let k = byu.load_float_matrix("velocity_control_law");
        // state control input id
        let state_f = byu.load_string("velocity_state_feedback");
        let state_r = byu.load_string("velocity_state_reference");

        // load controller rates: (v)elocity, (p)osition, (s)tate
        let v_rate = byu.load_u128("pid_control_rate");
        let s_rate = byu.load_u128("state_control_rate");

        let mut subs = vec![];

        // Use the list of motors to create the publishers and subscribers
        // Also save the gains for each motor
        let ref_clone = Arc::new(RwLock::new(vec![0f64; 3 * motor_ind.len()]));
        let motor_ref = ref_clone.clone();
        let command_pub = rosrust::publish("motor_commands", 1).unwrap();
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

        let state_fb = Arc::new(RwLock::new(vec![0f64; k[0].len()]));
        let stf_clone = state_fb.clone();

        // Create the remote control subscriber (will also need IMU)
        subs.push(
            rosrust::subscribe(&state_f, 1, move |msg: std_msgs::Float64MultiArray| {
                *stf_clone.write().unwrap() = msg.data;
            })
            .unwrap(),
        );

        let state_rf = Arc::new(RwLock::new(vec![0f64; k[0].len()]));
        let str_clone = state_rf.clone();

        // Create the remote control subscriber (will also need IMU)
        subs.push(
            rosrust::subscribe(&state_r, 1, move |msg: std_msgs::Float64MultiArray| {
                *str_clone.write().unwrap() = msg.data;
            })
            .unwrap(),
        );

        // Init Velocity controllers
        let mc = gains.into_iter().map(|x| PidController::new(x)).collect();

        // init state controller
        let sc = StateController::new(k);

        BuffLocomotion {
            // controller rates
            orate: s_rate,
            irate: v_rate,

            // for motor/controller indexing
            motor_index: motor_ind,

            state_feedback: state_fb,
            state_reference: state_rf,

            state_controller: sc,
            motor_controllers: mc,

            motor_feedback: motor_ref,

            publisher: command_pub,
            subscribers: subs,

            timestamp: Instant::now(),
        }
    }

    pub fn get_motor_index(&self, motor_name: String) -> usize {
        self.motor_index
            .iter()
            .position(|mn| *mn == motor_name)
            .unwrap() as usize
    }

    pub fn get_motor_feedback(&self) -> Vec<Vec<f64>> {
        let mut feedback = vec![];
        self.motor_feedback
            .read()
            .unwrap()
            .clone()
            .chunks(3)
            .for_each(|x| feedback.push(x.to_vec()));
        feedback
    }

    pub fn get_velocity_reference(&self) -> Vec<f64> {
        let state = self.state_reference.read().unwrap().clone();
        vec![state[2], state[3], state[4], 0.0, 0.0, 0.0]
    }

    pub fn update_motor_controllers(&mut self, references: Vec<f64>) {
        // Update the velocity controllers & publish
        let feedback = self.get_motor_feedback();
        let mut commands = vec![0f64; self.motor_index.len()];

        references.iter().enumerate().for_each(|(i, r)| {
            let error = r - feedback[i][1];
            commands[i] = self.motor_controllers[i].update(error);
        });

        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = commands;
        self.publisher.send(msg).unwrap();
    }

    pub fn update_state_control(&self) -> Vec<f64> {
        self.state_controller.update(self.get_velocity_reference())
    }

    pub fn spin(&mut self) {
        let mut timestamp_m = Instant::now();
        let mut timestamp_s = Instant::now();
        let mut micros;

        let mut motor_control = vec![0f64; self.motor_index.len()];

        while rosrust::is_ok() {
            // if micros > rate as microseconds
            micros = timestamp_m.elapsed().as_micros();
            if micros > 1e6 as u128 / self.irate {
                // update the motor signals
                timestamp_m = Instant::now();
                self.update_motor_controllers(motor_control.clone());
            }

            micros = timestamp_s.elapsed().as_micros();
            if micros > 1e6 as u128 / self.orate {
                // Update the state to find control in acceleration
                timestamp_s = Instant::now();
                motor_control = self.update_state_control();
            }
        }
    }
}
