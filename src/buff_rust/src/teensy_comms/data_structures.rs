use crate::utilities::loaders::*;
use chrono::prelude::*;
use std::env;
use std::fs;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::Path;
use std::sync::{Arc, RwLock};

pub struct EmbeddedDevice {
    name: String,
    timestamp: f64,
    data: Vec<f64>,
    config: Vec<u8>,
}

impl EmbeddedDevice {
    pub fn default() -> EmbeddedDevice {
        EmbeddedDevice {
            name: "UED".to_string(),
            timestamp: 0.0,
            data: vec![],
            config: vec![],
        }
    }

    pub fn anonymous(id: u8, data_size: usize, config: Vec<u8>) -> EmbeddedDevice {
        EmbeddedDevice {
            name: format!("embedded_device_{}", id),
            timestamp: 0.0,
            data: vec![0.0; data_size],
            config: config,
        }
    }

    pub fn named(name: String, data_size: usize, config: Vec<u8>) -> EmbeddedDevice {
        EmbeddedDevice {
            name: name,
            timestamp: 0.0,
            data: vec![0.0; data_size],
            config: config,
        }
    }

    pub fn update(&mut self, data: Vec<f64>, time: f64) {
        self.data = data[0..self.data.len()].to_vec();
        self.timestamp = time;
    }

    pub fn print(&self) {
        println!(
            "{}\n\tconfig:\t{:?}\n\tData:\t{}\n\t\t{:?}",
            self.name, self.config, self.timestamp, self.data
        );
    }

    pub fn as_string_vec(&self) -> Vec<String> {
        vec![self.timestamp.to_string()]
            .into_iter()
            .chain(
                self.data
                    .iter()
                    .map(|x| x.to_string())
                    .collect::<Vec<String>>(),
            )
            .collect()
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }

    pub fn data(&self) -> Vec<f64> {
        self.data.clone()
    }

    pub fn config(&self) -> Vec<u8> {
        self.config.clone()
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp.clone()
    }
}

pub struct EmbeddedController {
    name: String,
    limits: Vec<f64>,
    gains: Vec<f64>,
    feedback: Vec<f64>,
    reference: Vec<f64>,

    output: f64,
    timestamp: f64,

    control_type: u8,
}

impl EmbeddedController {
    pub fn default() -> EmbeddedController {
        EmbeddedController {
            name: "UEC".to_string(),
            limits: vec![0.0; 4],
            gains: vec![],
            feedback: vec![],
            reference: vec![],
            output: 0.0,
            timestamp: 0.0,
            control_type: u8::MAX,
        }
    }

    pub fn anonymous(id: u8, state_size: usize) -> EmbeddedController {
        EmbeddedController {
            name: format!("embedded_controller_{}", id),
            limits: vec![0.0; 4],
            gains: vec![0.0; state_size],
            feedback: vec![0.0; state_size],
            reference: vec![0.0; state_size],
            output: 0.0,
            timestamp: 0.0,
            control_type: u8::MAX,
        }
    }

    pub fn named(name: String, state_size: usize, mode: u8) -> EmbeddedController {
        EmbeddedController {
            name: name,
            limits: vec![0.0; 4],
            gains: vec![0.0; state_size],
            feedback: vec![0.0; state_size],
            reference: vec![0.0; state_size],
            output: 0.0,
            timestamp: 0.0,
            control_type: mode,
        }
    }

    pub fn set_config(&mut self, gains: Vec<f64>, limits: Vec<f64>) {
        self.limits = limits;
        self.gains = gains;
    }

    pub fn new(name: String, mode: u8, gains: Vec<f64>, limits: Vec<f64>) -> EmbeddedController {
        let motor_state_len = gains.len();

        EmbeddedController {
            name: name,
            limits: limits,
            gains: gains,
            feedback: vec![0.0; motor_state_len],
            reference: vec![0.0; motor_state_len],
            output: 0.0,
            timestamp: 0.0,
            control_type: mode,
        }
    }

    pub fn config(&self) -> Vec<u8> {
        vec![self.control_type]
            .into_iter()
            .chain(
                self.gains
                    .iter()
                    .map(|k| (*k as f32).to_be_bytes())
                    .flatten()
                    .chain(
                        self.limits
                            .iter()
                            .map(|l| (*l as f32).to_be_bytes())
                            .flatten(),
                    )
                    .collect::<Vec<u8>>(),
            )
            .collect()
    }

    pub fn update(&mut self, feedback: Vec<f64>, time: f64) {
        self.output = feedback[0];
        self.reference = feedback[1..self.gains.len()].to_vec();
        self.feedback = feedback[self.gains.len()..].to_vec();
        self.timestamp = time;
    }

    pub fn print(&self) {
        println!(
            "{}\t\n\tReference\tFeedback\tOutput\n\t{:?} {:?} {}\n\tGains\t\tLimits\n\t{:?} {:?}",
            self.name, self.reference, self.feedback, self.output, self.gains, self.limits
        );
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }

    pub fn output(&self) -> f64 {
        self.output
    }

    pub fn feedback(&self) -> Vec<f64> {
        self.feedback.clone()
    }

    pub fn reference(&self) -> Vec<f64> {
        self.reference.clone()
    }

    pub fn timestamp(&self) -> f64 {
        self.timestamp
    }
}

pub struct RobotStatus {
    pub motors: Vec<Arc<RwLock<EmbeddedDevice>>>,
    pub sensors: Vec<Arc<RwLock<EmbeddedDevice>>>,
    pub controllers: Vec<Arc<RwLock<EmbeddedController>>>,
    pub control_input: Arc<RwLock<Vec<f64>>>,
    pub control_output: Arc<RwLock<Vec<f64>>>,
    pub kee_vel_est: Arc<RwLock<Vec<f64>>>,
    pub imu_vel_est: Arc<RwLock<Vec<f64>>>,
    pub kee_imu_pos: Arc<RwLock<Vec<f64>>>,
    pub enc_mag_pos: Arc<RwLock<Vec<f64>>>,
    pub power_buffer: Arc<RwLock<f64>>,
    pub control_mode: Arc<RwLock<f64>>,

    pub forward_kinematics: Vec<Vec<f64>>,
    pub inverse_kinematics: Vec<Vec<f64>>,
}

impl RobotStatus {
    pub fn default() -> RobotStatus {
        RobotStatus {
            motors: vec![],
            sensors: vec![],
            controllers: vec![],
            control_input: Arc::new(RwLock::new(vec![])),
            control_output: Arc::new(RwLock::new(vec![])),
            kee_vel_est: Arc::new(RwLock::new(vec![])),
            imu_vel_est: Arc::new(RwLock::new(vec![])),
            kee_imu_pos: Arc::new(RwLock::new(vec![])),
            enc_mag_pos: Arc::new(RwLock::new(vec![])),
            power_buffer: Arc::new(RwLock::new(0.0)),
            control_mode: Arc::new(RwLock::new(0.0)),
            forward_kinematics: vec![vec![]],
            inverse_kinematics: vec![vec![]],
        }
    }

    pub fn from_byu(byu: BuffYamlUtil) -> RobotStatus {
        let sensor_index = byu.load_string_list("sensor_index");
        let sensor_buffers = byu.load_u8_list("sensor_buffers");

        let motor_index = byu.load_string_list("motor_index");
        let motor_gains = byu.load_float_matrix("motor_gains");
        let motor_limits = byu.load_float_matrix("motor_limits");
        let motor_can_index = byu.load_integer_matrix("motor_can_index");
        let motor_state_len = motor_gains[0].len();

        let controller_types = byu.load_integer_matrix("motor_controller_types");
        let forward_kinematics = byu.load_float_matrix("forward_kinematics");
        let inverse_kinematics = byu.load_float_matrix("inverse_kinematics");
        let robot_state_len = forward_kinematics.len();

        assert!(
            controller_types.len() == motor_index.len(),
            "Number of Controllers and Motors should match"
        );
        assert!(
            motor_can_index.len() == motor_index.len(),
            "Number of CAN motor configs should match the number of motors"
        );
        assert!(
            inverse_kinematics.len() == motor_index.len()
                && forward_kinematics[0].len() == motor_index.len(),
            "Number of rows in the inverse kinematics (cols of forward kinimatics) should match the number of motors"
        );
        assert!(
            forward_kinematics.len() == inverse_kinematics[0].len(),
            "Number of cols in the inverse kinematics (rows of forward kinimatics) should match the number of control inputs"
        );
        assert!(
            motor_gains.len() == motor_index.len(),
            "Number of motor gains should match the number of motors"
        );
        assert!(
            motor_limits.len() == motor_index.len(),
            "Number of motor limits should match the number of motors"
        );

        let sensors: Vec<Arc<RwLock<EmbeddedDevice>>> = sensor_index
            .into_iter()
            .zip(sensor_buffers.iter())
            .map(|(name, buffer)| {
                Arc::new(RwLock::new(EmbeddedDevice::named(
                    name,
                    *buffer as usize,
                    vec![],
                )))
            })
            .collect();

        let motors: Vec<Arc<RwLock<EmbeddedDevice>>> = motor_index
            .clone()
            .into_iter()
            .zip(motor_can_index.iter())
            .map(|(name, index)| {
                Arc::new(RwLock::new(EmbeddedDevice::named(
                    name,
                    motor_state_len,
                    index.to_vec(),
                )))
            })
            .collect();

        let controllers: Vec<Arc<RwLock<EmbeddedController>>> = controller_types
            .into_iter()
            .flatten()
            .zip(motor_gains.into_iter())
            .zip(motor_limits.into_iter())
            .zip(motor_index.into_iter())
            .map(|(((cont_type, gains), limits), name)| {
                Arc::new(RwLock::new(EmbeddedController::new(
                    name, cont_type, gains, limits,
                )))
            })
            .collect();

        RobotStatus {
            motors: motors,
            sensors: sensors,
            controllers: controllers,
            control_input: Arc::new(RwLock::new(vec![0.0; robot_state_len])),
            control_output: Arc::new(RwLock::new(vec![0.0; robot_state_len])),
            kee_vel_est: Arc::new(RwLock::new(vec![0.0; robot_state_len])),
            imu_vel_est: Arc::new(RwLock::new(vec![0.0; robot_state_len])),
            kee_imu_pos: Arc::new(RwLock::new(vec![0.0; robot_state_len])),
            enc_mag_pos: Arc::new(RwLock::new(vec![0.0; robot_state_len])),
            power_buffer: Arc::new(RwLock::new(0.0)),
            control_mode: Arc::new(RwLock::new(0.0)),
            forward_kinematics: forward_kinematics,
            inverse_kinematics: inverse_kinematics,
        }
    }

    pub fn new(robot_name: &str) -> RobotStatus {
        let byu = BuffYamlUtil::new(robot_name);
        RobotStatus::from_byu(byu)
    }

    pub fn from_self() -> RobotStatus {
        let byu = BuffYamlUtil::from_self();
        RobotStatus::from_byu(byu)
    }

    pub fn clone(&self) -> RobotStatus {
        RobotStatus {
            motors: self.motors.iter().map(|motor| motor.clone()).collect(),
            sensors: self.sensors.iter().map(|sensor| sensor.clone()).collect(),
            controllers: self
                .controllers
                .iter()
                .map(|controller| controller.clone())
                .collect(),
            kee_vel_est: self.kee_vel_est.clone(),
            imu_vel_est: self.imu_vel_est.clone(),
            kee_imu_pos: self.kee_vel_est.clone(),
            enc_mag_pos: self.enc_mag_pos.clone(),
            control_input: self.control_input.clone(),
            control_output: self.control_output.clone(),
            power_buffer: self.power_buffer.clone(),
            control_mode: self.control_mode.clone(),
            forward_kinematics: self.forward_kinematics.clone(),
            inverse_kinematics: self.inverse_kinematics.clone(),
        }
    }

    pub fn motor_init_packet(&mut self) -> Vec<u8> {
        assert!(
            self.motors.len() <= 16,
            "Invalid number of motors, check config"
        );
        vec![255, 0]
            .into_iter()
            .chain(
                self.motors
                    .iter()
                    .map(|motor| motor.read().unwrap().config())
                    .flatten(),
            )
            .collect()
    }

    pub fn motor_control_init_packet(&mut self) -> Vec<Vec<u8>> {
        self.controllers
            .iter()
            .enumerate()
            .map(|(i, controller)| {
                vec![255, 1, i as u8]
                    .into_iter()
                    .chain(controller.read().unwrap().config())
                    .collect()
            })
            .collect()
    }

    pub fn state_control_init_packet(&mut self) -> Vec<Vec<u8>> {
        // packets have a row of forward and inverse kinimatics,
        // Done like this so if you don't want to send all of the rows you don't have to
        (0..self.motors.len())
            .map(|i| {
                vec![255, 2, i as u8]
                    .into_iter()
                    .chain(
                        self.inverse_kinematics[i]
                            .iter()
                            .map(|x| (*x as f32).to_be_bytes())
                            .flatten(),
                    )
                    .chain(
                        self.forward_kinematics
                            .iter()
                            .map(|x| (x[i] as f32).to_be_bytes())
                            .flatten(),
                    )
                    .collect()
            })
            .collect()
    }

    pub fn load_initializers(&mut self) -> Vec<Vec<u8>> {
        let mut initializers = vec![self.motor_init_packet()];

        self.motor_control_init_packet()
            .into_iter()
            .for_each(|packet| initializers.push(packet));

        self.state_control_init_packet()
            .into_iter()
            .for_each(|packet| initializers.push(packet));

        initializers
    }

    pub fn get_reports(&mut self) -> Vec<Vec<u8>> {
        let mut reports = vec![];
        // (0..self.sensors.len()).for_each(|i| reports.push(vec![3, i as u8]));
        // (0..(self.motors.len() / 4) + 1).for_each(|i| reports.push(vec![1, i as u8]));
        (0..(self.controllers.len() / 2) + 1) // motor controller reports
            .for_each(|i| reports.push(vec![2, 1, 0, (2 * i) as u8, ((2 * i) + 1) as u8]));

        reports.push(vec![2, 1, 1]); // vel estimate packet
        reports.push(vec![2, 1, 2]); // pos estimate packet
        reports.push(vec![2, 1, 3]); // control manager info packet

        reports
    }

    pub fn update_motor_encoder(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        if index < self.motors.len() {
            self.motors[index]
                .write()
                .unwrap()
                .update(feedback, timestamp);
        }
    }

    pub fn update_controller(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        if index < self.controllers.len() {
            self.controllers[index]
                .write()
                .unwrap()
                .update(feedback, timestamp);
        }
    }

    pub fn update_sensor(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        self.sensors[index]
            .write()
            .unwrap()
            .update(feedback, timestamp);
    }

    pub fn print(&self) {
        println!("\n\tRobot Status Report:");
        (0..self.motors.len()).for_each(|i| self.motors[i].read().unwrap().print());
        (0..self.sensors.len()).for_each(|i| self.sensors[i].read().unwrap().print());
        (0..self.controllers.len()).for_each(|i| self.controllers[i].read().unwrap().print());
        println!();
    }

    pub fn save(&self) {
        let timestamp = Utc::now();
        let report_filename = format!(
            "report_{}_{}_{}_{}.csv",
            timestamp.year(),
            timestamp.month(),
            timestamp.day(),
            timestamp.hour()
        );
        let project_root =
            env::var("PROJECT_ROOT").expect("The environment variable PROJECT_ROOT is not set");
        let robot_name =
            env::var("ROBOT_NAME").expect("The environment variable ROBOT_NAME is not set");
        let report_dir = format!("{}/data/robot_status_reports/{}", project_root, robot_name);
        let report_path = format!("{}/{}", report_dir, report_filename);

        let report_is_new = !Path::new(&report_path).exists();
        fs::create_dir_all(report_dir)
            .expect("Could not create directory for robot status reports");
        let mut sensor_report_file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(&report_path)
            .expect("Error trying to open or create report file");

        let mut csv_header = String::new();
        let mut csv_data = String::new();
        if report_is_new {
            csv_header.push_str("timestamp,");
        }
        csv_data.push_str(&*format!("{},", timestamp));

        // use maps instead of for (.filter, .map, .for_each, .fold)
        self.sensors.iter().enumerate().for_each(|(i, sensor_arc)| {
            // use less lets
            let sensor_string_vec = sensor_arc.read().unwrap().as_string_vec();
            csv_header.push_str(&*format!("sensor{},", i));
            csv_header.push_str(&*",".repeat(sensor_string_vec.len() - 1));
            csv_data.push_str(&*(sensor_string_vec.join(",") + ","));
        });

        self.motors.iter().enumerate().for_each(|(i, motor_arc)| {
            let motor_string_vec = motor_arc.read().unwrap().as_string_vec();
            csv_header.push_str(&*format!("motor{},", i));
            csv_header.push_str(&*",".repeat(motor_string_vec.len() - 1));
            csv_data.push_str(&*(motor_string_vec.join(",") + ","));
        });

        if report_is_new {
            write!(sensor_report_file, "{}\n", csv_header).expect("Could not write to report file");
        }

        write!(sensor_report_file, "{}\n", csv_data).expect("Could not write to report file");
    }
}
