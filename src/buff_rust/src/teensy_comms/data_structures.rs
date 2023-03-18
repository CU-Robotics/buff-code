use crate::utilities::loaders::*;
use chrono::prelude::*;
use std::env;
use std::fs;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::Path;
use std::sync::{Arc, RwLock};

pub struct BuffBotSensorReport {
    pub id: u8,
    pub data: Vec<f64>,
    pub timestamp: f64,
}

impl BuffBotSensorReport {
    pub fn default() -> BuffBotSensorReport {
        BuffBotSensorReport {
            id: u8::MAX,
            data: vec![0.0; 24],
            timestamp: 0.0,
        }
    }

    pub fn new(id: u8, data: Vec<f64>) -> BuffBotSensorReport {
        BuffBotSensorReport {
            id: id,
            data: data,
            timestamp: 0.0,
        }
    }

    pub fn write(&mut self, data: Vec<f64>, time: f64) {
        self.data = data[0..self.data.len()].to_vec();
        self.timestamp = time;
    }

    pub fn print(&self) {
        println!(
            "\t\tSensor {} Data:\t{}\n\t\t{:?}",
            self.id, self.timestamp, self.data
        );
    }

    pub fn as_string_vec(&self) -> Vec<String> {
        let mut result = vec![self.timestamp.to_string()];
        for i in &self.data {
            result.push(i.to_string());
        }
        return result;
    }
}

pub struct BuffBotCANMotorReport {
    pub name: String,
    pub index: u8,
    pub can_bus: u8,
    pub motor_type: u8,
    pub esc_id: u8,
    pub feedback: Vec<f64>,
    pub timestamp: f64,
}

impl BuffBotCANMotorReport {
    pub fn default() -> BuffBotCANMotorReport {
        BuffBotCANMotorReport {
            name: "Mort Motorson".to_string(),
            index: u8::MAX,
            can_bus: 0,
            motor_type: u8::MAX,
            esc_id: u8::MAX,
            feedback: vec![0.0; 3],
            timestamp: 0.0,
        }
    }

    pub fn new(
        name: String,
        index: u8,
        can_bus: u8,
        motor_type: u8,
        esc_id: u8,
    ) -> BuffBotCANMotorReport {
        BuffBotCANMotorReport {
            name: name,
            index: index,
            can_bus: can_bus,
            motor_type: motor_type,
            esc_id: esc_id,
            feedback: vec![0.0; 3],
            timestamp: 0.0,
        }
    }

    pub fn write(&mut self, data: Vec<f64>, time: f64) {
        self.feedback = data;
        self.timestamp = time;
    }

    pub fn init_bytes(&self) -> Vec<u8> {
        vec![self.can_bus, self.motor_type, self.esc_id]
    }

    pub fn print(&self) {
        println!(
            "\t\tMotor {} Data:\t{}\n\t\t{:?}",
            self.index, self.timestamp, self.feedback
        );
    }

    pub fn as_string_vec(&self) -> Vec<String> {
        let mut result: Vec<String> = vec![self.index.to_string(), self.timestamp.to_string()];
        for i in &self.feedback {
            result.push(i.to_string());
        }
        return result;
    }
}

pub struct BuffBotControllerReport {
    pub gains: Vec<f64>,
    pub limits: Vec<f64>,
    pub feedback: Vec<f64>,
    pub reference: Vec<f64>,

    pub output: f64,
    pub control_type: u8,

    pub timestamp: f64,
}

impl BuffBotControllerReport {
    pub fn default() -> BuffBotControllerReport {
        BuffBotControllerReport {
            gains: vec![0.0; 3],
            limits: vec![0.0; 4],
            reference: vec![0.0; 2],
            feedback: vec![0.0; 3],
            output: 0.0,
            control_type: 0,
            timestamp: 0.0,
        }
    }

    pub fn new(control_type: u8, gains: Vec<f64>, limits: Vec<f64>) -> BuffBotControllerReport {
        BuffBotControllerReport {
            gains: gains,
            limits: limits,
            reference: vec![0.0; 2],
            feedback: vec![0.0; 3],
            output: 0.0,
            control_type: control_type,
            timestamp: 0.0,
        }
    }

    pub fn init_bytes(&self) -> Vec<u8> {
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

    pub fn write(&mut self, feedback: Vec<f64>, time: f64) {
        self.output = feedback[0];
        self.reference = feedback[1..3].to_vec();
        self.feedback = feedback[3..].to_vec();
        self.timestamp = time;
    }

    pub fn print(&self, id: usize) {
        println!("Controller {}\t\n\tReference\tFeedback\tOutput\n\t{:?} {:?} {}\n\tGains\t\tLimits\n\t{:?} {:?}", id, self.reference, self.feedback, self.output, self.gains, self.limits);
    }
}

pub struct BuffBotStatusReport {
    pub motors: Vec<Arc<RwLock<BuffBotCANMotorReport>>>,
    pub sensors: Vec<Arc<RwLock<BuffBotSensorReport>>>,
    pub controllers: Vec<Arc<RwLock<BuffBotControllerReport>>>,
    pub control_input: Arc<RwLock<Vec<f64>>>,
    pub kee_vel_est: Arc<RwLock<Vec<f64>>>,
    pub imu_vel_est: Arc<RwLock<Vec<f64>>>,
    pub kee_imu_pos: Arc<RwLock<Vec<f64>>>,
    pub enc_mag_pos: Arc<RwLock<Vec<f64>>>,
    pub power_buffer: Arc<RwLock<f64>>,
    pub control_mode: Arc<RwLock<f64>>,

    pub forward_kinematics: Vec<Vec<f64>>,
    pub inverse_kinematics: Vec<Vec<f64>>,
}

impl BuffBotStatusReport {
    pub fn default() -> BuffBotStatusReport {
        BuffBotStatusReport {
            motors: vec![],
            sensors: vec![
                Arc::new(RwLock::new(BuffBotSensorReport::new(2, vec![0.0; 9]))),
                Arc::new(RwLock::new(BuffBotSensorReport::new(1, vec![0.0; 7]))),
            ],
            controllers: vec![],
            control_input: Arc::new(RwLock::new(vec![0.0; 7])),
            kee_vel_est: Arc::new(RwLock::new(vec![0.0; 7])),
            imu_vel_est: Arc::new(RwLock::new(vec![0.0; 7])),
            kee_imu_pos: Arc::new(RwLock::new(vec![0.0; 7])),
            enc_mag_pos: Arc::new(RwLock::new(vec![0.0; 7])),
            power_buffer: Arc::new(RwLock::new(0.0)),
            control_mode: Arc::new(RwLock::new(0.0)),
            forward_kinematics: vec![vec![]],
            inverse_kinematics: vec![vec![]],
        }
    }

    pub fn from_byu(byu: BuffYamlUtil) -> BuffBotStatusReport {
        let motor_index = byu.load_string_list("motor_index");
        let motor_gains = byu.load_float_matrix("motor_gains");
        let motor_limits = byu.load_float_matrix("motor_limits");
        let motor_can_index = byu.load_integer_matrix("motor_can_index");
        let controller_types = byu.load_integer_matrix("motor_controller_types");
        let forward_kinematics = byu.load_float_matrix("forward_kinematics");
        let inverse_kinematics = byu.load_float_matrix("inverse_kinematics");

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
            inverse_kinematics[0].len() == 7    // INPUT_CONTROL_LEN, maybe correlate these
                && forward_kinematics.len() == inverse_kinematics[0].len(),
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

        let controllers: Vec<Arc<RwLock<BuffBotControllerReport>>> = controller_types
            .into_iter()
            .flatten()
            .zip(motor_gains.into_iter())
            .zip(motor_limits.into_iter())
            .map(|((cont_type, gains), limits)| {
                Arc::new(RwLock::new(BuffBotControllerReport::new(
                    cont_type, gains, limits,
                )))
            })
            .collect();

        let motors: Vec<Arc<RwLock<BuffBotCANMotorReport>>> = motor_index
            .into_iter()
            .zip(motor_can_index.iter())
            .enumerate()
            .map(|(i, (name, index))| {
                Arc::new(RwLock::new(BuffBotCANMotorReport::new(
                    name, i as u8, index[0], index[1], index[2],
                )))
            })
            .collect();

        BuffBotStatusReport {
            motors: motors,
            sensors: vec![
                Arc::new(RwLock::new(BuffBotSensorReport::new(2, vec![0.0; 9]))),
                Arc::new(RwLock::new(BuffBotSensorReport::new(1, vec![0.0; 6]))),
            ],
            controllers: controllers,
            control_input: Arc::new(RwLock::new(vec![0.0; 7])),
            kee_vel_est: Arc::new(RwLock::new(vec![0.0; 7])),
            imu_vel_est: Arc::new(RwLock::new(vec![0.0; 7])),
            kee_imu_pos: Arc::new(RwLock::new(vec![0.0; 7])),
            enc_mag_pos: Arc::new(RwLock::new(vec![0.0; 7])),
            power_buffer: Arc::new(RwLock::new(0.0)),
            control_mode: Arc::new(RwLock::new(0.0)),
            forward_kinematics: forward_kinematics,
            inverse_kinematics: inverse_kinematics,
        }
    }

    pub fn new(robot_name: &str) -> BuffBotStatusReport {
        let byu = BuffYamlUtil::new(robot_name);
        BuffBotStatusReport::from_byu(byu)
    }

    pub fn from_self() -> BuffBotStatusReport {
        let byu = BuffYamlUtil::from_self();
        BuffBotStatusReport::from_byu(byu)
    }

    pub fn clone(&self) -> BuffBotStatusReport {
        BuffBotStatusReport {
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
                    .map(|motor| motor.read().unwrap().init_bytes())
                    .flatten(),
            )
            .collect()
    }

    pub fn motor_control_init_packet(&mut self) -> Vec<Vec<u8>> {
        assert!(
            self.controllers.len() <= self.motors.len(),
            "Invalid number of controllers, check config"
        );
        self.controllers
            .iter()
            .enumerate()
            .map(|(i, controller)| {
                vec![255, 1, i as u8]
                    .into_iter()
                    .chain(controller.read().unwrap().init_bytes())
                    .collect()
            })
            .collect()
    }

    pub fn state_control_init_packet(&mut self) -> Vec<Vec<u8>> {
        assert!(
            self.inverse_kinematics.len() == self.motors.len(),
            "Invalid number of parameters, check config"
        );
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
                .write(feedback, timestamp);
        }
    }

    pub fn update_controller(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        if index < self.controllers.len() {
            self.controllers[index]
                .write()
                .unwrap()
                .write(feedback, timestamp);
        }
    }

    pub fn update_sensor(&mut self, index: usize, feedback: Vec<f64>, timestamp: f64) {
        self.sensors[index]
            .write()
            .unwrap()
            .write(feedback, timestamp);
    }

    pub fn print(&self) {
        println!("\n\tRobot Status Report:");
        (0..self.motors.len()).for_each(|i| self.motors[i].read().unwrap().print());
        (0..self.sensors.len()).for_each(|i| self.sensors[i].read().unwrap().print());
        (0..self.controllers.len()).for_each(|i| self.controllers[i].read().unwrap().print(i));
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
