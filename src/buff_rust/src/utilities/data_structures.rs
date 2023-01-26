use std::sync::{Arc, RwLock};
use std::time::Instant;

use crate::utilities::loaders::*;

pub struct BuffBotSensorReport {
    pub id: u8,
    pub data: Vec<f64>,
    pub timestamp: Instant,
}

impl BuffBotSensorReport {
    pub fn default() -> BuffBotSensorReport {
        BuffBotSensorReport {
            id: u8::MAX,
            data: vec![0.0; 24],
            timestamp: Instant::now(),
        }
    }

    pub fn new(id: u8, data: Vec<f64>) -> BuffBotSensorReport {
        BuffBotSensorReport {
            id: id,
            data: data,
            timestamp: Instant::now(),
        }
    }

    pub fn write(&mut self, data: Vec<f64>) {
        self.data = data[0..self.data.len()].to_vec();
        self.timestamp = Instant::now();
    }

    pub fn print(&self) {
        println!(
            "\t\tSensor {} Data:\t{}\n\t\t{:?}",
            self.id,
            self.timestamp.elapsed().as_micros(),
            self.data
        );
    }
}

pub struct BuffBotCANMotorReport {
    pub name: String,
    pub index: u8,
    pub can_bus: u8,
    pub motor_type: u8,
    pub esc_id: u8,
    pub feedback: Vec<f64>,
    pub timestamp: Instant,
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
            timestamp: Instant::now(),
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
            timestamp: Instant::now(),
        }
    }

    pub fn write(&mut self, data: Vec<f64>) {
        self.feedback = data;
        self.timestamp = Instant::now();
    }

    pub fn init_bytes(&self) -> Vec<u8> {
        vec![self.can_bus, self.motor_type, self.esc_id]
    }

    pub fn print(&self) {
        println!(
            "\t\tMotor {} Data:\t{}\n\t\t{:?}",
            self.index,
            self.timestamp.elapsed().as_micros(),
            self.feedback
        );
    }
}

pub struct BuffBotControllerReport {
    pub gains: Vec<f64>,
    pub limits: Vec<f64>,

    pub output: f64,
    pub reference: Vec<f64>,
    pub timestamp: Instant,
}

impl BuffBotControllerReport {
    pub fn default() -> BuffBotControllerReport {
        BuffBotControllerReport {
            gains: vec![0.0; 3],
            limits: vec![0.0; 4],
            output: 0.0,
            reference: vec![0.0; 2],
            timestamp: Instant::now(),
        }
    }

    pub fn new(gains: Vec<f64>, limits: Vec<f64>) -> BuffBotControllerReport {
        BuffBotControllerReport {
            gains: gains,
            limits: limits,
            output: 0.0,
            reference: vec![0.0; 2],
            timestamp: Instant::now(),
        }
    }

    pub fn init_bytes(&self) -> Vec<u8> {
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
            .collect()
    }

    pub fn write(&mut self, feedback: Vec<f64>) {
        self.output = feedback[0];
        self.reference = feedback[1..].to_vec();
    }
}

pub struct BuffBotStatusReport {
    pub motors: Vec<Arc<RwLock<BuffBotCANMotorReport>>>,
    pub sensors: Vec<Arc<RwLock<BuffBotSensorReport>>>,
    pub controllers: Vec<Arc<RwLock<BuffBotControllerReport>>>,
    pub chassis_inverse_kinematics: Vec<Vec<f64>>,
}

impl BuffBotStatusReport {
    pub fn default() -> BuffBotStatusReport {
        BuffBotStatusReport {
            motors: vec![],
            sensors: vec![
                Arc::new(RwLock::new(BuffBotSensorReport::new(2, vec![0.0; 9]))),
                Arc::new(RwLock::new(BuffBotSensorReport::new(1, vec![0.0; 6]))),
            ],
            controllers: vec![],
            chassis_inverse_kinematics: vec![vec![]],
        }
    }

    pub fn from_byu(byu: BuffYamlUtil) -> BuffBotStatusReport {
        let motor_index = byu.load_string_list("motor_index");
        let motor_can_index = byu.load_integer_matrix("motor_can_index");
        let motor_gains = byu.load_float_matrix("motor_gains");
        let motor_limits = byu.load_float_matrix("motor_limits");

        let chassis_inverse_kinematics = byu.load_float_matrix("chassis_inverse_kinematics");

        let controllers: Vec<Arc<RwLock<BuffBotControllerReport>>> = motor_gains
            .into_iter()
            .zip(motor_limits.into_iter())
            .map(|(gains, limits)| {
                Arc::new(RwLock::new(BuffBotControllerReport::new(gains, limits)))
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

        assert!(
            controllers.len() == motors.len(),
            "Number of Controllers and Motors should match"
        );
        assert!(
            chassis_inverse_kinematics.len() == motors.len(),
            "Number of rows in the chassis kinematics should match the number of motors"
        );

        BuffBotStatusReport {
            motors: motors,
            sensors: vec![
                Arc::new(RwLock::new(BuffBotSensorReport::new(2, vec![0.0; 9]))),
                Arc::new(RwLock::new(BuffBotSensorReport::new(1, vec![0.0; 6]))),
            ],
            controllers: controllers,
            chassis_inverse_kinematics: chassis_inverse_kinematics,
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
            chassis_inverse_kinematics: self.chassis_inverse_kinematics.clone(),
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
            self.chassis_inverse_kinematics.len() == self.motors.len(),
            "Invalid number of parameters, check config"
        );
        self.chassis_inverse_kinematics
            .chunks(2)
            .enumerate()
            .map(|(i, block)| {
                vec![255, 2, i as u8]
                    .into_iter()
                    .chain(
                        block
                            .iter()
                            .map(|block| block.iter().map(|x| (*x as f32).to_be_bytes()).flatten())
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
        (0..self.sensors.len()).for_each(|i| reports.push(vec![3, i as u8]));
        (0..(self.motors.len() / 4) + 1).for_each(|i| reports.push(vec![1, i as u8]));
        (0..(self.controllers.len() / 4) + 1).for_each(|i| reports.push(vec![2, 1, i as u8]));
        reports
    }

    pub fn update_motor_encoder(&mut self, index: usize, feedback: Vec<f64>) {
        if index < self.motors.len() {
            self.motors[index].write().unwrap().write(feedback);
        }
    }

    pub fn update_controller(&mut self, index: usize, feedback: Vec<f64>) {
        if index < self.controllers.len() {
            self.controllers[index].write().unwrap().write(feedback);
        }
    }

    pub fn update_sensor(&mut self, index: usize, feedback: Vec<f64>) {
        self.sensors[index].write().unwrap().write(feedback);
    }

    pub fn print(&self) {
        println!("\n\tRobot Status Report:");
        (0..self.motors.len()).for_each(|i| self.motors[i].read().unwrap().print());
        (0..self.sensors.len()).for_each(|i| self.sensors[i].read().unwrap().print());
        println!();
    }
}
