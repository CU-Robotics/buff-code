use crate::utilities::loaders::*;
use std::time::Instant;

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

    pub fn print(&mut self) {
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

    pub fn print(&mut self) {
        println!(
            "\t\tMotor {} Data:\t{}\n\t\t{:?}",
            self.index,
            self.timestamp.elapsed().as_micros(),
            self.feedback
        );
    }
}

pub struct BuffBotStatusReport {
    pub motors: Vec<BuffBotCANMotorReport>,
    pub sensors: Vec<BuffBotSensorReport>,
}

impl BuffBotStatusReport {
    pub fn default() -> BuffBotStatusReport {
        BuffBotStatusReport {
            motors: vec![],
            sensors: vec![
                BuffBotSensorReport::new(2, vec![0.0; 9]),
                BuffBotSensorReport::new(1, vec![0.0; 7]),
            ],
        }
    }

    pub fn new() -> BuffBotStatusReport {
        BuffBotStatusReport {
            motors: vec![],
            sensors: vec![
                BuffBotSensorReport::new(2, vec![0.0; 9]),
                BuffBotSensorReport::new(1, vec![0.0; 7]),
            ],
        }
    }

    pub fn load_robot() -> BuffBotStatusReport {
        let byu = BuffYamlUtil::from_self();
        let motor_index = byu.load_string_list("motor_index");
        let motor_can_index = byu.load_integer_matrix("motor_can_index");

        let motors = motor_index
            .into_iter()
            .zip(motor_can_index.iter())
            .enumerate()
            .map(|(i, (name, index))| {
                BuffBotCANMotorReport::new(name, i as u8, index[0], index[1], index[2])
            })
            .collect();

        BuffBotStatusReport {
            motors: motors,
            sensors: vec![
                BuffBotSensorReport::new(2, vec![0.0; 9]),
                BuffBotSensorReport::new(1, vec![0.0; 7]),
            ],
        }
    }

    pub fn motor_init_packet(&mut self) -> Vec<u8> {
        assert!(
            self.motors.len() <= 16,
            "Invalid number of motors, check config"
        );
        self.motors
            .iter()
            .map(|motor| motor.init_bytes())
            .flatten()
            .collect()
    }

    pub fn load_initializers(&mut self) -> Vec<Vec<u8>> {
        vec![self.motor_init_packet()]
    }

    pub fn get_reports(&mut self) -> Vec<Vec<u8>> {
        let mut reports = vec![];
        (0..(self.motors.len() / 4) + 1).for_each(|i| reports.push(vec![1, i as u8]));
        (0..self.sensors.len()).for_each(|i| reports.push(vec![3, i as u8]));
        reports
    }

    pub fn update_motor_encoder(&mut self, index: usize, feedback: Vec<f64>) {
        if index < self.motors.len() {
            self.motors[index].write(feedback);
        }
    }

    pub fn update_sensor(&mut self, index: usize, feedback: Vec<f64>) {
        self.sensors[index].write(feedback);
    }

    pub fn print(&mut self) {
        println!("\n\tRobot Status Report:");
        (0..self.motors.len()).for_each(|i| self.motors[i].print());
        (0..self.sensors.len()).for_each(|i| self.sensors[i].print());
        println!();
    }
}
