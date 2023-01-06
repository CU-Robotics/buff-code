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

    pub fn device_id(self) -> u8 {
        self.id
    }

    pub fn read(self) -> Vec<f64> {
        self.data
    }

    pub fn write(&mut self, data: Vec<f64>) {
        self.data = data;
        self.timestamp = Instant::now();
    }

    pub fn ts(self) -> Instant {
        self.timestamp
    }
}

pub struct BuffBotCANMotorReport {
    name: String,
    index: u8,
    can_bus: u8,
    motor_type: u8,
    esc_id: u8,
    feedback: Vec<f64>,
    timestamp: Instant,
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

    pub fn name(self) -> String {
        self.name
    }

    pub fn motor_type(self) -> u8 {
        self.motor_type
    }

    pub fn can_bus(self) -> u8 {
        self.can_bus
    }

    pub fn index(self) -> u8 {
        self.index
    }

    pub fn esc_id(self) -> u8 {
        self.esc_id
    }

    pub fn write(&mut self, data: Vec<f64>) {
        self.feedback = data;
        self.timestamp = Instant::now();
    }

    pub fn read(self) -> Vec<f64> {
        self.feedback
    }

    pub fn ts(self) -> Instant {
        self.timestamp
    }

    pub fn init_bytes(&self) -> Vec<u8> {
        vec![self.can_bus, self.motor_type, self.esc_id]
    }
}

pub struct BuffBotStatusReport {
    pub motors: Vec<BuffBotCANMotorReport>,
    pub remote_control: BuffBotSensorReport,
    pub inertial_feedback: BuffBotSensorReport,
}

impl BuffBotStatusReport {
    pub fn default() -> BuffBotStatusReport {
        BuffBotStatusReport {
            motors: vec![],
            remote_control: BuffBotSensorReport::default(),
            inertial_feedback: BuffBotSensorReport::default(),
        }
    }

    pub fn new() -> BuffBotStatusReport {
        BuffBotStatusReport {
            motors: vec![],
            remote_control: BuffBotSensorReport::default(),
            inertial_feedback: BuffBotSensorReport::default(),
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
            remote_control: BuffBotSensorReport::new(1, vec![0.0; 8]),
            inertial_feedback: BuffBotSensorReport::new(2, vec![0.0; 9]),
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

    pub fn update_motor_encoder(&mut self, index: usize, feedback: Vec<f64>) {
        self.motors[index].write(feedback);
    }
}
