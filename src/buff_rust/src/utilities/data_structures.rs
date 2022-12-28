use std::{time::Instant};

pub struct BuffBot_Sensor_Report {
	pub id: i8,
	pub data: Vec<f64>,
	pub timestamp: Instant,
}

impl BuffBot_Sensor_Report {
	pub fn default() -> BuffBot_Sensor_Report {
		BuffBot_Sensor_Report {
			id : -1,
			data : vec![0; 24],
			timestamp : Instant::now(),
		}
	}

	pub fn new(id: i8, data: Vec<f64>) -> BuffBot_Sensor_Report {
		BuffBot_Sensor_Report {
			id : id,
			data : data,
			timestamp : Instant::now(),
		}
	}

	pub fn device_id(self) -> i8 {
		self.id
	}

	pub fn read(self) -> Vec<f64> {
		self.data
	}

	pub fn write(self, data: Vec<f64>) {
		self.data = data;
		timestamp = Instant::now();
	}

	pub fn ts(self) -> Instant {
		self.timestamp
	}
}

pub struct BuffBot_CAN_Motor_Report {
	name: String,
	index: i8,
	can_bus: i8,
	motor_type: i8,
	esc_id: i8,
	feedback: Vec<f64>,
	timestamp: Instant,
}

impl BuffBot_CAN_Motor_Report {
	pub fn default() -> BuffBot_CAN_Motor_Report {
		BuffBot_CAN_Motor_Report {
			name : "Mort Motorson",
			index : -1,
			can_bus : 0,
			motor_type : -1,
			esc_id : -1,
			feedback: vec![0; 3],
			timestamp : Instant::now(),
		}
	}

	pub fn new(name: String, index: i8, can_bus: i8, motor_type: i8, esc_id: i8) -> BuffBot_CAN_Motor_Report {
		BuffBot_CAN_Motor_Report {
			name : name,
			index : index,
			can_bus : can_bus,
			motor_type : motor_type,
			esc_id : esc_id,
			feedback: vec![0; 3],
			timestamp : Instant::now(),
		}
	}

	pub fn name(self) -> String {
		self.name
	}

	pub fn index(self) -> i8 {
		self.index
	}

	pub fn motor_type(self) -> i8 {
		self.motor_type
	}

	pub fn can_bus(self) -> i8 {
		self.can_bus
	}

	pub fn index(self) -> i8 {
		self.index
	}

	pub fn write(self, data: Vec<f64>) {
		self.feedback = data;
		timestamp = Instant::now();
	}

	pub fn read(self) -> Vec<f64> {
		self.feedback
	}

	pub fn ts(self) -> Instant {
		self.timestamp
	}
}

pub struct BuffBot_Status_Report {
	pub motors: Vec<BuffBot_CAN_Motor_Report>,
	pub remote_control: BuffBot_Sensor_Report,
	pub inertial_feedback: BuffBot_Sensor_Report,
}


impl BuffBot_Status_Report {
	pub fn default() -> BuffBot_Status_Report {
		BuffBot_Status_Report {
			motors: vec![],
			remote_control: BuffBot_Sensor_Report::default(),
			inertial_feedback: BuffBot_Sensor_Report::default(),
		}
	}

	pub fn new() -> BuffBot_Status_Report {
		BuffBot_Status_Report {
			motors: vec![],
			remote_control: BuffBot_Sensor_Report::default(),
			inertial_feedback: BuffBot_Sensor_Report::default(),
		}
	}

	pub fn n_motors(self) -> usize {
		self.motors.len()
	}

	pub fn get_motor(self, index: i8) -> &BuffBot_CAN_Motor_Report {
		self.motors.iter().find(|&motor| motor == index).unwrap()
	}

	pub fn add_motor(name: String, index: i8, can_bus: i8, motor_type: i8, esc_id: i8) {
		self.motors.push(BuffBot_CAN_Motor_Report::new(name, index, can_bus, motor_type, esc_id));
	}

	pub fn update_motor_encoder(index: i8, feedback: Vec<f64>) {
		self.motors[index]
	}
}




