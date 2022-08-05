
extern crate hidapi;

use std::thread::sleep;
use std::time::Duration;
use hidapi::{HidApi, HidError, HidDevice};
use std::{sync::{RwLock, Arc}, time::{Instant}};
use rosrust::{ros_info};
use rosrust_msg::{std_msgs::{Float64MultiArray}, std_msgs};

pub struct HidBuffer {
    pub data: [u8; 64],
    pub seek_ptr: usize,
    pub update_flag: bool,
    pub time_stamp: Instant,
}

impl HidBuffer {
    pub fn new() -> HidBuffer {
        HidBuffer { 
        	data: [0; 64], 
        	seek_ptr: 0,
        	update_flag: false,
        	time_stamp: Instant::now() 
        }
    }

    pub fn seek(&mut self, set: Option<usize>) -> u8 {
    	self.seek_ptr = set.unwrap_or(self.seek_ptr);
    	let tmp = self.data[self.seek_ptr];

    	if self.seek_ptr >= 63{
    		self.seek_ptr = 0;
    	}
    	else{
	    	self.seek_ptr += 1;
	    }

	    tmp
    }

    pub fn seek_u16(&mut self, set: Option<usize>) -> u16 {
    	self.seek_ptr = set.unwrap_or(self.seek_ptr);

    	u16::from_be_bytes([self.seek(None), self.seek(None)])
    }

    pub fn seek_f32(&mut self, set: Option<usize>) -> f32 {
    	self.seek_ptr = set.unwrap_or(self.seek_ptr);

    	f32::from_le_bytes([self.seek(None), self.seek(None), self.seek(None), self.seek(None)])
    }

    pub fn unseek(&mut self) -> u8 {
    	if self.seek_ptr <= 0{
    		self.seek_ptr = 63;
    	}
    	else{
	    	self.seek_ptr -= 1;
	    }

    	self.data[self.seek_ptr]
    }

    pub fn put(&mut self, value: u8) {
    	self.data[self.seek_ptr] = value;
    	if self.seek_ptr < 63 {
	    	self.seek_ptr += 1;
    	}
    	else {
    		self.seek_ptr = 0;
    	}
    }

    pub fn reset(&mut self){
    	self.data = [0u8; 64];
    	self.seek_ptr = 0;
    	self.update_flag = false;
    }

    pub fn print_buffer(&self){
	    let mut data_string: String = String::new();

	    let mut i = 0;

	    for u in self.data {
	        data_string.push_str(&(u.to_string() + "\t"));

	        if (i + 1) % 16 == 0 && i != 0{
	            data_string.push_str("\n");
	        }
	        i += 1;
	    }
	    data_string.push_str("\n==========\n");
	    println!("{}", data_string);

	}
}

pub struct HidLayer {
	pub hidapi: HidApi,
	pub vid: u16,
	pub pid: u16,
	pub input: HidBuffer,
	pub output: HidBuffer,
	pub dr16_input: Arc<RwLock<[u8; 18]>>,
	pub teensy: Result<HidDevice, HidError>,
	pub output_queue: Arc<RwLock<HidBuffer>>,
	pub subscriber: rosrust::Subscriber,
	pub publishers: Vec<rosrust::Publisher<Float64MultiArray>>,
}

impl HidLayer {
	pub fn new() -> HidLayer {

    	let mut publishers = Vec::<rosrust::Publisher<Float64MultiArray>>::new();

    	let io_topics = rosrust::param("/buffbot/DEVICES/LUT").unwrap().get::<Vec<String>>().unwrap();

    	for topic in io_topics {
    		let topic_prefix = "/buffbot/TOPICS/".to_string();
    		let topic = rosrust::param(&(topic_prefix + &topic)).unwrap().get::<String>().unwrap();
    		let pubber = rosrust::publish::<Float64MultiArray>(&topic, 1).unwrap();
    		publishers.push(pubber);
    	}


    	let output_buffer = Arc::new(RwLock::new(HidBuffer::new()));

    	let buffer = Arc::clone(&output_buffer);
    	let writer_id = "/buffbot/TOPICS/HID_OUTPUT".to_string();
    	let writer_topic = rosrust::param(&writer_id).unwrap().get::<String>().unwrap();

		let sub = rosrust::subscribe(&writer_topic, 5, move |msg: std_msgs::String| {
			let mut w = buffer.write().unwrap();
			if w.data.len() - w.seek_ptr < msg.data.bytes().len() {
				for c in msg.data.bytes() {
					w.put(c as u8);		
				}
			}	
		}).unwrap();

		let dr16 = Arc::new(RwLock::new([0u8; 18]));

		let api = HidApi::new().expect("Failed to create API instance");
		let teensy = api.open(0x0000, 0x0000);

		HidLayer {
			hidapi: api,
			vid: 0x16C0,
			pid: 0x0486,
			teensy: teensy,
			dr16_input: dr16,
			input: HidBuffer::new(),
			output: HidBuffer::new(),
			output_queue: output_buffer,
			subscriber: sub,
			publishers: publishers,
		}
	}

	pub fn reset(&mut self){
		self.input.reset();
		self.output.reset();
		self.output_queue.write().unwrap().reset();
	}

	pub fn dump_config(&mut self){
		self.reset();
		let devices = rosrust::param("/buffbot/DEVICES/LUT").unwrap().get::<Vec<String>>().unwrap();

		for (i, device) in devices.iter().enumerate() {

			let type_param_id = format!("/buffbot/DEVICES/{}/TYPE", device);
			let dtype = rosrust::param(&type_param_id).unwrap().get::<String>().unwrap();

			let mut queue = self.output_queue.write().unwrap();
			queue.put('X' as u8);
			queue.put('X' as u8);
			if dtype == "motor" {
				let canbus_param_id = format!("/buffbot/DEVICES/{}/CANBUS", device);
				let motorid_param_id = format!("/buffbot/DEVICES/{}/MOTORID", device);

				let canid = rosrust::param(&canbus_param_id).unwrap().get::<u8>().unwrap();
				let motorid = rosrust::param(&motorid_param_id).unwrap().get::<u8>().unwrap();

				queue.put('M' as u8);
				queue.put(i as u8);
				queue.put(canid);
				queue.put(motorid);
			}
			else if dtype == "imu" {
				queue.put('I' as u8);
				queue.put(i as u8);
				queue.put(1);

			} else if dtype == "dr16"{
				queue.put('D' as u8);
				queue.put(i as u8);
			}

		}
		self.output_queue.write().unwrap().print_buffer();
	}

	pub fn read_bytes_as_bytes(&mut self, n_bytes: u8) -> Vec<f64> {
		let mut j = 0;
		let mut data = Vec::<f64>::new();


		while j < n_bytes {
			data.push(self.input.seek(None) as f64);
			j += 1;
		}
		data
	}

	pub fn read_bytes_as_u16(&mut self, n_bytes: u8) -> Vec<f64> {
		let mut j = 0;
		let mut data = Vec::<f64>::new();


		while j <= n_bytes - 2 {
			data.push(self.input.seek_u16(None) as f64);
			j += 2;
		}
		data
	}


	pub fn read_bytes_as_f32(&mut self, n_bytes: u8) -> Vec<f64> {
		let mut j = 0;
		let mut data = Vec::<f64>::new();


		while j < n_bytes - 3 {
			data.push(self.input.seek_f32(None) as f64);
			j += 3;
		}
		data
	}	

	pub fn parse_hid(&mut self, n: usize) {
		self.input.seek_ptr = 0;
		if n == 0 {
			return;
		}
		while self.input.seek_ptr < n - 1 {
			if self.input.seek(None) == 'X' as u8 {
				if self.input.seek(None) == 'X' as u8 {

					let data_bytes = self.input.seek(None);
					let data_id = self.input.seek(None) as usize;
					let data_type = self.input.seek(None) as usize;

					let data: Vec<f64>;

					match data_type {
						0 => {
							data = self.read_bytes_as_bytes(data_bytes);
						}
						1 => {
							data = self.read_bytes_as_u16(data_bytes);
						}
						2 => {
							data = self.read_bytes_as_f32(data_bytes);
						}
						_ => {
							continue;
						}
					}
					let mut msg = Float64MultiArray::default();
					msg.data = data;
					let _result = self.publishers[data_id].send(msg).unwrap();
					continue;
				}
			}
		}
		self.input.reset();
	}

	pub fn read(&mut self){
		let mut n = 0;
		match &self.teensy {
			Ok(dev) => {
				match dev.read(&mut self.input.data) {
					Ok(value) => {
						n = value
					}
					_ => { 
						n = 0;
						self.connection_repair();
					}
				}
			}
			_ => {self.connection_repair()}
		}

		self.input.time_stamp = Instant::now();
		// self.input.print_buffer();
		self.parse_hid(n);			
	}

	pub fn write(&mut self){
		let mut queue = self.output_queue.write().unwrap();
		self.output.reset();
		self.output.data = queue.data;

		match &self.teensy {
			Ok(dev) => {
				dev.write(&self.output.data);
			}
			_ => {
				drop(queue);
				self.connection_repair();
				return;
			}
		}
		
		queue.reset();
		self.output.reset();
	}

	pub fn init_comms(&mut self) {

		self.teensy = self.hidapi.open(self.vid, self.pid);
		match &self.teensy {
			Ok(_) => {
				self.dump_config();
				let param_id = format!("/buffbot/HID_ACTIVE");
				rosrust::param(&param_id).unwrap().set::<i32>(&1).unwrap();
			}
			_ => {
				let param_id = format!("/buffbot/HID_ACTIVE");
				rosrust::param(&param_id).unwrap().set::<i32>(&0).unwrap();
			}
		}
	
	}

	pub fn connection_repair(&mut self) {
		println!("Can't find teensy ...");
        if (self.input.time_stamp.elapsed().as_millis() as u64) < 9500 {
			sleep(Duration::from_millis(10000 - self.input.time_stamp.elapsed().as_millis() as u64));
			self.input.time_stamp = Instant::now();
		}
		self.init_comms();
	}

	pub fn spin(&mut self){

		self.init_comms();

		let mut timestamp = Instant::now();

		while rosrust::is_ok() {
			timestamp = Instant::now();

        	self.read();
			self.write();
			let micros = timestamp.elapsed().as_micros() as u64;

        	if micros < 1000 {
        		sleep(Duration::from_micros(1000 - micros));
        	}
		}
	}
}

