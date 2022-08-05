
use crate::buff_hid::HidBuffer;
use std::time::Duration;
use std::{sync::{RwLock, Arc}, time::{Instant}, thread::sleep};
use rosrust::{ros_info};
use rosrust_msg::{std_msgs::{Float64MultiArray}, std_msgs};

pub struct SwerveController {
	pub time_stamp: Instant,
	pub output: Arc<RwLock<HidBuffer>>
}

impl SwerveController {
	pub fn new(hid_output: Arc<RwLock<HidBuffer>>) -> SwerveController {

		SwerveController {
			time_stamp: Instant::now(),
			output: hid_output,
		}
	}

	pub fn spin(&mut self){
		self.time_stamp = Instant::now();

		if (self.time_stamp.elapsed().as_millis() as u64) < 1000 {
        	sleep(Duration::from_millis(1000 - self.time_stamp.elapsed().as_millis() as u64));
        }
	}
}