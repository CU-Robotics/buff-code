extern crate hidapi;

use hidapi::{HidApi, HidDevice, HidError};
// use rosrust::ros_info;
use crate::buff_rust::buff_utils::*;
use rosrust_msg::std_msgs;
use std::thread::sleep;
use std::time::Duration;
use std::{
    sync::{Arc, RwLock},
    time::Instant,
};

pub struct HidBuffer {
    pub data: [u8; 64],
    pub seek_ptr: usize,
    pub update_flag: bool,
    pub timestamp: Instant,
}

impl HidBuffer {
    pub fn new() -> HidBuffer {
        HidBuffer {
            data: [0; 64],
            seek_ptr: 0,
            update_flag: false,
            timestamp: Instant::now(),
        }
    }

    pub fn seek(&mut self, set: Option<usize>) -> u8 {
        self.seek_ptr = set.unwrap_or(self.seek_ptr);
        let tmp = self.data[self.seek_ptr];

        if self.seek_ptr >= 63 {
            self.seek_ptr = 0;
        } else {
            self.seek_ptr += 1;
        }

        tmp
    }

    pub fn put(&mut self, value: u8) {
        self.data[self.seek_ptr] = value;
        if self.seek_ptr < 63 {
            self.seek_ptr += 1;
        } else {
            self.seek_ptr = 0;
        }
    }

    pub fn puts(&mut self, data: Vec<u8>) {
        for d in data {
            self.put(d);
        }
    }

    pub fn check_of(&self, n: usize) -> bool {
        if 64 - self.seek_ptr > n {
            return false;
        }
        true
    }

    pub fn reset(&mut self) {
        self.data = [0u8; 64];
        self.seek_ptr = 0;
        self.update_flag = false;
        self.timestamp = Instant::now();
    }

    pub fn print_buffer(&self) {
        let mut data_string: String = String::new();

        let mut i = 0;

        for u in self.data {
            data_string.push_str(&(u.to_string() + "\t"));

            if (i + 1) % 16 == 0 && i != 0 {
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
    pub nc_timeout: u128,
    pub input: HidBuffer,
    pub output: HidBuffer,
    pub teensy: Result<HidDevice, HidError>,
    pub output_queue: Arc<RwLock<HidBuffer>>,
    pub subscriber: rosrust::Subscriber,
    pub publishers: Vec<rosrust::Publisher<std_msgs::UInt8MultiArray>>,
    pub timestamp: Instant,
}

impl HidLayer {
    pub fn new() -> HidLayer {
        let output_buffer = Arc::new(RwLock::new(HidBuffer::new()));

        let buffer = Arc::clone(&output_buffer);
        let can_topic = "can_output";

        let can_sub = rosrust::subscribe(&can_topic, 1, move |msg: std_msgs::UInt8MultiArray| {
            let mut w = buffer.write().unwrap();
            w.seek_ptr = 0;
            msg.data.iter().for_each(|c| {
                w.put(*c);
            });
        })
        .unwrap();

        let byu = BuffYamlUtil::default();
        let sensors = byu.load_string_list("sensor_index");

        println!("{:?}", sensors);

        let mut pubs: Vec<rosrust::Publisher<std_msgs::UInt8MultiArray>> = sensors
            .iter()
            .map(|s| rosrust::publish(format!("{}_raw", s).as_str(), 1).unwrap())
            .collect();

        pubs.splice(0..0, rosrust::publish("can_raw", 1));

        let api = HidApi::new().expect("Failed to create API instance");
        let teensy = api.open(0x0000, 0x0000);

        HidLayer {
            hidapi: api,
            nc_timeout: 8000,
            vid: 0x16C0,
            pid: 0x0486,
            teensy: teensy,
            input: HidBuffer::new(),
            output: HidBuffer::new(),
            output_queue: output_buffer,
            subscriber: can_sub,
            publishers: pubs,
            timestamp: Instant::now(),
        }
    }

    pub fn reset(&mut self) {
        self.input.reset();
        self.output.reset();
        self.output_queue.write().unwrap().reset();
    }

    pub fn read_bytes(&mut self, n_bytes: u8) -> Vec<u8> {
        vec![0u8; n_bytes as usize]
            .iter()
            .map(|_| self.input.seek(None))
            .collect()
    }

    pub fn parse_hid(&mut self, n: usize) {
        if n == 0 {
            return;
        }

        self.input.seek_ptr = 0;

        {
            let mut msg = std_msgs::UInt8MultiArray::default();
            msg.data = self.read_bytes(33);
            self.publishers[0].send(msg).unwrap();
        }

        {
            self.input.seek_ptr = 34;

            let sensor_id = self.input.seek(None);

            if sensor_id == 0 {
                self.input.reset();
                return;
            }

            let mut msg = std_msgs::UInt8MultiArray::default();
            msg.data = self.read_bytes(24);

            self.publishers[sensor_id as usize].send(msg).unwrap();
        }

        self.input.reset();
    }

    pub fn read(&mut self) {
        let mut n = 0;

        match &self.teensy {
            Ok(dev) => match dev.read(&mut self.input.data) {
                Ok(value) => {
                    n = value;
                    self.input.timestamp = Instant::now();
                }
                _ => {
                    n = 0;
                    self.connection_repair();
                }
            },
            _ => self.connection_repair(),
        }

        // self.input.print_buffer();
        self.parse_hid(n);
    }

    pub fn write(&mut self) {
        self.output.reset();
        {
            let mut queue = self.output_queue.write().unwrap();
            self.output.data = queue.data;
            queue.reset();
        }

        match &self.teensy {
            Ok(dev) => match dev.write(&self.output.data) {
                Ok(_) => self.output.timestamp = Instant::now(),
                Err(_) => self.connection_repair(),
            },
            _ => {
                self.connection_repair();
                return;
            }
        }

        // self.output.print_buffer();
        self.output.reset();
    }

    pub fn init_comms(&mut self) {
        self.teensy = self.hidapi.open(self.vid, self.pid);

        match &self.teensy {
            Ok(_) => {
                self.write();
                println!("Teensy connected");
                self.timestamp = Instant::now();

                let param_id = format!("/buffbot/HID_ACTIVE");
                rosrust::param(&param_id).unwrap().set::<i32>(&1).unwrap();
            }
            _ => {
                let param_id = format!("/buffbot/HID_ACTIVE");
                rosrust::param(&param_id).unwrap().set::<i32>(&0).unwrap();
                self.connection_repair();
            }
        }
    }

    pub fn connection_repair(&mut self) {
        drop(&self.teensy);
        // if prgram counter is more than timeout, don't try to
        // reconnect
        if self.timestamp.elapsed().as_millis() < self.nc_timeout {
            println!("Can't find teensy!");
            sleep(Duration::from_millis((self.nc_timeout / 5) as u64));
            self.init_comms();
        }
    }

    pub fn spin(&mut self) {
        self.init_comms();

        while rosrust::is_ok() {
            self.read();
            self.write();

            // Reset this timer to continue searching for a device instead
            // of exiting. For tests, don't reset this and the program will time out.
            self.timestamp = Instant::now();
        }
    }
}
