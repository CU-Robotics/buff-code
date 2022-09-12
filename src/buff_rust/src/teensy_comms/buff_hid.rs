extern crate hidapi;

use hidapi::{HidApi, HidDevice, HidError};
// use rosrust::ros_info;
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
    pub input: HidBuffer,
    pub output: HidBuffer,
    pub teensy: Result<HidDevice, HidError>,
    pub output_queue: Arc<RwLock<HidBuffer>>,
    pub subscriber: rosrust::Subscriber,
    pub publishers: Vec<rosrust::Publisher<std_msgs::UInt8MultiArray>>,
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

        let pubs = vec![
            rosrust::publish("can_raw", 1).unwrap(),
            rosrust::publish("imu_raw", 1).unwrap(),
            rosrust::publish("receiver_raw", 1).unwrap(),
        ];

        let api = HidApi::new().expect("Failed to create API instance");
        let teensy = api.open(0x0000, 0x0000);

        HidLayer {
            hidapi: api,
            vid: 0x16C0,
            pid: 0x0486,
            teensy: teensy,
            input: HidBuffer::new(),
            output: HidBuffer::new(),
            output_queue: output_buffer,
            subscriber: can_sub,
            publishers: pubs,
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
            let mut msg = std_msgs::UInt8MultiArray::default();
            self.input.seek_ptr = 33;
            msg.data = self.read_bytes(24);
            self.publishers[msg.data[0] as usize].send(msg).unwrap();
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
        let mut queue = self.output_queue.write().unwrap();
        self.output.reset();
        self.output.data = queue.data;
        queue.reset();
        drop(queue);

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
        self.input.timestamp = Instant::now();

        match &self.teensy {
            Ok(_) => {
                self.write();
                println!("Teensy connected");

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
        if self.input.timestamp.elapsed().as_millis() < 5000 {
            println!("Can't find teensy!");
            sleep(Duration::from_millis(
                5000 - self.input.timestamp.elapsed().as_millis() as u64,
            ));
        }

        self.init_comms();
    }

    pub fn spin(&mut self) {
        self.init_comms();

        while rosrust::is_ok() {
            // timestamp = Instant::now();

            self.read();
            self.write();

            // if micros < 500 {
            //     sleep(Duration::from_micros(500 - micros as u64));
            // } else {
            //     println!("overtime {}", micros);
            // }
        }
    }
}
