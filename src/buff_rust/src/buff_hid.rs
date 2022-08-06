extern crate hidapi;

use hidapi::{HidApi, HidDevice, HidError};
use rosrust::ros_info;
use rosrust_msg::{std_msgs, std_msgs::Float64MultiArray};
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

    pub fn seek_u16(&mut self, set: Option<usize>) -> u16 {
        self.seek_ptr = set.unwrap_or(self.seek_ptr);

        u16::from_be_bytes([self.seek(None), self.seek(None)])
    }

    pub fn seek_f32(&mut self, set: Option<usize>) -> f32 {
        self.seek_ptr = set.unwrap_or(self.seek_ptr);

        f32::from_le_bytes([
            self.seek(None),
            self.seek(None),
            self.seek(None),
            self.seek(None),
        ])
    }

    pub fn unseek(&mut self) -> u8 {
        if self.seek_ptr <= 0 {
            self.seek_ptr = 63;
        } else {
            self.seek_ptr -= 1;
        }

        self.data[self.seek_ptr]
    }

    pub fn put(&mut self, value: u8) {
        self.data[self.seek_ptr] = value;
        if self.seek_ptr < 63 {
            self.seek_ptr += 1;
        } else {
            self.seek_ptr = 0;
        }
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
    pub dr16_input: Arc<RwLock<[u8; 18]>>,
    pub teensy: Result<HidDevice, HidError>,
    pub output_queue: Arc<RwLock<HidBuffer>>,
    pub subscriber: rosrust::Subscriber,
    pub publishers: Vec<rosrust::Publisher<Float64MultiArray>>,
    pub write_time: Instant,
}

impl HidLayer {
    pub fn new() -> HidLayer {
        let mut publishers = Vec::<rosrust::Publisher<Float64MultiArray>>::new();

        let io_topics = rosrust::param("/buffbot/DEVICES/LUT")
            .unwrap()
            .get::<Vec<String>>()
            .unwrap();

        for topic in io_topics {
            let topic_prefix = "/buffbot/TOPICS/".to_string();
            let topic = rosrust::param(&(topic_prefix + &topic))
                .unwrap()
                .get::<String>()
                .unwrap();
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
        })
        .unwrap();

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
            write_time: Instant::now(),
        }
    }

    pub fn reset(&mut self) {
        self.write_time = Instant::now();
        self.input.reset();
        self.output.reset();
        self.output_queue.write().unwrap().reset();
    }

    pub fn dump_config(&mut self) {
        self.reset();
        let devices = rosrust::param("/buffbot/DEVICES/LUT")
            .unwrap()
            .get::<Vec<String>>()
            .unwrap();

        for (i, dtype) in devices
            .iter()
            .map(|device| {
                rosrust::param(&format!("/buffbot/DEVICES/{}/TYPE", device))
                    .unwrap()
                    .get::<String>()
                    .unwrap()
            })
            .enumerate()
        {
            match dtype.as_str() {
                "motor" => {
                    let canbus_param_id = format!("/buffbot/DEVICES/{}/CANBUS", devices[i]);
                    let motorid_param_id = format!("/buffbot/DEVICES/{}/MOTORID", devices[i]);
                    let motortype_param_id = format!("/buffbot/DEVICES/{}/MOTORTYPE", devices[i]);

                    let canid = rosrust::param(&canbus_param_id)
                        .unwrap()
                        .get::<u8>()
                        .unwrap();
                    let motorid = rosrust::param(&motorid_param_id)
                        .unwrap()
                        .get::<u8>()
                        .unwrap();
                    let motortype = rosrust::param(&motortype_param_id)
                        .unwrap()
                        .get::<u8>()
                        .unwrap();

                    let mut queue = self.output_queue.write().unwrap();
                    queue.put('X' as u8);
                    queue.put('X' as u8);
                    queue.put('M' as u8);
                    queue.put(i as u8);
                    queue.put(canid);
                    queue.put(motorid);
                    queue.put(motortype);
                }

                "imu" => {
                    let mut queue = self.output_queue.write().unwrap();
                    queue.put('X' as u8);
                    queue.put('X' as u8);
                    queue.put('I' as u8);
                    queue.put(i as u8);
                    queue.put(1);
                    drop(queue);
                }

                "dr16" => {
                    let mut queue = self.output_queue.write().unwrap();
                    queue.put('X' as u8);
                    queue.put('X' as u8);
                    queue.put('D' as u8);
                    queue.put(i as u8);
                    drop(queue);
                }

                _ => continue,
            }

            if self.output_queue.write().unwrap().seek_ptr >= 57 {
                self.read();
                self.write();
                sleep(Duration::from_millis(1));
            }
        }
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

                    let data: Vec<f64> = match data_type {
                        0 => self.read_bytes_as_bytes(data_bytes),
                        1 => self.read_bytes_as_u16(data_bytes),
                        2 => self.read_bytes_as_f32(data_bytes),
                        _ => continue,
                    };
                    let mut msg = Float64MultiArray::default();
                    msg.data = data;
                    let _result = self.publishers[data_id].send(msg).unwrap();
                    continue;
                }
            }
        }
        self.input.reset();
    }

    pub fn read(&mut self) {
        let mut n = 0;

        match &self.teensy {
            Ok(dev) => match dev.read(&mut self.input.data) {
                Ok(value) => n = value,
                _ => {
                    n = 0;
                    self.connection_repair();
                }
            },
            _ => self.connection_repair(),
        }

        self.input.timestamp = Instant::now();
        //self.input.print_buffer();
        self.parse_hid(n);
    }

    pub fn write(&mut self) {
        let mut queue = self.output_queue.write().unwrap();
        self.output.reset();
        self.output.data = queue.data;
        queue.reset();
        drop(queue);

        match &self.teensy {
            Ok(dev) => {
                let t = Instant::now();

                match dev.write(&self.output.data) {
                    Ok(value) => self.output.timestamp = Instant::now(),
                    Err(e) => self.connection_repair(),
                    _ => println!("Default Write case"),
                }
            }
            _ => {
                self.connection_repair();
                return;
            }
        }
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
        if self.output.timestamp.elapsed().as_millis() < 5000 {
            println!(
                "Sleeping for {}",
                5000 - self.output.timestamp.elapsed().as_millis() as u64
            );
            sleep(Duration::from_millis(
                5000 - self.output.timestamp.elapsed().as_millis() as u64,
            ));
        }
        self.init_comms();
    }

    pub fn spin(&mut self) {
        self.init_comms();

        let mut timestamp = Instant::now();

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.read();
            self.write();

            let mut micros = timestamp.elapsed().as_micros();

            if micros < 500 {
                sleep(Duration::from_micros(500 - micros as u64));
            }
        }
    }
}
