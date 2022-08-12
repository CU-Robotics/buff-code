extern crate hidapi;

use crate::device;
use hidapi::{HidApi, HidDevice, HidError};
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

    // pub fn seek_u16(&mut self, set: Option<usize>) -> u16 {
    //     self.seek_ptr = set.unwrap_or(self.seek_ptr);

    //     u16::from_be_bytes([self.seek(None), self.seek(None)])
    // }

    // pub fn seek_f32(&mut self, set: Option<usize>) -> f32 {
    //     self.seek_ptr = set.unwrap_or(self.seek_ptr);

    //     f32::from_le_bytes([
    //         self.seek(None),
    //         self.seek(None),
    //         self.seek(None),
    //         self.seek(None),
    //     ])
    // }

    // pub fn unseek(&mut self) -> u8 {
    //     if self.seek_ptr <= 0 {
    //         self.seek_ptr = 63;
    //     } else {
    //         self.seek_ptr -= 1;
    //     }

    //     self.data[self.seek_ptr]
    // }

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
    pub dtable: device::DeviceTable,
    pub teensy: Result<HidDevice, HidError>,
    pub output_queue: Arc<RwLock<HidBuffer>>,
    pub subscriber: rosrust::Subscriber,
}

impl HidLayer {
    pub fn new() -> HidLayer {
        let output_buffer = Arc::new(RwLock::new(HidBuffer::new()));

        let buffer = Arc::clone(&output_buffer);
        let writer_topic = "teensy_commands";

        let sub = rosrust::subscribe(&writer_topic, 5, move |msg: std_msgs::String| {
            let mut w = buffer.write().unwrap();
            if w.data.len() - w.seek_ptr < msg.data.bytes().len() {
                for c in msg.data.bytes() {
                    w.put(c as u8);
                }
            }
        })
        .unwrap();

        let robot_desc = format!("/buffbot/self");
        let filepath = rosrust::param(&robot_desc)
            .unwrap()
            .get::<String>()
            .unwrap();

        let dt = device::DeviceTable::from_yaml(filepath);

        let api = HidApi::new().expect("Failed to create API instance");
        let teensy = api.open(0x0000, 0x0000);

        HidLayer {
            hidapi: api,
            vid: 0x16C0,
            pid: 0x0486,
            teensy: teensy,
            dtable: dt,
            input: HidBuffer::new(),
            output: HidBuffer::new(),
            output_queue: output_buffer,
            subscriber: sub,
        }
    }

    pub fn reset(&mut self) {
        self.input.reset();
        self.output.reset();
        self.output_queue.write().unwrap().reset();
    }

    pub fn dump_config(&mut self) {
        self.reset();

        while self.dtable.dev_seek < self.dtable.devices.len() - 1 {
            let input_length = &self.dtable.devices[self.dtable.dev_seek]
                .input
                .read()
                .unwrap()
                .len();

            if self.output_queue.write().unwrap().check_of(*input_length) {
                self.write();
                self.read();
            }

            let dev = &self.dtable.devices[self.dtable.dev_seek];
            self.output_queue
                .write()
                .unwrap()
                .puts(dev.generate_init(self.dtable.dev_seek as u8));
            self.dtable.dev_seek += 1;
        }

        self.dtable.dev_seek = 0;
        // self.output_queue.write().unwrap().print_buffer();
    }

    pub fn dump_state(&mut self) {
        self.reset();

        while self.dtable.dev_seek < self.dtable.devices.len() - 1 {
            let input_length = &self.dtable.devices[self.dtable.dev_seek]
                .input
                .read()
                .unwrap()
                .len();

            if self.output_queue.write().unwrap().check_of(*input_length) {
                self.write();
                self.read();
            }

            let dev = &self.dtable.devices[self.dtable.dev_seek];
            self.output_queue
                .write()
                .unwrap()
                .puts(dev.generate_state(self.dtable.dev_seek as u8));
            self.dtable.dev_seek += 1;
        }

        self.dtable.dev_seek = 0;
        // self.output_queue.write().unwrap().print_buffer();
    }

    pub fn read_bytes(&mut self, n_bytes: u8) -> Vec<u8> {
        let mut j = 0;
        let mut data = Vec::<u8>::new();

        while j < n_bytes {
            data.push(self.input.seek(None));
            j += 1;
        }
        data
    }

    pub fn parse_hid(&mut self, n: usize) {
        self.input.seek_ptr = 0;
        if n == 0 {
            return;
        }
        while self.input.seek_ptr < n - 1 {
            match self.input.seek(None) as char {
                'T' => {
                    match self.input.seek(None) as char {
                        'T' => {
                            // self.publish_msg();
                            let num_bytes = self.input.seek(None);
                            let dev_id = self.input.seek(None);
                            let data = self.read_bytes(num_bytes);
                            self.dtable.set_input(num_bytes, dev_id, data);
                        }
                        _ => continue,
                    }
                }

                _ => continue,
            }
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
            Ok(dev) => {
                // let t = Instant::now();

                match dev.write(&self.output.data) {
                    Ok(_) => self.output.timestamp = Instant::now(),
                    Err(_) => self.connection_repair(),
                }
                // println!("write time {}", t.elapsed().as_micros());
            }
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

        let mut timestamp;
        let mut micros;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.read();
            // micros = timestamp.elapsed().as_micros();
            // println!("Read time {}", micros);
            self.write();
            // println!("Write time {}", timestamp.elapsed().as_micros() - micros);

            self.dump_state();

            micros = timestamp.elapsed().as_micros();

            if micros < 1000 {
                sleep(Duration::from_micros(1000 - micros as u64));
            } // else {
              //     println!("overtime {}", micros);
              // }
        }
    }
}
