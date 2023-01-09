extern crate hidapi;

// use rosrust::ros_info;
use crate::utilities::{buffers::*, data_structures::*, loaders::*};
use hidapi::{HidApi, HidDevice, HidError};
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

/// A Handler for HID teensys
/// Usage:
/// '''
///     let layer = HidLayer::new();
///     layer.spin();
/// '''
pub struct HidLayer {
    pub nc_timeout: u128,
    pub initialized: bool,
    pub pid: u16,
    pub vid: u16,

    pub input: ByteBuffer,
    pub output: ByteBuffer,
    pub output_queue: Arc<RwLock<ByteBuffer>>,
    pub robot_status: BuffBotStatusReport,

    pub hidapi: HidApi,
    pub teensy: Result<HidDevice, HidError>,

    // pub subscriber: rosrust::Subscriber,
    // pub publishers: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
    pub timestamp: Instant,
}

impl HidLayer {
    /// Create a new HidLayer object
    /// Usage:
    /// '''
    ///     use crate::teensy_comms::HidLayer;
    ///     let layer = HidLayer::new();
    /// '''
    pub fn new() -> HidLayer {
        // Create a buffer to catch outgoing data
        let output_buffer = Arc::new(RwLock::new(ByteBuffer::new(64)));

        // Clone the output buffer
        // let output_buffer_clone = Arc::clone(&output_buffer);

        // Create a subscriber to the motor_commands ROS topic.
        // let can_sub = rosrust::subscribe(
        //     "motor_commands",
        //     1,
        //     move |mut msg: std_msgs::Float64MultiArray| {
        //         let mut w = output_buffer_clone.write().unwrap();
        //         msg.data
        //             .iter_mut()
        //             .enumerate()
        //             .for_each(|(i, x)| w.puts(1 + i, x.to_be_bytes().to_vec()));
        //     },
        // )
        // .unwrap();

        let byu = BuffYamlUtil::new("penguin");
        // let sensors = byu.load_string_list("sensor_index");

        // let mut pubs: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>> = sensors
        //     .iter()
        //     .map(|s| rosrust::publish(s, 1).unwrap())
        //     .collect();

        // pubs.splice(0..0, rosrust::publish("motor_feedback", 1));

        let api = HidApi::new().expect("Failed to create API instance");

        HidLayer {
            nc_timeout: 8000,
            initialized: false,
            vid: byu.load_u16("teensy_vid"),
            pid: byu.load_u16("teensy_pid"),

            input: ByteBuffer::new(64),
            output: ByteBuffer::new(64),
            output_queue: output_buffer,
            robot_status: BuffBotStatusReport::load_robot(),

            hidapi: api,
            teensy: Err(hidapi::HidError::InitializationError),

            // subscriber: rosrust::Subscriber,
            // publishers: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
            timestamp: Instant::now(),
        }
    }

    /// Reset the HidLayer object
    /// Usage:
    /// '''
    ///     hidlayer.reset(); // clears the current layers data
    /// '''
    pub fn reset(&mut self) {
        self.input.reset();
        self.output.reset();
        self.output_queue.write().unwrap().reset();
        self.robot_status = BuffBotStatusReport::load_robot();
    }

    /// Set bytes in the output queue, great for tests
    /// Usage:
    /// '''
    ///     hidlayer.set_output_bytes(idx, bytes); // bytes will be sent next write
    /// '''
    pub fn set_output_bytes(&mut self, idx: usize, data: Vec<u8>) {
        self.output_queue.write().unwrap().puts(idx, data);
    }

    /// Read an HID packet and handle the result
    /// Usage:
    /// '''
    ///     // Devices sends a report
    ///     layer.read();
    /// '''
    pub fn read(&mut self) -> usize {
        match &self.teensy {
            Ok(dev) => match dev.read(&mut self.input.data) {
                Ok(value) => {
                    self.input.timestamp = Instant::now();
                    return value;
                }
                _ => {}
            },
            _ => {}
        }
        return 0;
    }

    /// Write an HID packet from the output queue and handle the result
    /// Usage:
    /// '''
    ///     // output queue becomes ready
    ///     layer.write();
    /// '''
    pub fn write(&mut self) {
        self.output.reset();
        {
            let mut queue = self.output_queue.write().unwrap();
            self.output.data.copy_from_slice(&queue.data);
            queue.reset();
        }

        match &self.teensy {
            Ok(dev) => match dev.write(&self.output.data) {
                Ok(_) => self.output.timestamp = Instant::now(),
                _ => {}
            },
            _ => {}
        }
    }

    /// After requesting a report this can be used to wait for a reply
    /// Usage:
    /// '''
    ///     // set layer.output.data[0] to an int [1-3]
    ///     layer.write()
    ///     layer.wait_for_packet(255, 10);
    /// '''
    pub fn validate_input_report(&mut self) -> u8 {
        let mode = self.input.get(0);
        let timer = self.input.get_i32(60);

        if timer == 0 {
            println!("\tTeensy does not recognize the connection!");
        } else if timer > 1200 {
            println!(
                "\tTeensy request {} took longer than the cycle limit {}",
                mode, timer
            );
        }

        return mode;
    }

    /// After requesting a report this can be used to wait for a reply
    /// Usage:
    /// '''
    ///     // set layer.output.data[0] to an int [1-3]
    ///     layer.write()
    ///     layer.wait_for_report_reply(255, 10);
    /// '''
    pub fn wait_for_report_reply(&mut self, packet_id: u8, timeout: u128) -> u8 {
        let mut loopt;
        let t = Instant::now();

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match self.read() {
                64 => {
                    if self.validate_input_report() == packet_id {
                        return packet_id;
                    }
                }
                _ => {}
            }

            self.write();

            // HID runs at 1 ms
            while loopt.elapsed().as_micros() < 1000 {}
        }

        // If packet never arrives
        panic!("\tTimed out waiting for reply from Teensy");
    }

    /// send initializers to a teensy
    /// Usage:
    /// '''
    ///     layer.teensy = layer.hidapi.open(layer.vid, layer.pid);
    ///     layer.initialize();
    /// '''
    pub fn initialize(&mut self) {
        self.set_output_bytes(0, vec![255]);
        self.robot_status
            .load_initializers()
            .into_iter()
            .for_each(|initializer| self.set_output_bytes(1, initializer));
        self.write();

        if self.wait_for_report_reply(255, 10) == 255 {
            self.initialized = true;
            println!("\tTeensy Initialized!");
        } else {
            panic!("Teensy did not respond to initializers");
        }
    }

    /// Initialize the connection to a teensy
    /// Usage:
    /// '''
    ///     let layer = HidLayer::new();
    ///     layer.init_comms();
    /// '''
    pub fn init_comms(&mut self) {
        self.teensy = self.hidapi.open(self.vid, self.pid);

        match &self.teensy {
            Ok(_) => {
                println!("\tTeensy connected");
                self.initialize();
                self.timestamp = Instant::now();

                // if rosrust::is_ok() {
                //     let param_id = format!("/buffbot/HID_ACTIVE");
                //     rosrust::param(&param_id).unwrap().set::<i32>(&1).unwrap();
                // }
            }
            _ => {
                // if rosrust::is_ok() {
                //     let param_id = format!("/buffbot/HID_ACTIVE");
                //     rosrust::param(&param_id).unwrap().set::<i32>(&1).unwrap();
                // }
            }
        }
    }

    /// Attempt to repair hidapi connection to a teensy
    /// Usage:
    /// '''
    ///     // teensy throws an HidError or HidApiError
    ///     layer.connection_repair();
    /// '''
    pub fn connection_repair(&mut self) {
        drop(&self.teensy);

        // if prgram counter is more than timeout, don't try to
        // reconnect (prevent infinite recursion while repairing connection)
        if self.timestamp.elapsed().as_millis() < self.nc_timeout {
            println!("\tCan't find teensy!");
            sleep(Duration::from_millis((self.nc_timeout / 5) as u64));
            self.init_comms();
        }
    }

    /// Parse the stored HID packet into BuffBot Data Structures
    /// Usage:
    /// '''
    ///     // HID packet waiting
    ///     if layer.read() > 0 { // layer.read returns the number of bytes read
    ///         layer.parse_report();
    ///     }
    /// '''
    pub fn parse_report(&mut self) {
        // match the report number to determine the structure
        match self.validate_input_report() {
            1 => {
                let index_offset = (self.input.get(1) * 4) as usize;
                self.robot_status
                    .update_motor_encoder(index_offset, self.input.get_floats(2, 3));
                self.robot_status
                    .update_motor_encoder(index_offset + 1, self.input.get_floats(14, 3));
                self.robot_status
                    .update_motor_encoder(index_offset + 2, self.input.get_floats(26, 3));
                self.robot_status
                    .update_motor_encoder(index_offset + 3, self.input.get_floats(38, 3));
            }
            2 => {}
            3 => {
                self.robot_status
                    .update_sensor(self.input.get(1) as usize, self.input.get_floats(2, 9));
            }
            u8::MAX => {
                println!("\tTeensy requested new initializers... somethings wrong");
            }
            _ => {}
        }
    }

    /// Main function to spin and connect the teensy to ROS
    /// Usage:
    /// '''
    ///     layer.init_comms();
    ///     layer.spin();       // runs until watchdog times out
    /// '''
    pub fn spin(&mut self) {
        let mut report_request = 0;
        let reports = self.robot_status.get_reports();

        loop {
            let loopt = Instant::now();

            match self.initialized {
                false => {
                    self.initialize();
                }
                true => {
                    self.set_output_bytes(0, reports[report_request].clone());
                    self.write();
                    self.wait_for_report_reply(reports[report_request][0], 10);
                    self.parse_report();
                    report_request = (report_request + 1) % reports.len();
                }
            }

            // Reset watchdog timer.
            self.timestamp = Instant::now();

            while loopt.elapsed().as_micros() < 1000 {}
            self.robot_status.motors[4].print();
        }
    }

    /// Attempt to delete a teensy connection
    /// Usage:
    /// '''
    ///     // teensy communication finished
    ///     layer.close();
    /// '''
    pub fn close(&mut self) {
        drop(&self.teensy);
    }
}
