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
    pub hidapi: HidApi,
    pub vid: u16,
    pub pid: u16,
    pub nc_timeout: u128,
    pub input: ByteBuffer,
    pub output: ByteBuffer,
    pub teensy: Result<HidDevice, HidError>,
    pub output_queue: Arc<RwLock<ByteBuffer>>,
    pub robot_report: BuffBotStatusReport,
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
            hidapi: api,
            nc_timeout: 8000,
            vid: byu.load_u16("teensy_vid"),
            pid: byu.load_u16("teensy_pid"),
            teensy: Err(hidapi::HidError::InitializationError),
            input: ByteBuffer::new(64),
            output: ByteBuffer::new(64),
            output_queue: output_buffer,
            robot_report: BuffBotStatusReport::load_robot(),
            // subscriber: can_sub,
            // publishers: pubs,
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
        self.robot_report = BuffBotStatusReport::load_robot();
    }

    /// Set bytes in the output queue, great for tests
    /// Usage:
    /// '''
    ///     hidlayer.set_output_bytes(idx, bytes); // bytes will be sent next write
    /// '''
    pub fn set_output_bytes(&mut self, idx: usize, data: Vec<u8>) {
        self.output_queue.write().unwrap().puts(idx, data);
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
        match self.input.get(0) {
            0 => {

                // self.robot_report.load_initializers().iter().for_each(|init| {
                //     self.output_queue.write().unwrap().puts(0, init);
                //     self.write();
                //     self.read();
                // });
            }
            1 => {}
            2 => {}
            3 => {}
            u8::MAX => {}
            _ => {}
        }
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

    /// Initialize the connection to a teensy
    /// Usage:
    /// '''
    ///     layer.init_comms();
    /// '''
    pub fn init_comms(&mut self) {
        self.teensy = self.hidapi.open(self.vid, self.pid);

        match &self.teensy {
            Ok(_) => {
                self.write();
                println!("Teensy connected");
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
        // reconnect
        if self.timestamp.elapsed().as_millis() < self.nc_timeout {
            println!("Can't find teensy!");
            sleep(Duration::from_millis((self.nc_timeout / 5) as u64));
            self.init_comms();
        }
    }

    /// Main function to spin and connect the teensy to ROS
    /// Usage:
    /// '''
    ///     layer.init_comms();
    /// '''
    pub fn spin(&mut self) {
        match self.read() {
            64 => {
                self.parse_report();

                if self.input.get_i32(60) == 0 {
                    println!("\tTeensy does not recognize the connection!");
                }
            }
            0 => {
                println!("\tNo Packet available");
            }
            _ => {
                println!("\tCorrupt packet!");
            }
        }

        self.write();

        // Reset this timer to continue searching for a device instead
        // of exiting. For tests, don't reset this and the program will time out.
        self.timestamp = Instant::now();
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
