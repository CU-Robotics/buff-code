use crate::utilities::loaders::*;
use rosrust_msg::std_msgs;
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

pub struct CANPipeline {
    /*
        This pipeline will publish hid can packets into their
        respective ros topic. It also subscribes to each motors
        command (ie. ros topic <motor_name>_command, published by the controller)
        @attrib
            motor_options: unique identifying sequence to declare each
                motors data position in a can packet (aquired from ros param)
            motor_commands: the values to insert in the can packet (aquired from ros topic)
            subscribers: the ros topics to subscribe to (generated from motor configs, configs from ros param)
    */
    pub rate: u128,
    pub motor_options: Vec<Vec<u8>>,
    pub motor_commands: Arc<RwLock<Vec<f64>>>,
    pub motor_feedback: Arc<RwLock<Vec<f64>>>,
    pub subscribers: Vec<rosrust::Subscriber>,
    pub can_publisher: rosrust::Publisher<std_msgs::UInt8MultiArray>,
    pub feedback_publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
}

impl CANPipeline {
    pub fn new() -> CANPipeline {
        // Get the motor/can identifying sequences

        let byu = BuffYamlUtil::default();
        let opts = byu.load_integer_matrix("motor_can_index");
        let rate = byu.load_u128("pid_control_rate");

        // Create new RWLock for ros subscriber
        let motor_commands = Arc::new(RwLock::new(vec![0f64; opts.len()]));
        let motor_feedback = Arc::new(RwLock::new(vec![0f64; 3 * opts.len()]));

        let mut subs = vec![];

        // Publisher for motor feedback
        let fb_pub = rosrust::publish("motor_feedback", 1).unwrap();

        let cmd_clone = motor_commands.clone();
        // Subscriber for motor command
        subs.push(
            rosrust::subscribe(
                "motor_commands",
                1,
                move |msg: std_msgs::Float64MultiArray| {
                    *cmd_clone.write().unwrap() = msg.data;
                },
            )
            .unwrap(),
        );

        // Also subscribe to the can_raw ros topic (published by teensy comms HID Layer)
        let opt_clone = opts.clone();
        let fb_clone = motor_feedback.clone();

        subs.push(
            rosrust::subscribe("can_raw", 10, move |msg: std_msgs::UInt8MultiArray| {
                // HID packets begin with a valid can bus number (1, 2, sometimes 3)
                if msg.data[0] == 0 {
                    return;
                }

                // 4 messages per packet
                for i in 0..4 {
                    // each message is 8 bytes, 2 id and 6 data, dont forget can bus number is @ 0
                    let input = msg.data[(i * 8) + 3..=((i + 1) * 8)].to_vec();
                    let mut data: Vec<f64> = input
                        .chunks_exact(2)
                        .map(|chunk| i16::from_be_bytes(chunk.try_into().unwrap_or([0, 0])) as f64)
                        .collect();

                    // Scale the position to [0, 360)
                    data[0] = (data[0] / 8191.0) * 360.0;

                    // Get the publishers index via option lookup
                    let midx = opts
                        .iter()
                        .position(|opt| {
                            // packet[0] is can bus number, first 2 bytes of each message are
                            // message type and motor offset
                            *opt == vec![msg.data[0], msg.data[(i * 8) + 1], msg.data[(i * 8) + 2]]
                        })
                        .unwrap_or(usize::MAX);

                    // println!("msg recieved {:?}", msg.data[(i * 8) + 1])
                    if midx != usize::MAX {
                        // Send the motors feedback to ros
                        let mut fb = fb_clone.write().unwrap();
                        fb[3 * midx] = data[0];
                        fb[(3 * midx) + 1] = data[1];
                        fb[(3 * midx) + 2] = data[2];
                    }
                }
            })
            .unwrap(),
        );

        let can_pub = rosrust::publish("can_output", 1).unwrap();

        CANPipeline {
            rate: rate,
            motor_options: opt_clone,
            motor_commands: motor_commands,
            motor_feedback: motor_feedback,
            subscribers: subs,
            can_publisher: can_pub,
            feedback_publisher: fb_pub,
        }
    }

    pub fn generate_can_packet(&self) -> Vec<u8> {
        /*
            controller commands -> can packet
        */
        let mut packet = vec![0u8; 50];
        packet[0] = 1;
        let commands = self.motor_commands.read().unwrap().clone();

        self.motor_options.iter().enumerate().for_each(|(i, opts)| {
            let bytes = i16::to_be_bytes((commands[i].clamp(-1.0, 1.0) * 32767.0) as i16).to_vec();
            packet[(((opts[0] - 1) * 24) + (opts[1] * 8) + (opts[2] * 2) + 1) as usize] = bytes[0];
            packet[(((opts[0] - 1) * 24) + (opts[1] * 8) + (opts[2] * 2) + 2) as usize] = bytes[1];
        });

        packet
    }

    pub fn publish_can_packet(&self) {
        let mut msg = std_msgs::UInt8MultiArray::default();
        msg.data = self.generate_can_packet();
        self.can_publisher.send(msg).unwrap();
    }

    pub fn publish_feedback_packet(&self) {
        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = self.motor_feedback.read().unwrap().to_vec();
        self.feedback_publisher.send(msg).unwrap();
    }

    pub fn spin(&self) {
        let mut micros;
        let mut timestamp;
        let mut ctr = 0;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.publish_can_packet();
            self.publish_feedback_packet();
            ctr += 1;

            if ctr > 100 {
                let mut data = self.motor_feedback.write().unwrap();
                *data = data.iter().map(|_| 0.0).collect();
            }

            micros = timestamp.elapsed().as_micros();
            if micros < 1e6 as u128 / self.rate {
                sleep(Duration::from_micros(
                    ((1e6 as u128 / self.rate) - micros) as u64,
                ));
            } else {
                println!("can manager overtime {}", micros);
            }
        }
    }
}

pub struct DR16Pipeline {
    /*
        This is just to clean up the dr16 output from HID
    */
    pub rate: u128,
    pub subscriber: rosrust::Subscriber,
    pub publishers: Vec<rosrust::Publisher<std_msgs::Float64MultiArray>>,
    pub input_data: Arc<RwLock<Vec<u8>>>,
    pub pitch_acc: f64,
    pub yaw_acc: f64,
    pub mouse_sensitivity: f64,
}

impl DR16Pipeline {
    pub fn new() -> DR16Pipeline {
        let byu = BuffYamlUtil::default();
        let rate = byu.load_u128("state_control_rate");

        let input = Arc::new(RwLock::new(vec![0u8; 24]));
        let input_clone = input.clone();

        let publ = vec![
            rosrust::publish("joystick_input", 1).unwrap(),
            rosrust::publish("keyboard_input", 1).unwrap(),
        ];

        let sub = rosrust::subscribe("dr16_raw", 10, move |msg: std_msgs::UInt8MultiArray| {
            let mut input_data = input.write().unwrap();
            msg.data
                .iter()
                .enumerate()
                .for_each(|(i, x)| input_data[i] = *x);
        })
        .unwrap();

        DR16Pipeline {
            rate: rate,
            subscriber: sub,
            publishers: publ,
            input_data: input_clone,
            pitch_acc: 0.0,
            yaw_acc: 0.0,
            mouse_sensitivity: 0.1,
        }
    }

    pub fn generate_joystick_velocity_message(&mut self) -> Vec<f64> {
        let mut data = vec![0f64; 8];

        /*
            Remote control format (f64s)
            0: fire (shooter toogle)
            1: toggle (mode select)
            2: x speed
            3: y speed
            4: omega
            5: pitch
            6: yaw
            7: feeder
        */

        let input_data = self.input_data.read().unwrap().clone();

        if input_data.len() < 20 {
            return data;
        }

        data[0] = (input_data[0] & 0x03) as f64; // set switch 1
        data[1] = ((input_data[0] & 0x0C) >> 2) as f64; // set switch 2

        // Extract the int16s and convert to floats
        input_data[3..11]
            .chunks_exact(2)
            .enumerate()
            .for_each(|(i, chunk)| {
                data[i + 2] = i16::from_be_bytes(chunk.try_into().unwrap_or([0, 0])) as f64
                    / i16::MAX as f64
                    * 8000.0;
            });

        if data[0] == 1.0 {
            data[4] = -4000.0;
        } else if data[0] == 2.0 {
            data[4] = 4000.0;
        } else {
            data[4] = 0.0;
        }

        // handle pan tilt accumulation
        // self.pitch_acc += data[4] * self.mouse_sensitivity;
        // self.yaw_acc += data[5] * self.mouse_sensitivity;

        // data[5] = self.pitch_acc;
        // data[6] = self.yaw_acc;

        data
    }

    pub fn generate_keyboard_message(&mut self) -> Vec<f64> {
        let mut data = vec![0f64; 7];

        /*
            Remote control format (f64s)
            0: fire (shooter toogle)
            1: toggle (mode select)
            2: x speed normalized
            3: y speed normalized
            4: omega speed normalized
            5: pitch global normalized
            6: yaw global normalized
        */

        let input_data = self.input_data.read().unwrap().clone();

        if input_data.len() < 20 {
            return data;
        }

        // parse keyboard input
        data[0] = input_data[0] as f64; // set lmb
        data[1] = input_data[1] as f64; // set rmb

        // not implemented
        data
    }

    pub fn publish_messages(&mut self) {
        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = self.generate_joystick_velocity_message();
        self.publishers[0].send(msg).unwrap();

        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = self.generate_keyboard_message();
        self.publishers[1].send(msg).unwrap();
    }

    pub fn spin(&mut self) {
        let mut micros;
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.publish_messages();

            micros = timestamp.elapsed().as_micros();
            if micros < 1e6 as u128 / self.rate {
                sleep(Duration::from_micros(
                    ((1e6 as u128 / self.rate) - micros) as u64,
                ));
            } else {
                println!("dr16 manager overtime {}", micros);
            }
        }
    }
}
