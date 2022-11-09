use rosrust::ros_info;
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
    pub motor_options: Vec<Vec<u8>>,
    pub motor_commands: Arc<RwLock<Vec<f64>>>,
    pub subscribers: Vec<rosrust::Subscriber>,
    pub can_publisher: rosrust::Publisher<std_msgs::UInt8MultiArray>,
}

impl CANPipeline {
    pub fn new() -> CANPipeline {
        // Get the motor/can identifying sequences
        let motor_info = rosrust::param("/buffbot/can")
            .unwrap()
            .get::<Vec<Vec<String>>>()
            .unwrap();

        // Save the motor names for quiker indexing
        let names: Vec<String> = motor_info.iter().map(|motor| motor[0].clone()).collect();

        // Save motor configs
        let opts: Vec<Vec<u8>> = motor_info
            .iter()
            .map(|motor| {
                motor[1..]
                    .iter()
                    .map(|x| x.parse::<u8>().unwrap())
                    .collect()
            })
            .collect();

        let opt_clone = opts.clone();

        // Create new RWLock for ros subscriber
        let motor_commands = Arc::new(RwLock::new(vec![0f64; opts.len()]));

        let mut subs = vec![];
        let mut pubs = vec![];

        names.iter().enumerate().for_each(|(i, motor_name)| {
            let cmd_clone = motor_commands.clone();
            // Publisher for motor feedback
            pubs.push(rosrust::publish(format!("{}_state", motor_name).as_str(), 1).unwrap());

            let cmd_clone = motor_commands.clone();
            // Subscriber for motor command
            subs.push(
                rosrust::subscribe(
                    format!("{}_command", motor_name).as_str(),
                    1,
                    move |msg: std_msgs::Float64| {
                        cmd_clone.write().unwrap()[i] = msg.data;
                    },
                )
                .unwrap(),
            );
        });

        // Also subscribe to the can_raw ros topic (published by teensy comms HID Layer)
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
                        let mut tmp_msg = std_msgs::Float64MultiArray::default();
                        tmp_msg.data = data[..3].to_vec();
                        pubs[midx].send(tmp_msg).unwrap();
                    }
                }
            })
            .unwrap(),
        );

        let can_pub = rosrust::publish("can_output", 1).unwrap();

        CANPipeline {
            motor_options: opt_clone,
            motor_commands: motor_commands,
            subscribers: subs,
            can_publisher: can_pub,
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
            let power = commands[i];
            let bytes = i16::to_be_bytes((power.clamp(-1.0, 1.0) * 32767.0) as i16).to_vec();
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

    pub fn spin(&self) {
        let mut micros;
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.publish_can_packet();

            micros = timestamp.elapsed().as_micros();
            if micros < 10000 {
                sleep(Duration::from_micros(10000 - micros as u64));
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
    pub subscriber: rosrust::Subscriber,
    pub publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
    pub input_data: Arc<RwLock<Vec<u8>>>,
    pub keyboard_select: bool,
    pub pitch_acc: f64,
    pub yaw_acc: f64,
    pub mouse_sensitivity: f64,
}

impl DR16Pipeline {
    pub fn new() -> DR16Pipeline {
        let keyboard = rosrust::param("/buffbot/keyboard_select")
            .unwrap()
            .get::<Vec<bool>>()
            .unwrap()[0];

        let input = Arc::new(RwLock::new(vec![0u8; 24]));
        let input_clone = input.clone();

        let publ = rosrust::publish("reference_input", 1).unwrap();

        let sub = rosrust::subscribe("receiver_raw", 10, move |msg: std_msgs::UInt8MultiArray| {
            let mut input_data = input.write().unwrap();
            msg.data
                .iter()
                .enumerate()
                .for_each(|(i, x)| input_data[i] = *x);
        })
        .unwrap();

        DR16Pipeline {
            subscriber: sub,
            publisher: publ,
            input_data: input_clone,
            keyboard_select: keyboard,
            pitch_acc: 0.0,
            yaw_acc: 0.0,
            mouse_sensitivity: 0.1,
        }
    }

    pub fn generate_control_message(&mut self) -> Vec<f64> {
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

        if self.keyboard_select {
            data[0] = input_data[0] as f64; // set lmb
            data[1] = input_data[1] as f64; // set rmb

        // not implemented
        } else {
            data[0] = (input_data[0] & 0x03) as f64; // set switch 1
            data[1] = (input_data[0] & 0x0C) as f64; // set switch 2

            // Extract the int16s and convert to floats
            input_data[3..11]
                .chunks_exact(2)
                .enumerate()
                .for_each(|(i, chunk)| {
                    data[i + 2] = i16::from_be_bytes(chunk.try_into().unwrap_or([0, 0])) as f64
                        / i16::MAX as f64;
                });

            // handle pan tilt accumulation
            self.pitch_acc += data[4] * self.mouse_sensitivity;
            self.yaw_acc += data[5] * self.mouse_sensitivity;

            data[4] = self.pitch_acc;
            data[5] = self.yaw_acc;
        }

        data
    }

    pub fn publish_message(&mut self) {
        let mut msg = std_msgs::Float64MultiArray::default();
        msg.data = self.generate_control_message();
        self.publisher.send(msg).unwrap();
    }

    pub fn spin(&mut self) {
        let mut micros;
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            self.publish_message();

            micros = timestamp.elapsed().as_micros();
            if micros < 25000 {
                sleep(Duration::from_micros(25000 - micros as u64));
            } else {
                println!("dr16 manager overtime {}", micros);
            }
        }
    }
}
