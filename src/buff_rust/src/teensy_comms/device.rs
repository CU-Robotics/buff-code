extern crate yaml_rust;
use rosrust::ros_info;
use rosrust_msg::std_msgs;
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

pub struct CANPipeline {
    pub motor_options: Vec<Vec<u8>>,
    // pub motor_states: Arc<RwLock<Vec<Vec<f64>>>>,
    pub motor_commands: Arc<RwLock<Vec<f64>>>,
    pub subscribers: Vec<rosrust::Subscriber>,
}

impl CANPipeline {
    pub fn new() -> CANPipeline {
        let can_desc = format!("/buffbot/can");

        let motor_info = rosrust::param(&can_desc)
            .unwrap()
            .get::<Vec<Vec<String>>>()
            .unwrap();

        let names: Vec<String> = motor_info.iter().map(|motor| motor[0].clone()).collect();

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

        // let motor_states = Arc::new(RwLock::new(vec![vec![0f64; 3]; opts.len()]));
        // let state_clone = motor_states.clone();

        let motor_commands = Arc::new(RwLock::new(vec![0f64; opts.len()]));

        let mut subs = vec![];
        let mut pubs = vec![];

        names.iter().enumerate().for_each(|(i, x)| {
            let cmd_clone = motor_commands.clone();
            pubs.push(rosrust::publish(format!("{}_state", x).as_str(), 1).unwrap());

            let cmd_clone = motor_commands.clone();
            subs.push(
                rosrust::subscribe(
                    format!("{}_command", x).as_str(),
                    1,
                    move |msg: std_msgs::Float64| {
                        cmd_clone.write().unwrap()[i] = msg.data;
                    },
                )
                .unwrap(),
            );
        });

        subs.push(
            rosrust::subscribe("can_raw", 5, move |msg: std_msgs::UInt8MultiArray| {
                if msg.data[0] == 0 {
                    return;
                }

                for i in 0..3 {
                    let input = msg.data[(i * 9) + 2..=((i + 1) * 9)].to_vec();
                    let mut data: Vec<f64> = input
                        .chunks_exact(2)
                        .map(|chunk| i16::from_be_bytes(chunk.try_into().unwrap_or([0, 0])) as f64)
                        .collect();

                    data[0] = (data[0] / 8190.0) * 360;

                    let midx = opts
                        .iter()
                        .position(|opt| {
                            *opt == vec![msg.data[0] - 1, i as u8, msg.data[(i * 9) + 1] as u8]
                        })
                        .unwrap_or(usize::MAX);

                    if midx != usize::MAX {
                        let mut tmp_msg = std_msgs::Float64MultiArray::default();
                        tmp_msg.data = data[..3].to_vec();
                        pubs[midx].send(tmp_msg).unwrap();
                    }
                }
            })
            .unwrap(),
        );

        CANPipeline {
            motor_options: opt_clone,
            // motor_states: state_clone,
            motor_commands: motor_commands,
            subscribers: subs,
        }
    }

    pub fn generate_can_packet(&self) -> Vec<u8> {
        let mut packet = vec![0u8; 50];
        packet[0] = 1;
        let commands = self.motor_commands.read().unwrap().clone();

        self.motor_options.iter().enumerate().for_each(|(i, opts)| {
            let power = commands[i];
            let bytes = i16::to_be_bytes((power.clamp(-1.0, 1.0) * 32767.0) as i16).to_vec();
            packet[((opts[0] * 24) + (opts[1] * 8) + (opts[2] * 2) + 1) as usize] = bytes[0];
            packet[((opts[0] * 24) + (opts[1] * 8) + (opts[2] * 2) + 2) as usize] = bytes[1];
        });

        packet
    }

    pub fn spin() {
        let mut micros;
        let mut timestamp;
        let pipeline = CANPipeline::new();

        // let state_publisher = rosrust::publish("motor_states", 1).unwrap();
        let can_publisher = rosrust::publish("can_output", 1).unwrap();

        while rosrust::is_ok() {
            timestamp = Instant::now();

            // let mut msg = std_msgs::Float64MultiArray::default();
            // msg.data = pipeline
            //     .motor_states
            //     .read()
            //     .unwrap()
            //     .clone()
            //     .into_iter()
            //     .flatten()
            //     .collect::<Vec<f64>>();
            // state_publisher.send(msg).unwrap();

            let mut msg = std_msgs::UInt8MultiArray::default();
            msg.data = pipeline.generate_can_packet();
            can_publisher.send(msg).unwrap();

            micros = timestamp.elapsed().as_micros();
            if micros < 2500 {
                sleep(Duration::from_micros(2500 - micros as u64));
            } else {
                println!("overtime {}", micros);
            }
        }
    }
}

pub fn mpu_spin() {
    let publisher = rosrust::publish("imu_processed", 5).unwrap();

    let _sub = rosrust::subscribe("imu_raw", 1, move |msg: std_msgs::UInt8MultiArray| {
        let mut proc_msg = std_msgs::Float64MultiArray::default();
        proc_msg.data = msg
            .data
            .chunks_exact(4)
            .map(|chunk| f32::from_be_bytes(chunk.try_into().unwrap_or([0, 0, 0, 0])) as f64)
            .collect();

        publisher.send(proc_msg).unwrap();
    })
    .unwrap();

    rosrust::spin();
}

pub fn dr16_spin() {
    let publisher = rosrust::publish("receiver_processed", 5).unwrap();

    let _sub = rosrust::subscribe("receiver_raw", 1, move |msg: std_msgs::UInt8MultiArray| {
        let mut channels = vec![0, 0, 0, 0];

        channels[0] = ((msg.data[2] as u16 & 0x07) << 8) | msg.data[1] as u16;
        channels[1] = ((msg.data[3] as u16 & 0xFC) << 5) | ((msg.data[2] as u16 & 0xF8) >> 3);
        channels[2] = (((msg.data[5] as u16 & 0x01) << 10) | ((msg.data[4] as u16) << 2))
            | (msg.data[3] as u16 & 0x03);
        channels[3] = ((msg.data[6] as u16 & 0x0F) << 7) | (msg.data[5] as u16 & 0xFE);

        let s1 = (msg.data[6] & 0x30) >> 4;
        let s2 = (msg.data[6] & 0xC0) >> 6;

        let xvel: f64 = (channels[3] as f64 - 1024.0) / ((1704.0 - 476.0) / 2.0);
        let yvel: f64 = (channels[2] as f64 - 1024.0) / ((1684.0 - 364.0) / 2.0);

        let rotv: f64 = match s1 {
            1 => 1.0,
            2 => -1.0,
            3 => 0.0,
            _ => 0.0,
        };

        let shoot: f64 = match s2 {
            1 => 1.0,
            2 => -1.0,
            3 => 0.0,
            _ => 0.0,
        };

        let gphi: f64 = (channels[1] as f64 - 1024.0) / ((1684.0 - 268.0) / 2.0);
        let gpsi: f64 = (channels[0] as f64 - 1024.0) / ((1684.0 - 364.0) / 2.0);

        let mut proc_msg = std_msgs::Float64MultiArray::default();
        proc_msg.data = vec![xvel, yvel, 0.0, 0.0, 0.0, rotv, 0.0, gphi, gpsi, shoot];
        publisher.send(proc_msg).unwrap();
    })
    .unwrap();

    rosrust::spin();
}
