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

                for i in 0..4 {
                    let input = msg.data[(i * 8) + 3..=((i + 1) * 8)].to_vec();
                    let mut data: Vec<f64> = input
                        .chunks_exact(2)
                        .map(|chunk| i16::from_be_bytes(chunk.try_into().unwrap_or([0, 0])) as f64)
                        .collect();

                    data[0] = (data[0] / 8191.0) * 360.0;

                    let midx = opts
                        .iter()
                        .position(|opt| {
                            *opt == vec![
                                msg.data[0] - 1,
                                msg.data[(i * 8) + 1],
                                msg.data[(i * 8) + 2],
                            ]
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
