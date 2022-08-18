extern crate yaml_rust;
use yaml_rust::{yaml::Yaml, YamlLoader};
// use serde::{Deserialize, Serialize, Deserializer, Serializer, SerializeStruct};
use queues::CircularBuffer;
use queues::IsQueue;
use rosrust::ros_info;
use std::{
    fs,
    // fs::File,
    sync::{Arc, RwLock},
    time::Instant,
};

pub struct Device {
    name: String,
    group: String,
    key: char,
    options: Vec<u8>,
}

impl Device {
    pub fn new(device: String, device_type: String, hidkey: char, opts: Vec<u8>) -> Device {
        Device {
            name: device,
            group: device_type,
            key: hidkey,
            options: opts,
        }
    }

    pub fn generate_msg_header(&self, id: u8) -> Vec<u8> {
        let mut msg = vec![self.key as u8, self.key as u8, id];
        msg.extend(&self.options);
        msg
    }

    pub fn generate_anonymous_header(&self) -> Vec<u8> {
        let mut msg = vec![self.key as u8, self.key as u8];
        msg.extend(&self.options);
        msg
    }
}

pub fn parse_mpu(input: Vec<u8>) -> Vec<f64> {
    input
        .chunks_exact(4)
        .map(|chunk| f32::from_be_bytes(chunk.try_into().unwrap_or([0, 0, 0, 0])) as f64)
        .collect()
}

pub fn parse_dr16(input: Vec<u8>) -> Vec<f64> {
    let mut channels = vec![0, 0, 0, 0];

    channels[0] = ((input[1] as u16 & 0x07) << 8) | input[0] as u16;
    channels[1] = ((input[2] as u16 & 0xFC) << 5) | ((input[1] as u16 & 0xF8) >> 3);
    channels[2] =
        (((input[4] as u16 & 0x01) << 10) | ((input[3] as u16) << 2)) | (input[2] as u16 & 0x03);
    channels[3] = ((input[5] as u16 & 0x0F) << 7) | (input[4] as u16 & 0xFE);

    // ros_info!("{:?}", channels);

    let s1 = (input[5] & 0x30) >> 4;
    let s2 = (input[5] & 0xC0) >> 6;

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

    // ros_info!("dr16 input {:?}", vec![xvel, yvel, rotv, gphi, gpsi, shoot]);

    vec![xvel, yvel, rotv, gphi, gpsi, shoot]
}

pub fn parse_rmmotor(input: Vec<u8>) -> Vec<f64> {
    let mut data: Vec<f64> = input
        .chunks_exact(2)
        .map(|chunk| u16::from_be_bytes(chunk.try_into().unwrap_or([0, 0])) as f64)
        .collect();

    data[0] = (data[0] / 8190.0) * 2.0 * std::f64::consts::PI;
    data[1] = (data[1] / 60.0) * 2.0 * std::f64::consts::PI;
    data
}

pub struct MotorTable {
    pub data: Vec<Vec<f64>>,
    pub names: Vec<String>,
    pub timestamp: Vec<Instant>,
}

impl MotorTable {
    pub fn new_arc() -> Arc<RwLock<MotorTable>> {
        let ri = MotorTable {
            data: vec![],
            names: vec![],
            timestamp: vec![],
        };
        Arc::new(RwLock::new(ri))
    }
}

pub struct RawInput {
    pub data: Vec<f64>,
    pub timestamp: Instant,
}

impl RawInput {
    pub fn new_arc() -> Arc<RwLock<RawInput>> {
        let ri = RawInput {
            data: vec![],
            timestamp: Instant::now(),
        };
        Arc::new(RwLock::new(ri))
    }
}

pub struct DeviceTable {
    pub devices: Vec<Device>,
    pub dev_seek: usize,
    pub input_mtable: Arc<RwLock<MotorTable>>,
    pub output_mtable: Arc<RwLock<MotorTable>>,
    pub imu_buffer: Arc<RwLock<RawInput>>,
    pub remote_buffer: Arc<RwLock<RawInput>>,
}

impl DeviceTable {
    pub fn new() -> DeviceTable {
        DeviceTable {
            devices: Vec::new(),
            dev_seek: 0,
            input_mtable: MotorTable::new_arc(),
            output_mtable: MotorTable::new_arc(),
            imu_buffer: RawInput::new_arc(),
            remote_buffer: RawInput::new_arc(),
        }
    }

    pub fn add_motor_buffer(&self, devname: String) {
        let mut input = self.input_mtable.write().unwrap();
        input.data.push(vec![0.0, 0.0, 0.0, 0.0]);
        input.names.push(devname.clone());
        input.timestamp.push(Instant::now());

        let mut output = self.output_mtable.write().unwrap();
        output.data.push(vec![0.0]);
        output.names.push(devname.clone());
        output.timestamp.push(Instant::now());
    }

    pub fn from_yaml(filepath: String) -> DeviceTable {
        let file_data = fs::read_to_string(filepath + "/devices.yaml").unwrap();
        // let dt: DeviceTable = serde_yaml::from_str(file_data.as_str()).unwrap();
        let doc = &YamlLoader::load_from_str(file_data.as_str()).unwrap()[0];

        let mut dt = DeviceTable::new();

        if let Yaml::Hash(ref h) = doc {
            for (k, v) in h {
                let devname = k;
                let dev_yaml = v;
                let mut dgroup = "";
                let mut dkey = 'X';
                let mut dopts = Vec::<u8>::new();
                if let Yaml::Hash(ref h) = dev_yaml {
                    for (k, v) in h {
                        if let Yaml::String(pstr) = k {
                            match pstr.as_str() {
                                "group" => {
                                    dgroup = v.as_str().unwrap();
                                    if dgroup == "can" {
                                        dt.add_motor_buffer(devname.as_str().unwrap().to_string());
                                        dkey = 'M';
                                    } else if dgroup == "receiver" {
                                        dkey = 'D';
                                    } else {
                                        dkey = 'I';
                                    }
                                }
                                "options" => {
                                    dopts = v
                                        .as_vec()
                                        .unwrap()
                                        .iter()
                                        .map(|x| x.as_i64().unwrap() as u8)
                                        .collect::<Vec<u8>>();
                                }

                                _ => {}
                            }
                        }
                    }
                }
                let device = Device::new(
                    devname.as_str().unwrap().to_string(),
                    dgroup.to_string(),
                    dkey,
                    dopts,
                );
                dt.devices.push(device);
            }
        }

        dt
    }

    pub fn generate_inits(&self) -> Vec<Vec<u8>> {
        self.devices
            .iter()
            .enumerate()
            .map(|(i, dev)| {
                let mut msg: Vec<u8>;
                msg = dev.generate_msg_header(i as u8);
                msg.extend(dev.options.clone());

                msg
            })
            .collect()
    }

    pub fn set_input(&mut self, _n_bytes: u8, id: u8, input: Vec<u8>) {
        if id == 255 {
            return;
        }
        if id == 200 {
            let can = input[0];
            let offset = input[1] % 4 - 1;
            let mid = (input[1] / 4) as u8;
            // println!("{} {} {}", can, mid, offset);

            match self.devices.iter().position(|dev| {
                dev.group == "can"
                    && dev.options[0] == can
                    && dev.options[1] == mid
                    && dev.options[2] == offset
            }) {
                Some(idx) => {
                    // println!("{}", self.devices[idx].name);
                    let mut input_table = self.input_mtable.write().unwrap();
                    match input_table
                        .names
                        .iter()
                        .position(|name| *name == self.devices[idx].name)
                    {
                        Some(pos) => {
                            input_table.data[pos] = parse_rmmotor(vec![
                                input[2], input[3], input[4], input[5], input[6], input[7],
                                input[8], input[9],
                            ]);
                            input_table.timestamp[pos] = Instant::now();
                        }
                        _ => {}
                    }

                    return;
                }
                _ => {
                    return;
                }
            }
        }

        match self.devices[id as usize].name.as_str() {
            "mpu6050" => {
                let mut buffer = self.imu_buffer.write().unwrap();
                buffer.data = parse_mpu(input);
                buffer.timestamp = Instant::now();
            }
            "dr16" => {
                // println!("dev Taking buffer ->");
                let mut buffer = self.remote_buffer.write().unwrap();
                buffer.data = parse_dr16(input);
                buffer.timestamp = Instant::now();
                // println!("dev dropping buffer <-");
            }
            _ => {}
        }
    }

    pub fn get_motor_outputs(&self) -> Arc<RwLock<MotorTable>> {
        self.output_mtable.clone()
    }

    pub fn get_motor_inputs(&self) -> Arc<RwLock<MotorTable>> {
        self.input_mtable.clone()
    }

    pub fn get_remote_input(&self) -> Arc<RwLock<RawInput>> {
        self.remote_buffer.clone()
    }

    pub fn get_imu_input(&self) -> Arc<RwLock<RawInput>> {
        self.imu_buffer.clone()
    }
}
