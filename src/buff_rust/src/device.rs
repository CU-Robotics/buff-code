extern crate yaml_rust;
use rosrust_msg::std_msgs::Float64MultiArray;
use yaml_rust::{yaml::Yaml, YamlLoader};
// use serde::{Deserialize, Serialize, Deserializer, Serializer, SerializeStruct};
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
    ros_pub: Option<rosrust::Publisher<Float64MultiArray>>,
    timestamp: Instant,

    pub input: Arc<RwLock<Vec<f64>>>,
    pub output: Arc<RwLock<Vec<u8>>>,
}

impl Device {
    pub fn new(
        device: String,
        device_type: String,
        hidkey: char,
        opts: Vec<u8>,
        with_ros: bool,
        input_len: u8,
        output_len: u8,
    ) -> Device {
        let mut publisher = None;

        if with_ros {
            // let dev_topic = device.as_str() + "raw";
            publisher = Some(rosrust::publish(format!("{}_raw", device).as_str(), 1).unwrap());
        }

        Device {
            name: device,
            group: device_type,
            key: hidkey,
            options: opts,
            ros_pub: publisher,
            timestamp: Instant::now(),
            input: Arc::new(RwLock::new(vec![0f64; input_len as usize])),
            output: Arc::new(RwLock::new(vec![0u8; output_len as usize])),
        }
    }

    pub fn generate_init(&self, id: u8) -> Vec<u8> {
        let mut msg = vec![self.key as u8, self.key as u8, id];
        msg.extend(&self.options);
        msg
    }

    pub fn generate_state(&self, id: u8) -> Vec<u8> {
        let leng = self.output.read().unwrap().len();
        let mut msg = vec!['m' as u8, 'm' as u8, id];

        if leng > 0 {
            let data = self.output.read().unwrap();
            // println!("hid {:?}", data);
            data.iter().for_each(|x| msg.push(*x));
        } else {
            msg = vec![];
        }

        msg
    }

    pub fn parse_mpu(&mut self, input: Vec<u8>) {
        let data: Vec<f64> = input
            .chunks_exact(4)
            .map(|chunk| f32::from_be_bytes(chunk.try_into().unwrap_or([0, 0, 0, 0])) as f64)
            .collect();

        match &self.ros_pub {
            Some(rpub) => {
                let mut msg = Float64MultiArray::default();
                msg.data = data.iter().map(|x| *x).collect();
                let _result = rpub.send(msg).unwrap();
            }
            _ => {}
        };

        *self.input.write().unwrap() = data;
        self.timestamp = Instant::now();
    }

    pub fn parse_dr16(&mut self, input: Vec<u8>) {
        let mut channels = vec![0, 0, 0, 0];
        channels[0] = ((input[1] as u16 & 0x07) << 8) | input[0] as u16;
        channels[1] = ((input[2] as u16 & 0xFC) << 5) | ((input[1] as u16 & 0xF8) >> 3);
        channels[2] = (((input[4] as u16 & 0x01) << 10) | ((input[3] as u16) << 2))
            | (input[2] as u16 & 0x03);
        channels[3] = ((input[5] as u16 & 0x0F) << 7) | (input[4] as u16 & 0xFE);

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

        *self.input.write().unwrap() = vec![xvel, yvel, rotv, gphi, gpsi, shoot];

        match &self.ros_pub {
            Some(rpub) => {
                let mut msg = Float64MultiArray::default();
                msg.data = vec![xvel, yvel, rotv, gphi, gpsi];
                let _result = rpub.send(msg).unwrap();
            }
            _ => {}
        };
        self.timestamp = Instant::now();
    }

    pub fn parse_rmmotor(&mut self, input: Vec<u8>) {
        let data: Vec<f64> = input
            .chunks_exact(2)
            .map(|chunk| i16::from_be_bytes(chunk.try_into().unwrap_or([0, 0])) as f64)
            .collect();

        match &self.ros_pub {
            Some(rpub) => {
                let mut msg = Float64MultiArray::default();
                msg.data = data.iter().map(|x| *x).collect();
                let _result = rpub.send(msg).unwrap();
            }
            _ => {}
        };

        *self.input.write().unwrap() = data;
        self.timestamp = Instant::now();
    }
}

pub struct MotorTable<T> {
    pub data: Vec<Arc<RwLock<Vec<T>>>>,
    pub names: Vec<String>,
}

pub struct DeviceTable {
    pub devices: Vec<Device>,
    pub dev_seek: usize,
}

impl DeviceTable {
    pub fn new() -> DeviceTable {
        DeviceTable {
            devices: Vec::new(),
            dev_seek: 0,
        }
    }

    pub fn from_yaml(filepath: String) -> DeviceTable {
        let file_data = fs::read_to_string(filepath + "/devices.yaml").unwrap();
        // let dt: DeviceTable = serde_yaml::from_str(file_data.as_str()).unwrap();
        let doc = &YamlLoader::load_from_str(file_data.as_str()).unwrap()[0];

        let mut dt = DeviceTable::new();

        // dump_node(doc, 0);

        if let Yaml::Hash(ref h) = doc {
            for (_, v) in h {
                if let Yaml::Hash(ref h) = v {
                    for (k, v) in h {
                        let devname = k;
                        let dev_yaml = v;
                        let mut dgroup = "";
                        let mut dkey = 'X';
                        let mut dopts = Vec::<u8>::new();
                        let mut dros_pub = false;
                        let mut dinput_len = 0;
                        let mut doutput_len = 0;
                        if let Yaml::Hash(ref h) = dev_yaml {
                            for (k, v) in h {
                                if let Yaml::String(pstr) = k {
                                    match pstr.as_str() {
                                        "group" => {
                                            dgroup = v.as_str().unwrap();
                                        }
                                        "key" => {
                                            dkey =
                                                v.as_str().unwrap().chars().collect::<Vec<char>>()
                                                    [0];
                                        }
                                        "options" => {
                                            dopts = v
                                                .as_vec()
                                                .unwrap()
                                                .iter()
                                                .map(|x| x.as_i64().unwrap() as u8)
                                                .collect::<Vec<u8>>();
                                        }
                                        "with_ros" => dros_pub = v.as_bool().unwrap(),
                                        "input_size" => dinput_len = v.as_i64().unwrap() as u8,
                                        "output_size" => doutput_len = v.as_i64().unwrap() as u8,

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
                            dros_pub,
                            dinput_len,
                            doutput_len,
                        );
                        dt.devices.push(device);
                    }
                }
            }
        }

        dt
    }

    // pub fn add(
    //     &mut self,
    //     device: String,
    //     device_type: String,
    //     hidkey: char,
    //     opts: Vec<u8>,
    //     with_ros: bool,
    //     input_len: u8,
    //     output_len: u8,
    // ) {
    //     let dev = Device::new(
    //         device,
    //         device_type,
    //         hidkey,
    //         opts,
    //         with_ros,
    //         input_len,
    //         output_len,
    //     );

    //     self.devices.push(dev);
    // }

    // pub fn save_state(&self) {
    //     // let filepath = "/home/mdyse/buff-code/data/test.yaml";
    //     // let mut file = File::create(filepath).unwrap();
    //     // file.write_all(serde_yaml::to_string(&self).unwrap().as_bytes());
    // }

    // pub fn generate_init(&self) -> Vec<u8> {
    //     let mut packet = Vec::new();
    //     for (i, dev) in self.devices.iter().enumerate() {
    //         packet.extend(dev.generate_init(i as u8));
    //     }

    //     packet
    // }

    pub fn set_input(&mut self, _n_bytes: u8, id: u8, input: Vec<u8>) {
        if (id as usize) < self.devices.len() {
            if self.devices[id as usize].group == "motor" {
                self.devices[id as usize].parse_rmmotor(input);
                return;
            }

            match self.devices[id as usize].name.as_str() {
                "MPU6050" => self.devices[id as usize].parse_mpu(input),
                "DR16" => self.devices[id as usize].parse_dr16(input),
                _ => {}
            }
        }
    }

    pub fn get_motor_outputs(&self) -> MotorTable<u8> {
        let mut mtable = MotorTable::<u8> {
            data: vec![],
            names: vec![],
        };

        self.devices.iter().for_each(|device| {
            if "motor" == device.group.as_str() {
                mtable.data.push(Arc::clone(&device.output));
                mtable.names.push(device.name.clone());
            }
        });

        mtable
    }

    pub fn get_motor_inputs(&self) -> MotorTable<f64> {
        let mut mtable = MotorTable::<f64> {
            data: vec![],
            names: vec![],
        };

        self.devices.iter().for_each(|device| {
            if "motor" == device.group.as_str() {
                mtable.data.push(Arc::clone(&device.input));
                mtable.names.push(device.name.clone());
            }
        });

        mtable
    }
}

// pub fn _generate_test_yaml() {
//     let robot = format!("/buffbot/self");
//     let filepath = "/home/mdyse/buff-code/data/test.yaml"; // rosrust::param(&robot).unwrap().get::<String>().unwrap();
//                                                            // let contents = fs::read_to_string(filepath + "/devices.yaml").expect("file not found");
//     let mut dt = DeviceTable::new();
//     dt.add(
//         "DR16".to_string(),
//         vec![0],
//         false,
//         18,
//         1,
//         "receiver".to_string(),
//     );
//     dt.add(
//         "MPU6050".to_string(),
//         vec![1],
//         false,
//         24,
//         1,
//         "imu".to_string(),
//     );
//     dt.add(
//         "FL_SWERVE_STEER".to_string(),
//         vec![2, 0, 0],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );
//     dt.add(
//         "FR_SWERVE_STEER".to_string(),
//         vec![2, 1, 0],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );
//     dt.add(
//         "RR_SWERVE_STEER".to_string(),
//         vec![2, 2, 0],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );
//     dt.add(
//         "RL_SWERVE_STEER".to_string(),
//         vec![2, 3, 0],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );
//     dt.add(
//         "FL_SWERVE_DRIVE".to_string(),
//         vec![2, 4, 1],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );
//     dt.add(
//         "FR_SWERVE_DRIVE".to_string(),
//         vec![2, 5, 1],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );
//     dt.add(
//         "RR_SWERVE_DRIVE".to_string(),
//         vec![2, 6, 1],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );
//     dt.add(
//         "RL_SWERVE_DRIVE".to_string(),
//         vec![2, 7, 1],
//         false,
//         6,
//         4,
//         "motor".to_string(),
//     );

//     println!("{:?}", serde_yaml::to_string(&dt).unwrap());
//     let mut file = File::create(filepath).unwrap();
//     file.write_all(serde_yaml::to_string(&dt).unwrap().as_bytes());
// }
