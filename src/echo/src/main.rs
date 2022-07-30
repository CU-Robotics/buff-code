// use rosrust::{self, ros_info};
// use serde_derive::{Deserialize, Serialize};

// #[allow(clippy::cognitive_complexity)]
// fn main() {
//     env_logger::init();

//     // Initialize node
//     rosrust::init("param_test");

//     // Create parameter, go through all methods, and delete it
//     let param = rosrust::param("/buffbot/DEBUG").unwrap();
//     ros_info!("Handling ~foo:");
//     ros_info!("Exists? {:?}", param.exists()); // false
//     ros_info!("get_raw? {:?}", param.get_raw()); // false

// }

extern crate hidapi;


fn main() {
    let api = hidapi::HidApi::new().unwrap();
    // Print out information about all connected devices
    for device in api.device_list() {
        println!("{:#?}", device);
    }

    // Connect to device using its VID and PID
    let (VID, PID) = (1452, 657);
    let device = api.open(VID, PID).unwrap();

    // Read data from device
    let mut buf = [0u8; 8];
    let res = device.read(&mut buf[..]).unwrap();
    println!("Read: {:?}", &buf[..res]);

    // Write data to device
    let buf = [0u8, 1, 2, 3, 4];
    let res = device.write(&buf).unwrap();
    println!("Wrote: {:?} byte(s)", res);
}