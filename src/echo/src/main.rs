use rosrust::{self, ros_info};
use serde_derive::{Deserialize, Serialize};

#[allow(clippy::cognitive_complexity)]
fn main() {
    env_logger::init();

    // Initialize node
    rosrust::init("param_test");

    // Create parameter, go through all methods, and delete it
    let param = rosrust::param("/buffbot/DEBUG").unwrap();
    ros_info!("Handling ~foo:");
    ros_info!("Exists? {:?}", param.exists()); // false
    ros_info!("get_raw? {:?}", param.get_raw()); // false

}