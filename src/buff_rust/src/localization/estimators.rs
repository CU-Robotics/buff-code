// use rosrust_msg::std_msgs;
// use std::{
//     sync::{Arc, RwLock},
//     time::Instant,
// };

// // use image::{GenericImage, GenericImageView, ImageBuffer, RgbImage};

// use crate::utilities::loaders::*;

// pub struct RobotStateEstimator {
//     pub kinematic_matrix: Vec<Vec<f64>>,
//     pub state: Vec<f64>,
//     pub trajectory: Vec<f64>,
//     pub motor_hist: Vec<f64>,
//     pub motor_feedback: Vec<Arc<RwLock<Vec<f64>>>>,
//     pub subscribers: Vec<rosrust::Subscriber>,
//     pub state_publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
//     pub trajectory_publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
// }

// impl RobotStateEstimator {
//     pub fn new() -> RobotStateEstimator {
//         let byu = BuffYamlUtil::from_self();
//         // motor config info (only needed for name indexing)
//         let k_mat = byu.load_float_matrix("kinematic_matrix");
//         // let state_ref = byu.load_string("velocity_state_feedback");

//         let n = k_mat[0].len();
//         let motor_feedback = vec![Arc::new(RwLock::new(vec![0f64; 3])); n];

//         let subs = (0..n)
//             .map(|i| {
//                 let ref_clone = motor_feedback[i].clone();
//                 rosrust::subscribe(
//                     format!("motor_{}_feedback", i).as_str(),
//                     1,
//                     move |msg: std_msgs::Float64MultiArray| {
//                         *ref_clone.write().unwrap() = msg.data;
//                     },
//                 )
//                 .unwrap()
//             })
//             .collect();

//         let publ1 = rosrust::publish("robot_state", 1).unwrap();
//         let publ2 = rosrust::publish("robot_trajectory", 1).unwrap();

//         RobotStateEstimator {
//             kinematic_matrix: k_mat,
//             state: vec![0f64; 6],
//             trajectory: vec![0f64; 6],
//             motor_hist: vec![0f64; n],
//             motor_feedback: motor_feedback,
//             subscribers: subs,
//             state_publisher: publ1,
//             trajectory_publisher: publ2,
//         }
//     }

//     pub fn get_motor_angles(&mut self) -> Vec<f64> {
//         self.motor_feedback
//             .iter()
//             .map(|fb| fb.read().unwrap()[0])
//             .collect()
//     }

//     pub fn get_motor_speeds(&mut self) -> Vec<f64> {
//         self.motor_feedback
//             .iter()
//             .map(|fb| fb.read().unwrap()[1])
//             .collect()
//     }

//     // pub fn accel_from_encoder_speed(&mut self) -> Vec<f64> {
//     //     let vel = self.get_motor_speeds();
//     //     let accel = vel
//     //         .iter()
//     //         .zip(self.motor_hist.iter())
//     //         .map(|(v, h)| (v - h) / (self.rate as f64))
//     //         .collect();
//     //     self.motor_hist = vel;
//     //     accel
//     // }

//     // pub fn publish_accel(&mut self) {
//     //     let mut msg = std_msgs::Float64MultiArray::default();
//     //     msg.data = self.accel_from_encoder_speed();
//     //     self.accel_publisher.send(msg).unwrap();
//     // }

//     pub fn update_kinematics(&mut self) -> Vec<f64> {
//         let v = self.get_motor_speeds();

//         self.kinematic_matrix
//             .iter()
//             .map(|a| a.iter().zip(v.iter()).map(|(a, x)| a * x).sum::<f64>())
//             .collect()
//     }

//     // pub fn update_deadreackoning(&mut self, dt: f64) -> (Vec<f64>, Vec<f64>) {
//     //     self.velocity = self.update_kinematics();

//     //     self.postion = self.position.iter().zip(self.velocity.iter()).map(|(pos, vel)| pos + (vel * dt)).collect();

//     //     (self.postion, self.velocity)
//     // }

//     // pub fn update_inertial(&mut self) -> Vec<f64> {
//     //     // let v = self.get_inertias();

//     //     self.kinematic_matrix
//     //         .iter()
//     //         .map(|a| a.iter().zip(v.iter()).map(|(a, x)| a * x).sum::<f64>())
//     //         .collect()
//     // }

//     pub fn update(&mut self) {}

//     pub fn publish_state(&self) {
//         let mut msg = std_msgs::Float64MultiArray::default();
//         msg.data = self.state.clone();
//         self.trajectory_publisher.send(msg).unwrap();
//     }

//     pub fn spin(&mut self) {
//         let mut timestamp;

//         while rosrust::is_ok() {
//             timestamp = Instant::now();

//             self.trajectory = self.update_kinematics();
//             self.publish_trajectory();

//             while timestamp.elapsed().as_millis() < 10 {}
//         }
//     }
// }

// // // pub struct ArenaStateEstimator {
// // //     pub rate: u128,
// // //     pub map: ImageBuffer,
// // //     pub enemy_robots: Vec<f64>,
// // //     pub buff_robots: Vec<f64>,
// // //     pub camera_transform: Vec<f64>,
// // //     pub gimbal_heading: Vec<f64>,
// // //     pub subscribers: Vec<rosrust::Subscriber>,
// // //     pub state_publisher: rosrust::Publisher<std_msgs::Float64MultiArray>,
// // // }

// // // // Construct a new RGB ImageBuffer with the specified width and height.
// // // let img: RgbImage = ImageBuffer::new(512, 512);,
