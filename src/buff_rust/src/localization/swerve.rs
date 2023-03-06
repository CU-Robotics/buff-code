// use crate::localization::swerve::*;
// use nalgebra::Vector3;
// use std::{
//     // sync::{Arc, RwLock},
//     thread::sleep,
//     time::{Duration, Instant},
// };

// pub struct RobotStateEstimator {
//     timestamp: Instant,

//     robot_state: Body3D,

//     // motors: Arc<RwLock<MotorTable>>,
//     // imu_input: Arc<RwLock<RawInput>>,
//     // dr16_input: Arc<RwLock<RawInput>>,
//     // remote_control: Arc<RwLock<Vec<f64>>>,
//     // inertial_reference: Arc<RwLock<Vec<f64>>>,
//     // motor_reference: Arc<RwLock<Vec<f64>>>,
//     // can_sub: rosrust::Subscriber,
//     // imu_sub: rosrust::Subscriber,
//     inertial_feedback: Vector3<f64>,
//     acceleration_feedback: Vector3<f64>,
//     motor_states: Vec<Vec<f64>>,
//     tf_pub: rosrust::Publisher<tf2_msgs::TFMessage>,
// }

// impl RobotStateEstimator {
//     pub fn new(filepath: String) -> RobotStateEstimator {
//         let tf_publisher = rosrust::publish("tf", 1).unwrap();

//         // let sub = rosrust::subscribe("imu_raw", 5, move |msg: std_msgs::UInt8MultiArray| {
//         //     let mut proc_msg = std_msgs::Float64MultiArray::default();
//         //     proc_msg.data = msg
//         //         .data
//         //         .chunks_exact(4)
//         //         .map(|chunk| f32::from_be_bytes(chunk.try_into().unwrap_or([0, 0, 0, 0])) as f64)
//         //         .collect();

//         //     publisher.send(proc_msg);
//         // })
//         // .unwrap();

//         let body = Body3D::from_urdf(filepath);

//         RobotStateEstimator {
//             timestamp: Instant::now(),

//             tf_pub: tf_publisher,

//             inertial_feedback: Vector3::new(0.0, 0.0, 0.0),
//             acceleration_feedback: Vector3::new(0.0, 0.0, 0.0),
//             motor_states: vec![vec![0f64; 3]; body.frames.len()],

//             robot_state: body,
//         }
//     }

//     // pub fn get_remote_control(&self) -> Arc<RwLock<Vec<f64>>> {
//     //     self.remote_control.clone()
//     // }

//     // pub fn get_inertial_reference(&self) -> Arc<RwLock<Vec<f64>>> {
//     //     self.inertial_reference.clone()
//     // }

//     // pub fn get_motor_reference(&self) -> Arc<RwLock<Vec<f64>>> {
//     //     self.motor_reference.clone()
//     // }

//     pub fn broadcast_state(&self) {
//         let msg = tf2_msgs::TFMessage {
//             transforms: self
//                 .robot_state
//                 .frames
//                 .iter()
//                 .map(|x| x.as_tfmessages(self.timestamp.elapsed().as_micros() as u32))
//                 .flatten()
//                 .collect(),
//         };
//         self.tf_pub
//             .send(msg)
//             .expect("joint states Failed to publish");
//     }

//     pub fn spin(&mut self) {
//         let mut timestamp;

//         while rosrust::is_ok() {
//             timestamp = Instant::now();

//             // {
//             //     let motors = self.motors.read().unwrap();
//             //     let data = motors.data.clone();
//             //     let names = motors.names.clone();
//             //     let timestamp = motors.timestamp.clone();
//             //     drop(motors);

//             //     *self.motor_reference.write().unwrap() = data.iter().map(|d| d[1]).collect();
//             //     self.robot_state.set_motors(names, data, timestamp);
//             // }

//             // {
//             //     // println!("esty Taking buffer ->");
//             //     let dr16 = self.dr16_input.read().unwrap();
//             //     let data = dr16.data.clone();
//             //     let timestamp = dr16.timestamp.clone();
//             //     drop(dr16);
//             //     // println!("esty droping buffer ->");

//             //     *self.remote_control.write().unwrap() = vec![
//             //         data[0], data[1], 0.0, 0.0, 0.0, data[2], 0.0, data[3], data[4],
//             //     ];
//             // }

//             // {
//             //     // println!("esty Taking buffer ->");
//             //     let imu = self.imu_input.read().unwrap();
//             //     let data = imu.data.clone();
//             //     let timestamp = imu.timestamp.clone();
//             //     drop(imu);
//             //     // println!("esty droping buffer ->");

//             //     *self.inertial_reference.write().unwrap() = data;
//             // }

//             // self.robot_state.integrate_step();
//             self.broadcast_state();

//             if (timestamp.elapsed().as_millis() as u64) < 100 {
//                 sleep(Duration::from_millis(
//                     100 - timestamp.elapsed().as_millis() as u64,
//                 ));
//             }
//         }
//     }
// }


// use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
// use std::cmp::Ordering;
// use std::time::Instant;

// rosrust::rosmsg_include!(
//     tf2_msgs / TFMessage,
//     geometry_msgs / TransformStamped,
//     geometry_msgs / Transform
// );

// impl Eq for geometry_msgs::TransformStamped {}

// impl Ord for geometry_msgs::TransformStamped {
//     fn cmp(&self, other: &geometry_msgs::TransformStamped) -> Ordering {
//         self.header.stamp.cmp(&other.header.stamp)
//     }
// }

// impl PartialOrd for geometry_msgs::TransformStamped {
//     fn partial_cmp(&self, other: &geometry_msgs::TransformStamped) -> Option<Ordering> {
//         Some(self.header.stamp.cmp(&other.header.stamp))
//     }
// }

// #[derive(PartialEq, Debug, Clone)]
// pub struct CartesianNode {
//     pub parent: String,
//     pub children: Vec<String>,
//     pub omega: Vector3<f64>,
//     pub isometry: Isometry3<f64>,
//     pub timestamp: Instant,
// }

// impl CartesianNode {
//     pub fn new(xyz: [f64; 3], rpy: [f64; 3], pid: String, chid: Vec<String>) -> CartesianNode {
//         CartesianNode {
//             omega: Vector3::new(0.0, 0.0, 0.0), // always wrt parent frame
//             isometry: Isometry3::from_parts(
//                 Translation3::new(xyz[0], xyz[1], xyz[2]),
//                 UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
//             ),
//             parent: pid,
//             children: chid,
//             timestamp: Instant::now(),
//         }
//     }

//     pub fn default() -> CartesianNode {
//         CartesianNode {
//             omega: Vector3::new(0.0, 0.0, 0.0),
//             isometry: Isometry3::identity(),
//             parent: "base_link".to_string(),
//             children: vec![],
//             timestamp: Instant::now(),
//         }
//     }

//     pub fn as_tfmessages(&self, timestamp: u32) -> Vec<geometry_msgs::TransformStamped> {
//         self.children
//             .iter()
//             .map(|child| geometry_msgs::TransformStamped {
//                 child_frame_id: child.clone(),
//                 header: std_msgs::Header {
//                     frame_id: self.parent.clone(),
//                     stamp: rosrust::Time::from_seconds(timestamp / 1000),
//                     seq: 0,
//                 },
//                 transform: geometry_msgs::Transform {
//                     rotation: geometry_msgs::Quaternion {
//                         x: self.isometry.rotation.coords[0],
//                         y: self.isometry.rotation.coords[1],
//                         z: self.isometry.rotation.coords[2],
//                         w: self.isometry.rotation.coords[3],
//                     },
//                     translation: geometry_msgs::Vector3 {
//                         x: self.isometry.translation.vector[0],
//                         y: self.isometry.translation.vector[1],
//                         z: self.isometry.translation.vector[2],
//                     },
//                 },
//             })
//             .collect()
//     }

//     pub fn integrate(&mut self) {
//         let (r, p, y) = self.isometry.rotation.euler_angles();
//         let bomega = self.isometry.rotation.to_rotation_matrix() * self.omega;
//         println!("{:?}", self.isometry.rotation.euler_angles());
//         println!("{:?}", bomega);
//         // coords[0] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
//         //     * 0.5
//         //     * ((-coords[0] * bomega[0]) + (-coords[1] * bomega[1]) + (-coords[2] * bomega[2]));
//         // coords[1] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
//         //     * 0.5
//         //     * ((coords[3] * bomega[0]) + (-coords[2] * bomega[1]) + (coords[1] * bomega[2]));
//         // coords[2] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
//         //     * 0.5
//         //     * ((coords[2] * bomega[0]) + (coords[3] * bomega[1]) + (-coords[0] * bomega[2]));
//         // coords[3] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
//         //     * 0.5
//         //     * ((-coords[1] * bomega[0]) + (coords[0] * bomega[1]) + (coords[3] * bomega[2]));

//         let dt = self.timestamp.elapsed().as_millis() as f64 / 1000.0;
//         self.isometry.rotation = UnitQuaternion::from_euler_angles(
//             r + (bomega[0] * dt),
//             p + (bomega[1] * dt),
//             y + (bomega[2] * dt),
//         );
//         self.timestamp = Instant::now();
//         // self.isometry.rotation.coords = ;
//     }
// }

// pub struct Body3D {
//     pub frames: Vec<CartesianNode>,
//     pub names: Vec<String>,
//     pub timestamp: Instant,
// }

// impl Body3D {
//     pub fn new() -> Body3D {
//         Body3D {
//             frames: vec![],
//             names: vec![],
//             timestamp: Instant::now(),
//         }
//     }

//     pub fn from_urdf(filepath: String) -> Body3D {
//         let urdf_text =
//             urdf_rs::utils::convert_xacro_to_urdf(filepath.to_string() + "/buffbot.xacro").unwrap();
//         let robot_urdf = urdf_rs::read_from_string(&urdf_text).unwrap();

//         let mut state = Body3D::new();

//         robot_urdf.joints.iter().for_each(|j| {
//             state.frames.push(CartesianNode::new(
//                 j.origin.xyz,
//                 j.origin.rpy,
//                 j.parent.link.clone(),
//                 vec![j.child.link.clone()],
//             ));
//             state.names.push(j.name.clone());
//         });

//         rosrust::param("robot_description")
//             .unwrap()
//             .set::<String>(&urdf_text)
//             .unwrap();

//         state
//     }

//     // pub fn get_parent_isos(&self, name: String) {}

//     // pub fn update_frame_omega(&mut self, name: String, data: Vector3<f64>) {
//     //     match self.names.iter().position(|n| *n == name) {
//     //         Some(idx) => {
//     //             self.frames[idx].omega = data;
//     //         }
//     //         _ => {}
//     //     }
//     // }

//     // pub fn set_motors(&mut self, names: Vec<String>, data: Vec<Vec<f64>>, timestamp: Vec<Instant>) {
//     //     names.iter().zip(data.iter()).for_each(|(name, data)| {
//     //         self.update_frame_omega(name.to_string().clone(), Vector3::new(0.0, 0.0, data[1]));
//     //     });
//     // }

//     // pub fn set_reference_input(&self, data: Vec<f64>, timestamp: Instant) {}

//     pub fn integrate_step(&mut self) {
//         for i in 0..self.frames.len() {
//             self.frames[i].integrate();
//         }
//     }
// }
