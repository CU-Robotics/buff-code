rosrust::rosmsg_include!(
    tf2_msgs / TFMessage,
    geometry_msgs / TransformStamped,
    geometry_msgs / Transform
);

use crate::device::MotorTable;
use nalgebra::UnitQuaternion;
use rosrust_msg::sensor_msgs::JointState;
use std::cmp::Ordering;
use std::time::Duration;
use std::{thread::sleep, time::Instant};

impl Eq for geometry_msgs::TransformStamped {}

impl Ord for geometry_msgs::TransformStamped {
    fn cmp(&self, other: &geometry_msgs::TransformStamped) -> Ordering {
        self.header.stamp.cmp(&other.header.stamp)
    }
}

impl PartialOrd for geometry_msgs::TransformStamped {
    fn partial_cmp(&self, other: &geometry_msgs::TransformStamped) -> Option<Ordering> {
        Some(self.header.stamp.cmp(&other.header.stamp))
    }
}

pub fn to_transform_stamped(
    joint: &urdf_rs::Joint,
    motors: &MotorTable<f64>,
    timestamp: &Instant,
) -> geometry_msgs::TransformStamped {
    let chid = joint.child.link.clone();
    let pid = joint.parent.link.clone();

    let q;

    if joint.name == "swerve_xy" {
        q = UnitQuaternion::from_euler_angles(
            joint.origin.rpy[0],
            joint.origin.rpy[1],
            joint.origin.rpy[2],
        );
    } else {
        let motor_index = motors.names.iter().position(|r| *r == joint.name).unwrap();
        let data = motors.data[motor_index].read().unwrap();

        if pid == "base_link" {
            q = UnitQuaternion::from_euler_angles(
                joint.origin.rpy[0],
                joint.origin.rpy[1],
                joint.origin.rpy[2] + (data[0] / 20000.0),
            );
        } else {
            if motor_index == 7 {
                println!("{}", data[0] / 20000.0);
            }
            q = UnitQuaternion::from_euler_angles(
                joint.origin.rpy[0],
                joint.origin.rpy[1] + (data[0] / 20000.0),
                joint.origin.rpy[2],
            );
        }
        // if motor_index == 0 {
        //     println!("{}", data[0]);
        // }
    }

    geometry_msgs::TransformStamped {
        child_frame_id: chid,
        header: std_msgs::Header {
            frame_id: pid,
            stamp: rosrust::Time::from_seconds(timestamp.elapsed().as_micros() as u32 / 1000),
            seq: 0,
        },
        transform: geometry_msgs::Transform {
            rotation: geometry_msgs::Quaternion {
                x: q.coords[0],
                y: q.coords[1],
                z: q.coords[2],
                w: q.coords[3],
            },
            translation: geometry_msgs::Vector3 {
                x: joint.origin.xyz[0],
                y: joint.origin.xyz[1],
                z: joint.origin.xyz[2],
            },
        },
    }
}

// pub struct TF {
// 	name: String,
// 	xyz: Point3<f64>,
// 	q: Quaternion,
// }

pub struct RobotState {
    timestamp: Instant,
    robot: urdf_rs::Robot,
    motors: MotorTable<f64>,
    js_pub: Option<rosrust::Publisher<JointState>>,
    tf_pub: Option<rosrust::Publisher<tf2_msgs::TFMessage>>,
}

impl RobotState {
    pub fn new(filepath: String, motortable: MotorTable<f64>) -> RobotState {
        let urdf_text =
            urdf_rs::utils::convert_xacro_to_urdf(filepath.to_string() + "/buffbot.xacro").unwrap();
        let robot_urdf = urdf_rs::read_from_string(&urdf_text).unwrap();

        let dbg = format!("/buffbot/buff_rust/debug");
        let debug = true; //rosrust::param(&dbg).unwrap().get::<bool>().unwrap();

        let mut js_publisher = None;
        let mut tf_publisher = None;

        if debug {
            rosrust::param("robot_description")
                .unwrap()
                .set::<String>(&urdf_text)
                .unwrap();

            // js_publisher = Some(rosrust::publish("joint_states", 1).unwrap());
            tf_publisher = Some(rosrust::publish("tf", 1).unwrap());
        }

        RobotState {
            timestamp: Instant::now(),
            robot: robot_urdf,
            motors: motortable,
            js_pub: js_publisher,
            tf_pub: tf_publisher,
        }
    }

    pub fn broadcast_tfs(&self) {
        let msg = tf2_msgs::TFMessage {
            transforms: self
                .robot
                .joints
                .iter()
                .map(|x| to_transform_stamped(x, &self.motors, &self.timestamp))
                .collect(),
        };

        if let Some(ros_pub) = &self.tf_pub {
            ros_pub.send(msg).expect("joint states Failed to publish");
        }
    }

    pub fn broadcast_state(&self) {
        let mut xs = Vec::<f64>::new();
        let mut vs = Vec::<f64>::new();
        let mut ts = Vec::<f64>::new();

        let mut names = Vec::<String>::new();

        self.motors.names.iter().enumerate().for_each(|(i, n)| {
            let data = self.motors.data[i].read().unwrap();
            if data.len() == 3 {
                xs.push(data[0]);
                vs.push(data[1]);
                ts.push(data[2]);
                names.push(n.to_string());
            }
        });

        let mut msg = rosrust_msg::sensor_msgs::JointState::default();
        msg.name = names;
        msg.position = xs;
        msg.velocity = vs;
        msg.effort = ts;

        if let Some(ros_pub) = &self.js_pub {
            ros_pub.send(msg).expect("joint states Failed to publish");
        }
    }

    pub fn spin(&mut self) {
        let mut timestamp;

        while rosrust::is_ok() {
            timestamp = Instant::now();

            if let Some(_) = &self.tf_pub {
                // self.broadcast_state();
                self.broadcast_tfs()
            }

            if (timestamp.elapsed().as_millis() as u64) < 100 {
                sleep(Duration::from_millis(
                    100 - timestamp.elapsed().as_millis() as u64,
                ));
            }
        }
    }
}
