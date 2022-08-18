use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3, Vector4};
use std::cmp::Ordering;
use std::time::Instant;

rosrust::rosmsg_include!(
    tf2_msgs / TFMessage,
    geometry_msgs / TransformStamped,
    geometry_msgs / Transform
);

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

#[derive(PartialEq, Debug, Clone)]
pub struct CartesianNode {
    pub parent: String,
    pub children: Vec<String>,
    pub omega: Vector3<f64>,
    pub isometry: Isometry3<f64>,
    pub timestamp: Instant,
}

impl CartesianNode {
    pub fn new(xyz: [f64; 3], rpy: [f64; 3], pid: String, chid: Vec<String>) -> CartesianNode {
        CartesianNode {
            omega: Vector3::new(0.0, 0.0, 0.0), // always wrt parent frame
            isometry: Isometry3::from_parts(
                Translation3::new(xyz[0], xyz[1], xyz[2]),
                UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
            ),
            parent: pid,
            children: chid,
            timestamp: Instant::now(),
        }
    }

    pub fn default() -> CartesianNode {
        CartesianNode {
            omega: Vector3::new(0.0, 0.0, 0.0),
            isometry: Isometry3::identity(),
            parent: "base_link".to_string(),
            children: vec![],
            timestamp: Instant::now(),
        }
    }

    pub fn as_tfmessages(&self, timestamp: u32) -> Vec<geometry_msgs::TransformStamped> {
        self.children
            .iter()
            .map(|child| geometry_msgs::TransformStamped {
                child_frame_id: child.clone(),
                header: std_msgs::Header {
                    frame_id: self.parent.clone(),
                    stamp: rosrust::Time::from_seconds(timestamp / 1000),
                    seq: 0,
                },
                transform: geometry_msgs::Transform {
                    rotation: geometry_msgs::Quaternion {
                        x: self.isometry.rotation.coords[0],
                        y: self.isometry.rotation.coords[1],
                        z: self.isometry.rotation.coords[2],
                        w: self.isometry.rotation.coords[3],
                    },
                    translation: geometry_msgs::Vector3 {
                        x: self.isometry.translation.vector[0],
                        y: self.isometry.translation.vector[1],
                        z: self.isometry.translation.vector[2],
                    },
                },
            })
            .collect()
    }

    pub fn integrate(&mut self) {
        let (r, p, y) = self.isometry.rotation.euler_angles();
        let bomega = self.isometry.rotation.to_rotation_matrix() * self.omega;
        println!("{:?}", self.isometry.rotation.euler_angles());
        println!("{:?}", bomega);
        // coords[0] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
        //     * 0.5
        //     * ((-coords[0] * bomega[0]) + (-coords[1] * bomega[1]) + (-coords[2] * bomega[2]));
        // coords[1] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
        //     * 0.5
        //     * ((coords[3] * bomega[0]) + (-coords[2] * bomega[1]) + (coords[1] * bomega[2]));
        // coords[2] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
        //     * 0.5
        //     * ((coords[2] * bomega[0]) + (coords[3] * bomega[1]) + (-coords[0] * bomega[2]));
        // coords[3] += (self.timestamp.elapsed().as_millis() as f64 / 1000.0)
        //     * 0.5
        //     * ((-coords[1] * bomega[0]) + (coords[0] * bomega[1]) + (coords[3] * bomega[2]));

        let dt = self.timestamp.elapsed().as_millis() as f64 / 1000.0;
        self.isometry.rotation = UnitQuaternion::from_euler_angles(
            r + (bomega[0] * dt),
            p + (bomega[1] * dt),
            y + (bomega[2] * dt),
        );
        self.timestamp = Instant::now();
        // self.isometry.rotation.coords = ;
    }
}

pub struct Body3D {
    pub frames: Vec<CartesianNode>,
    pub names: Vec<String>,
    pub timestamp: Instant,
}

impl Body3D {
    pub fn new() -> Body3D {
        Body3D {
            frames: vec![],
            names: vec![],
            timestamp: Instant::now(),
        }
    }

    pub fn from_urdf(filepath: String, debug: bool) -> Body3D {
        let urdf_text =
            urdf_rs::utils::convert_xacro_to_urdf(filepath.to_string() + "/buffbot.xacro").unwrap();
        let robot_urdf = urdf_rs::read_from_string(&urdf_text).unwrap();

        let mut state = Body3D::new();

        robot_urdf.joints.iter().for_each(|j| {
            state.frames.push(CartesianNode::new(
                j.origin.xyz,
                j.origin.rpy,
                j.parent.link.clone(),
                vec![j.child.link.clone()],
            ));
            state.names.push(j.name.clone());
        });

        if debug {
            rosrust::param("robot_description")
                .unwrap()
                .set::<String>(&urdf_text)
                .unwrap();
        }
        state
    }

    pub fn get_parent_isos(&self, name: String) {}

    pub fn update_frame_omega(&mut self, name: String, data: Vector3<f64>) {
        match self.names.iter().position(|n| *n == name) {
            Some(idx) => {
                self.frames[idx].omega = data;
            }
            _ => {}
        }
    }

    pub fn set_motors(&mut self, names: Vec<String>, data: Vec<Vec<f64>>, timestamp: Vec<Instant>) {
        names.iter().zip(data.iter()).for_each(|(name, data)| {
            self.update_frame_omega(name.to_string().clone(), Vector3::new(0.0, 0.0, data[1]));
        });
    }

    pub fn set_control_input(&self, data: Vec<f64>, timestamp: Instant) {
        // println!("{:?} {}", data, timestamp.elapsed().as_micros());
    }

    pub fn set_reference_input(&self, data: Vec<f64>, timestamp: Instant) {}

    pub fn integrate_step(&mut self) {
        for i in 0..self.frames.len() {
            self.frames[i].integrate();
        }
    }
}
