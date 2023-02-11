pub fn euclidean_distance(p1: &Vec<f64>, p2: &Vec<f64>) -> f64 {
    p1.iter()
        .zip(p2.iter())
        .map(|(p1, p2)| (p2 - p1).powf(2.0))
        .sum::<f64>()
}

#[derive(Clone)]
pub struct RobotPose {
    pub position: Vec<f64>,
    pub rotation: Vec<f64>,
    pub velocity: Vec<f64>,
    pub rotational_velocity: Vec<f64>,
}

pub struct ArucoTag {
    pub position: Vec<f64>,
    pub rotation: Vec<f64>,
    pub id: f64,
}

pub struct ArenaObstacle {
    pub position: Vec<f64>,
    pub rotation: Vec<f64>,
    pub bounds: Vec<f64>,
}

pub enum ArenaObject {
    Robot(RobotPose),
    ArucoTag(ArucoTag),
    Obstacle(ArenaObstacle),
}

impl RobotPose {
    pub fn detection_match(&self, point: Vec<f64>) -> f64 {
        point
            .iter()
            .zip(self.position.iter().zip(self.velocity.iter()))
            .map(|(p1, (p2, v))| (p1 - p2) - v)
            .sum::<f64>()
    }
}

impl ArenaObject {
    pub fn position(&self) -> Vec<f64> {
        match self {
            ArenaObject::Robot(robot_pose) => robot_pose.position.clone(),
            ArenaObject::ArucoTag(aruco_tag) => aruco_tag.position.clone(),
            ArenaObject::Obstacle(arena_obstacle) => arena_obstacle.position.clone(),
        }
    }

    pub fn rotation(&self) -> Vec<f64> {
        match self {
            ArenaObject::Robot(robot_pose) => robot_pose.rotation.clone(),
            ArenaObject::ArucoTag(aruco_tag) => aruco_tag.rotation.clone(),
            ArenaObject::Obstacle(arena_obstacle) => arena_obstacle.rotation.clone(),
        }
    }

    pub fn velocity(&self) -> Vec<f64> {
        match self {
            ArenaObject::Robot(robot_pose) => robot_pose.velocity.clone(),
            ArenaObject::ArucoTag(_) => vec![0.0, 0.0, 0.0],
            ArenaObject::Obstacle(_) => vec![0.0, 0.0, 0.0],
        }
    }

    pub fn rotational_velocity(&self) -> Vec<f64> {
        match self {
            ArenaObject::Robot(robot_pose) => robot_pose.rotational_velocity.clone(),
            ArenaObject::ArucoTag(_) => vec![0.0, 0.0, 0.0],
            ArenaObject::Obstacle(_) => vec![0.0, 0.0, 0.0],
        }
    }

    pub fn default_robot() -> ArenaObject {
        ArenaObject::Robot(RobotPose {
            position: vec![0.0; 3],
            rotation: vec![0.0; 3],
            velocity: vec![0.0; 3],
            rotational_velocity: vec![0.0; 3],
        })
    }

    pub fn new_robot(xyz: Vec<f64>, rpy: Vec<f64>) -> ArenaObject {
        ArenaObject::Robot(RobotPose {
            position: xyz,
            rotation: rpy,
            velocity: vec![0.0; 3],
            rotational_velocity: vec![0.0; 3],
        })
    }

    pub fn new_wall(xyz: Vec<f64>, rpy: Vec<f64>, width: f64, length: f64) -> ArenaObject {
        ArenaObject::Obstacle(ArenaObstacle {
            position: xyz,
            rotation: rpy,
            bounds: vec![width, length],
        })
    }

    pub fn new_tag(xyz: Vec<f64>, rpy: Vec<f64>, id: f64) -> ArenaObject {
        ArenaObject::ArucoTag(ArucoTag {
            position: xyz,
            rotation: rpy,
            id: id,
        })
    }

    pub fn update_from_position(&mut self, xyz: Vec<f64>, rpy: Vec<f64>, dt: f64) {
        match self {
            ArenaObject::Robot(robot_pose) => {
                robot_pose.velocity = xyz
                    .iter()
                    .zip(robot_pose.position.iter())
                    .map(|(new_pos, pos)| (new_pos - pos) / dt)
                    .collect();
                robot_pose.rotational_velocity = rpy
                    .iter()
                    .zip(robot_pose.rotation.iter())
                    .map(|(new_pos, pos)| (new_pos - pos) / dt)
                    .collect();
                robot_pose.position = xyz;
                robot_pose.rotation = rpy;
            }
            ArenaObject::ArucoTag(_) => {}
            ArenaObject::Obstacle(_) => {}
        }
    }

    pub fn update_from_speed(&mut self, dxyz: Vec<f64>, drpy: Vec<f64>, dt: f64) {
        // Using eulers backwrds & forwards
        match self {
            ArenaObject::Robot(robot_pose) => {
                robot_pose.position = dxyz
                    .iter()
                    .zip(robot_pose.velocity.iter())
                    .map(|(new_vel, vel)| (new_vel + vel) * dt / 2.0)
                    .collect();
                robot_pose.rotation = drpy
                    .iter()
                    .zip(robot_pose.rotational_velocity.iter())
                    .map(|(new_vel, vel)| (new_vel + vel) * dt / 2.0)
                    .collect();
                robot_pose.velocity = dxyz;
                robot_pose.rotational_velocity = drpy;
            }
            ArenaObject::ArucoTag(_) => {}
            ArenaObject::Obstacle(_) => {}
        }
    }

    pub fn print(&self) {
        match self {
            ArenaObject::Robot(robot_pose) => println!("Robot: {:?}", robot_pose.position),
            ArenaObject::ArucoTag(aruco_tag) => println!("Tag: {:?}", aruco_tag.position),
            ArenaObject::Obstacle(arena_obstacle) => {
                println!("Wall: {:?}", arena_obstacle.position)
            }
        }
    }
}
