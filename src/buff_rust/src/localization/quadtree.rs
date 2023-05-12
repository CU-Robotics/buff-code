use crate::localization::data_structures::*;
use opencv::{core::{Mat, CV_64F}, highgui, imgproc::rectangle};
use image;

const NUM_CHILDREN: usize = 5;

#[derive(Clone)]
pub struct QuadTreeNode {
    pub index: usize,
    pub position: Vec<f64>,
    pub children: Vec<Option<Box<QuadTreeNode>>>,
}

pub struct QuadTree {
    pub head: Option<Box<QuadTreeNode>>,
    pub objects: Vec<ArenaObject>,
}

impl QuadTreeNode {
    pub fn default(index: usize, position: Vec<f64>) -> QuadTreeNode {
        QuadTreeNode {
            index: index,
            position: position,
            children: vec![None; NUM_CHILDREN],
        }
    }

    pub fn has_child(&self, index: usize) -> bool {
        match &self.children[index] {
            Some(_) => true,
            None => false,
        }
    }

    pub fn angle_to_direction(&self, angle: f64) -> usize {
        (angle / (2.0 * std::f64::consts::PI / NUM_CHILDREN as f64)) as usize
    }

    pub fn distance_to(&self, point: &Vec<f64>) -> f64 {
        // let x_err = point[0] - self.position[0];
        // let y_err = point[1] - self.position[1];

        // (x_err.powf(2.0) + y_err.powf(2.0)).sqrt() // magnitude of error
        point
            .iter()
            .zip(self.position.iter())
            .map(|(p1, p2)| (p1 - p2).powf(2.0))
            .sum::<f64>()
            .sqrt()
    }

    pub fn angle_to(&self, point: &Vec<f64>) -> f64 {
        let x_err = point[0] - self.position[0];
        let y_err = point[1] - self.position[1];

        let magnitude = (x_err.powf(2.0) + y_err.powf(2.0)).sqrt();
        if y_err == 0.0 {
            (x_err / magnitude).acos()
        } else {
            (2.0 * std::f64::consts::PI) - (x_err / magnitude).acos() // subtract from 2pi
        }
    }

    pub fn direction_to_position(&self, position: &Vec<f64>) -> usize {
        self.angle_to_direction(self.angle_to(position))
    }

    pub fn child_distance(&self, index: usize) -> f64 {
        match &self.children[index] {
            Some(tree_node) => self.distance_to(&tree_node.position),
            None => 0.0,
        }
    }

    // pub fn insert_child(&mut self, mut node: Box<QuadTreeNode>, index: usize) {
    //     let node_distance = self.distance_to(&node.position);
    //     let child_distance = self.child_distance(index);

    //     match &mut self.children[index] {
    //         &mut Some(ref mut tree_node) => {
    //         	if child_distance <= 2 * node_distance {
    //             	tree_node.insert(node);
    // 			}
    // 			else {
    // 				node.children[index] = Box::new(*tree_node);
    //     			self.children[index] = Some(node);
    // 			}
    //         }
    //         &mut None => {},
    //     };
    // }

    pub fn insert_between(&mut self, mut node: Box<QuadTreeNode>, index: usize) -> bool {
        let current_child = self.children[index].as_ref().unwrap().clone();
        let ret = node.insert(Box::new(*current_child));
        self.children[index] = Some(node);
        ret
    }

    pub fn insert(&mut self, mut node: Box<QuadTreeNode>) -> bool {
        // println!("Inserting {}", self.index);

        (0..NUM_CHILDREN).for_each(|i| {
            if node.has_child(i) {
                let child = node.children[i].as_ref().unwrap().clone();
                self.insert(child);
                node.children[i] = None;
            }
        });

        let index = self.direction_to_position(&node.position);
        if index == usize::MAX {
            println!(
                "Invalid insertion angle (popsibly object overlap)\n\t{:?} == {:?}",
                node.position, self.position
            );
            return false;
        }

        if self.has_child(index) {
            // if the node is less than half way to the child, it should become the childs parent
            if self.child_distance(index) >= 2.0 * self.distance_to(&node.position) {
                // println!("insert between {}", index);
                self.insert_between(node, index)
            } else {
                // println!("insert child {}", index);
                self.children[index].as_mut().unwrap().insert(node)
            }
        } else {
            self.children[index] = Some(node);
            // println!("new leaf {}", self.children[index].as_ref().unwrap().index);
            true
        }
    }

    pub fn get_ordered_children(&self) -> Vec<usize> {
        let mut ordered_children = (0..NUM_CHILDREN)
            .filter(|index| match &self.children[*index] {
                Some(_) => true,
                None => false,
            })
            .collect::<Vec<usize>>();

        ordered_children.sort_by_key(|i| match &self.children[*i] {
            Some(_) => (self.child_distance(*i) * 10000.0) as i128,
            None => i128::MAX,
        });

        ordered_children
    }

    pub fn get_ordered_children_wrt(&self, point: &Vec<f64>) -> Vec<(f64, usize)> {
        let mut ordered_children = (0..NUM_CHILDREN)
            .map(|index| {
                if self.has_child(index) {
                    (
                        self.children[index].as_ref().unwrap().distance_to(point),
                        index,
                    )
                } else {
                    (-1.0, index)
                }
            })
            .collect::<Vec<(f64, usize)>>();

        ordered_children.sort_by_key(|(d, _)| (d * 100000.0) as i128);

        ordered_children
    }

    pub fn search_child(&self, point: &Vec<f64>, radius: &f64, index: usize) -> Vec<usize> {
        if self.has_child(index) {
            self.children[index].as_ref().unwrap().search(point, radius)
        } else {
            vec![]
        }
    }

    pub fn search(&self, point: &Vec<f64>, radius: &f64) -> Vec<usize> {
        let mut self_set = false;
        let mut results = vec![];
        let magnitude = self.distance_to(point); // magnitude of error

        if magnitude <= *radius {
            // BFS relative to the
            self.get_ordered_children_wrt(point)
                .into_iter()
                .for_each(|(dist, index)| {
                    if dist >= 0.0 {
                        // 4 symmetric diretions two point toward, two point away
                        // remember the children are ordered here
                        // handle self result
                        if dist >= magnitude && !self_set {
                            results.push(self.index);
                            self_set = true;
                        }

                        results.extend(self.search_child(point, radius, index));
                    }
                });

            if !self_set {
                results.push(self.index);
            }
        } else {
            results.extend(self.search_child(point, radius, self.direction_to_position(&point)));
        }

        results
    }

    pub fn get_child_indices(&self) -> Vec<i16> {
        self.children
            .iter()
            .map(|child| match &child {
                &Some(_) => child.as_ref().unwrap().index as i16,
                &None => -1,
            })
            .collect()
    }

    pub fn print(&self) {
        println!("Node {} :\t{:?}", self.index, self.get_child_indices());
        (0..NUM_CHILDREN).for_each(|i| {
            if self.has_child(i) {
                self.children[i].as_ref().unwrap().print();
            }
        });
    }
}

impl QuadTree {
    pub fn new() -> QuadTree {
        // quadtree puts it self at the head initially (so self index is 0, easy to find)
        let pose = ArenaObject::new_robot(vec![0.0, 0.0, 0.0], vec![0.0, 0.0, 0.0]);

        let mut qt = QuadTree {
            head: None,
            objects: vec![],
        };

        qt.insert(pose);
        qt
    }

    pub fn from_arena_image(filepath: &str) -> QuadTree {
        let qt = QuadTree::new();
        let _img = image::open(filepath).unwrap();

        // resize image

        // iterate through pixels & and add as an object if color = (0,0,0) or 0

        // find position of obstacle using pixel (x,y) * PIXEL_TO_METERS_SCALE
        // obstacles are then PIXEL_TO_METERS_SCALE x PIXEL_TO_METERS_SCALE

        // arena positions are in meters and velocities are meters/second

        // posibly want to insert a 0 width, 0 length obstacle at center of map
        //  if this is the head the tree will be much better balanced (opposed to
        // the head on a coner or edge)

        qt
    }

    pub fn has_head(&self) -> bool {
        match self.head {
            Some(_) => true,
            None => false,
        }
    }

    pub fn centor_of_mass(&self) -> Vec<f64> {
        let n = self.objects.len() as f64;
        let mut avg = vec![0.0, 0.0, 0.0];
        self.objects.iter().for_each(|object| {
            object.position().iter().enumerate().for_each(|(i, pos)| {
                avg[i] += *pos / n;
            });
        });

        avg
    }

    pub fn distance_to_com(&self, point: &Vec<f64>) -> f64 {
        self.centor_of_mass()
            .iter()
            .zip(point.iter())
            .map(|(p1, p2)| (p1 - p2).powf(2.0))
            .sum::<f64>()
            .sqrt()
    }

    pub fn insert(&mut self, object: ArenaObject) -> bool {
        // println!("Insert called");
        self.objects.push(object);
        let new_node = Box::new(QuadTreeNode::default(
            self.objects.len() - 1,
            self.objects.last().unwrap().position(),
        ));

        if self.has_head() {
            if self.distance_to_com(&self.head.as_ref().unwrap().position)
                < new_node.distance_to(&self.centor_of_mass())
            {
                // println!("insert to head");
                self.head.as_mut().unwrap().insert(new_node)
            } else {
                // println!("insert as head");
                let head_clone = self.head.as_mut().unwrap().clone();
                self.head = Some(new_node);
                self.head.as_mut().unwrap().insert(Box::new(*head_clone))
            }
        } else {
            // println!("set to head");

            self.head = Some(new_node);
            true
        }
    }

    pub fn obstacles_near(&self, point: Vec<f64>, radius: f64) -> Vec<usize> {
        if self.has_head() {
            self.head.as_ref().unwrap().search(&point, &radius)
        } else {
            vec![]
        }
    }

    pub fn obstacles_on_line(&self, p1: Vec<f64>, p2: Vec<f64>) -> Vec<usize> {
        let radius = 0.1;
        let max_points = euclidean_distance(&p1, &p2) / (2.0 * radius);
        let search_points = (0..max_points as i8)
            .map(|i| {
                p1.iter()
                    .zip(p2.iter())
                    .map(|(p1, p2)| (i as f64 / max_points * (p2 - p1)) + p1)
                    .collect::<Vec<f64>>()
            })
            .collect::<Vec<Vec<f64>>>();

        let mut results = vec![];

        if self.has_head() {
            search_points.iter().for_each(|p1| {
                results.extend(self.head.as_ref().unwrap().search(&p1, &radius));
            });
        }

        results
    }

    pub fn get_robots_near(&self, point: Vec<f64>) -> Vec<RobotPose> {
        let mut results = vec![];
        // maybe change the search radius
        self.obstacles_near(point, 3.0)
            .iter()
            .for_each(|index| match &self.objects[*index] {
                ArenaObject::Robot(robot) => results.push(robot.clone()),
                _ => {}
            });

        results
    }

    pub fn new_tag_detection(&self) {
        // use the tag location relative to the
        // robot to assert the robots position

        // look up the detected tag
        // re-insert robot (self, 0) at tag position + offest from detection
        // do we also need to carry over detections?
    }

    // pub fn new_target_detection(&self, depth: f64, pixel_coords: Vec<i16>, camera_heading: f64) {
    //     // handle image coord projection need camera params

    //     // match point with nearby bots and choose the maximum
    //     // let closest_match = self.get_robots_near(point).iter().map(|robot| {
    //     //     robot.detection_match()
    //     // }).filter_max();
    // }

    pub fn display(&self) {
        let mut image = Mat::ones(500, 500, CV_64F).unwrap();
        self.objects.iter().for_each(|obj| match &obj {
            ArenaObject::Robot(robot) => rectangle(image, obj.position()),
            _ => {}
        });
        highgui::named_window("hello opencv!", 0).unwrap();
        highgui::imshow("hello opencv!", &image).unwrap();
        highgui::wait_key(10000).unwrap();
    }

    pub fn print(&self) {
        println!("\t===== Quadtree Dump =====");
        println!("\t----- Quadtree Objects -----");
        self.objects.iter().for_each(|obj| obj.print());
        if self.has_head() {
            println!("\t--- Quadtree Nodes ---");

            self.head.as_ref().unwrap().print();
        }
        println!("\t=============\n");
    }
}
