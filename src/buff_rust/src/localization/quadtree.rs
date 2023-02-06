use crate::localization::data_structures::*;
use image;

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
            children: vec![None, None, None, None],
        }
    }

    pub fn direction_from_angle(&self, angle: f64) -> usize {
        match angle {
            angle if angle >= std::f64::consts::PI / 2.0 => 3,
            angle if angle >= 0.0 => 0,
            angle if angle >= -std::f64::consts::PI / 2.0 => 1,
            angle if angle < -std::f64::consts::PI / 2.0 => 2,
            _ => usize::MAX,
        }
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
            (x_err / magnitude).acos() * y_err / y_err.abs() // if y is negative use the angles shadow
        }
    }

    pub fn direction_from_position(&self, position: &Vec<f64>) -> usize {
        self.direction_from_angle(self.angle_to(position))
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

    pub fn insert(&mut self, mut node: Box<QuadTreeNode>) -> bool {
        let index = self.direction_from_position(&node.position);
        if index == usize::MAX {
     		println!("Invalid insertion angle (popsibly object overlap)\n\t{:?} == {:?}", node.position, self.position);
     		return false;
     	}

        let closer_than_child =
            self.child_distance(index) >= 2.0 * self.distance_to(&node.position);

        match &mut self.children[index] {
            &mut Some(ref mut tree_node) => {
                if !closer_than_child {
                    tree_node.insert(node)
                } else {
                    node.children[index] = Some(tree_node.clone());
                    self.children[index] = Some(node);
                    true
                }
            }
            &mut None => {
                self.children[index] = Some(node);
                true
            }
        }
    }

    pub fn get_ordered_children(&self) -> Vec<usize> {
        let mut ordered_children = (0..self.children.len()).filter(|index| {
        	match &self.children[*index] {
        		Some(_) => true,
        		None => false,
        	}
        }).collect::<Vec<usize>>();

        ordered_children.sort_by_key(|i| {
        	match &self.children[*i] {
        		Some(_) => (self.child_distance(*i) * 10000.0) as i128,
        		None => i128::MAX,
        	}
        });

        ordered_children
    }

    pub fn get_ordered_children_wrt(&self, point: &Vec<f64>) -> Vec<(f64, usize)> {
        let mut ordered_children = (0..self.children.len()).map(|index| {
        	match &self.children[index] {
        		Some(tree_node) => (tree_node.distance_to(point), index),
        		None => (-1.0, index),
        	}
        }).collect::<Vec<(f64, usize)>>();

        ordered_children.sort_by_key(|(_, i)| {
        	match &self.children[*i] {
        		Some(tree_node) => (tree_node.distance_to(point) * 100000.0) as i128,
        		None => i128::MAX,
        	}
        });

        ordered_children
    }

    pub fn search_child(&self, point: &Vec<f64>, radius: &f64, index: usize) -> Vec<usize> {
        match &self.children[index] {
            Some(tree_node) => tree_node.search(point, radius),
            None => vec![],
        }
    }

    pub fn search(&self, point: &Vec<f64>, radius: &f64) -> Vec<usize> {
    	let mut self_set = false;
        let mut results = vec![];
        let x_err = point[0] - self.position[0];
        let y_err = point[1] - self.position[1];

        let magnitude = (x_err.powf(2.0) + y_err.powf(2.0)).sqrt(); // magnitude of error

        if magnitude < *radius {
        	// BFS relative to the 
            self.get_ordered_children_wrt(point)
                .into_iter()
                .filter(|(dist, index)| {
                	match &self.children[*index] {
                		Some(_) => {
                			if dist < radius {
                				true
                			}
                			else {
                				false
                			}
                		}
                		None => false
                	}
                })
                .for_each(|(dist, i)| {
                	// 4 symmetric diretions two point toward, two point away
                	if dist >= magnitude && !self_set {
                		results.push(self.index);
                		self_set = true;
                	}
                	
                	if dist > 0.0{
                		results.extend(self.search_child(point, radius, i))
                	}
                });

        	if !self_set {
        		results.push(self.index);
        	}


        } else {
            let index = self.direction_from_position(&point);
            results.extend(self.search_child(point, radius, index));
        }


        results
    }
}

impl QuadTree {
    pub fn new() -> QuadTree {
        QuadTree {
            head: None,
            objects: vec![],
        }
    }

    pub fn from_arena_image(filepath: &str) -> QuadTree {
        let qt = QuadTree::new();
        let _img = image::open(filepath).unwrap();

        qt
    }

    pub fn insert(&mut self, object: ArenaObject) -> bool {
        self.objects.push(object);
        let new_node = Box::new(QuadTreeNode::default(
            self.objects.len() - 1,
            self.objects.last().unwrap().position(),
        ));
        let head = &mut self.head;

        match head {
            &mut Some(ref mut obj) => {
                obj.insert(new_node)
            }
            &mut None => {
                self.head = Some(new_node);
                true
            }
        }
    }

    pub fn obstacles_near(&self, point: Vec<f64>, radius: f64) -> Vec<usize> {
        match &self.head {
            Some(obj) => obj.search(&point, &radius),
            None => {
                vec![]
            }
        }
    }
}
