use image;
use crate::localization::data_structures::*;

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
            _ => panic!("invalid angle for insertion"),
        }
    }

    pub fn distance_to(&self, point: &Vec<f64>) -> f64 {
    	let x_err = point[0] - self.position[0];
        let y_err = point[1] - self.position[1];

        (x_err.powf(2.0) + y_err.powf(2.0)).sqrt() // magnitude of error
    }

    pub fn angle_to(&self, point: &Vec<f64>) -> f64 {
    	let x_err = point[0] - self.position[0];
    	let y_err = point[1] - self.position[1];
    	
    	let magnitude = (x_err.powf(2.0) + y_err.powf(2.0)).sqrt();
        if y_err == 0.0 {
            (x_err / magnitude).acos()
        } else {
            (x_err / magnitude).acos() * y_err / y_err.abs()       // if y is negative use the angles shadow
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

    pub fn insert_child(&mut self, mut node: Box<QuadTreeNode>, index: usize) {
        let node_distance = self.distance_to(&node.position);
        let child_distance = self.child_distance(index);

        match &mut self.children[index] {
            &mut Some(ref mut tree_node) => {
            	if child_distance < node_distance {
	            	tree_node.insert(node)	
	            	return;				
				}
            }
            &mut None => {},
        };

		node.children[index] = self.children[index];
        self.children[index] = Some(node)   
    }

    pub fn insert(&mut self, node: Box<QuadTreeNode>) {
        // println!("inserting {:?}", object.position);
        // dot product of error and zero vec is cos^-1(x / magnitude)
        let dir = self.direction_from_position(&node.position);
        self.insert_child(node, dir);
    }

    pub fn search_child(&self, position: &Vec<f64>, radius: &f64, index: usize) -> Vec<usize> {
    	match &self.children[index] {
            Some(tree_node) => tree_node.search(position, radius),
            None => vec![],
        }
    }

    pub fn search(&self, position: &Vec<f64>, radius: &f64) -> Vec<usize> {
    	let mut results = vec![];
    	let x_err = position[0] - self.position[0];
        let y_err = position[1] - self.position[1];

        let magnitude = (x_err.powf(2.0) + y_err.powf(2.0)).sqrt(); // magnitude of error

        if magnitude < *radius {
        	results.push(self.index);
        	results.extend(self.search_child(position, radius, 0));
        	results.extend(self.search_child(position, radius, 1));
        	results.extend(self.search_child(position, radius, 2));
        	results.extend(self.search_child(position, radius, 3));
        }
        else {
	        let index = self.direction_from_position(&position);
	        results.extend(self.search_child(position, radius, index));
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
        let img = image::open(filepath).unwrap();

        qt
    }

    pub fn insert(&mut self, object: ArenaObject) {
        self.objects.push(object);
        let new_node = Box::new(QuadTreeNode::default(
            self.objects.len() - 1,
            self.objects.last().unwrap().position(),
        ));
        let head = &mut self.head;

        match head {
            &mut Some(ref mut obj) => {
                obj.insert(new_node);
            }
            &mut None => {
                self.head = Some(new_node);
            }
        }
    }

    pub fn obstacles_near(&self, point: Vec<f64>, radius: f64) -> Vec<usize> {
    	match &self.head {
            Some(obj) => {
                obj.search(&point, &radius)
            }
            None => {
                vec![]
            }
        }
    }
}
