#[allow(unused_imports)]
use crate::localization::{data_structures::*, quadtree::*};

#[cfg(test)]
pub mod quadtree_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn robotpose_init() {
        let pose = ArenaObject::default_robot();

        assert_eq!(pose.position(), vec![0.0, 0.0, 0.0]);
        assert_eq!(pose.velocity(), vec![0.0, 0.0, 0.0]);
        assert_eq!(pose.rotation(), vec![0.0, 0.0, 0.0]);
        assert_eq!(pose.rotational_velocity(), vec![0.0, 0.0, 0.0]);
    }

    #[test]
    pub fn robotpose_update_position() {
        let mut pose = ArenaObject::default_robot();

        // x direction
        pose.update_from_position(vec![1.0, 0.0, 0.0], vec![0.0, 0.0, 0.0], 1.0);

        assert_eq!(
            pose.position(),
            vec![1.0, 0.0, 0.0],
            "failed to update pos after x position input"
        );
        assert_eq!(
            pose.velocity(),
            vec![1.0, 0.0, 0.0],
            "failed to update vel after x position input"
        );
        assert_eq!(
            pose.rotation(),
            vec![0.0, 0.0, 0.0],
            "failed to update rot after x position input"
        );
        assert_eq!(
            pose.rotational_velocity(),
            vec![0.0, 0.0, 0.0],
            "failed to update rot vel after x position input"
        );

        pose.update_from_position(vec![0.0, 0.0, 0.0], vec![0.0, 0.0, 1.0], 1.0);

        assert_eq!(
            pose.position(),
            vec![0.0, 0.0, 0.0],
            "failed to update pos after z rotation input"
        );
        assert_eq!(
            pose.velocity(),
            vec![-1.0, 0.0, 0.0],
            "failed to update vel after z rotation input"
        );
        assert_eq!(
            pose.rotation(),
            vec![0.0, 0.0, 1.0],
            "failed to update rot after z rotation input"
        );
        assert_eq!(
            pose.rotational_velocity(),
            vec![0.0, 0.0, 1.0],
            "failed to update rot vel after z rotation input"
        );
    }

    #[test]
    pub fn robotpose_update_speed() {
        let mut pose = ArenaObject::default_robot();

        // x direction
        pose.update_from_speed(vec![1.0, 0.0, 0.0], vec![0.0, 0.0, 0.0], 1.0);

        assert_eq!(
            pose.position(),
            vec![0.5, 0.0, 0.0],
            "failed to update pos after x speed input"
        );
        assert_eq!(
            pose.velocity(),
            vec![1.0, 0.0, 0.0],
            "failed to update vel after x speed input"
        );
        assert_eq!(
            pose.rotation(),
            vec![0.0, 0.0, 0.0],
            "failed to update rot after x speed input"
        );
        assert_eq!(
            pose.rotational_velocity(),
            vec![0.0, 0.0, 0.0],
            "failed to update rot vel after x speed input"
        );

        pose.update_from_speed(vec![0.0, 0.0, 0.0], vec![0.0, 0.0, 1.0], 1.0);

        assert_eq!(
            pose.position(),
            vec![0.5, 0.0, 0.0],
            "failed to update pos after z rotational speed input"
        );
        assert_eq!(
            pose.velocity(),
            vec![0.0, 0.0, 0.0],
            "failed to update vel after z rotational speed input"
        );
        assert_eq!(
            pose.rotation(),
            vec![0.0, 0.0, 0.5],
            "failed to update rot after z rotational speed input"
        );
        assert_eq!(
            pose.rotational_velocity(),
            vec![0.0, 0.0, 1.0],
            "failed to update rot vel after z rotational speed input"
        );
    }

    #[test]
    pub fn quadtree_basic() {
        let zero = vec![0.0; 3];
        let points = vec![vec![0.0, 0.0, 0.0],
            vec![1.0, 0.0, 0.0],
            vec![0.05, 0.0, 0.0],
            vec![0.0, 10.0, 0.0],
            vec![0.0, 0.05, 0.0],
            vec![10.0, 0.0, 0.0],
            vec![-1.0, 0.0, 0.0],
            vec![0.25, 0.25, 0.0]];

        let mut qt = QuadTree::new();
        let pose = ArenaObject::new_robot(points[0].clone(), zero.clone());
        let pose1 = ArenaObject::new_robot(points[1].clone(), zero.clone());
        let pose2 = ArenaObject::new_tag(points[2].clone(), zero.clone(), 0.0);
        let pose3 = ArenaObject::new_wall(points[3].clone(), zero.clone(), 5.0, 5.0);
        let pose4 = ArenaObject::new_wall(points[4].clone(), zero.clone(), 5.0, 5.0);
        let pose5 = ArenaObject::new_robot(points[5].clone(), zero.clone());

        let mut insertion_count = 0;
        insertion_count += if qt.insert(pose) {1} else{0};
        insertion_count += if qt.insert(pose1) {1} else{0};
        insertion_count += if qt.insert(pose2) {1} else{0};
        insertion_count += if qt.insert(pose3) {1} else{0};
        insertion_count += if qt.insert(pose4) {1} else{0};
        insertion_count += if qt.insert(pose5) {1} else{0};

        // check number of elements inserted
        assert_eq!(qt.objects.len(), insertion_count, "mismatch in count of objects inserted");

        // let head = &mut qt.head;

        // match head {
        //     &mut Some(ref mut node) => {
        //         match &mut node.children[0] {
        //             &mut Some(ref mut node1) => match &node1.children[3] {
        //                 &Some(_) => {}
        //                 &None => panic!("failed to insert x2 node"),
        //             },
        //             &mut None => panic!("failed to insert x1 node"),
        //         }

        //         match &mut node.children[3] {
        //             &mut Some(_) => {}
        //             &mut None => panic!("failed to insert x3 node"),
        //         }

        //         match &mut node.children[1] {
        //             &mut Some(_) => {}
        //             &mut None => panic!("failed to insert x4 node"),
        //         }
        //     }
        //     &mut None => panic!("failed to set head"),
        // }


        {
            let radius = 100.0;
            let test_point = vec![0.0, 0.0, 0.0];
            let results = qt.obstacles_near(test_point.clone(), radius);
            let distances: Vec<f64> = points.iter().map(|point| test_point.iter().zip(point.iter()).map(|(p1, p2)| (p1 - p2).powf(2.0)).sum::<f64>().sqrt()).collect();
            assert_ne!(results.into_iter().filter(|&r| distances[r] < radius).collect::<Vec<usize>>().len(), 0, "failed to serch objects");
        }

        {
            let radius = 0.05;
            let test_point = vec![0.0, 0.0, 0.0];
            let results = qt.obstacles_near(test_point.clone(), radius);
            let distances: Vec<f64> = points.iter().map(|point| test_point.iter().zip(point.iter()).map(|(p1, p2)| (p1 - p2).powf(2.0)).sum::<f64>().sqrt()).collect();
            assert_ne!(results.into_iter().filter(|&r| distances[r] < radius).collect::<Vec<usize>>().len(), 0, "failed to serch objects");
        }

        {
            let radius = 1.0;
            let test_point = vec![1.0, 0.0, 0.0];
            let results = qt.obstacles_near(test_point.clone(), radius);
            let distances: Vec<f64> = points.iter().map(|point| test_point.iter().zip(point.iter()).map(|(p1, p2)| (p1 - p2).powf(2.0)).sum::<f64>().sqrt()).collect();
            assert_ne!(results.into_iter().filter(|&r| distances[r] < radius).collect::<Vec<usize>>().len(), 0, "failed to serch objects");
        }

        {
            let radius = 0.05;
            let test_point = vec![0.0, 10.0, 0.0];
            let results = qt.obstacles_near(test_point.clone(), radius);
            let distances: Vec<f64> = points.iter().map(|point| test_point.iter().zip(point.iter()).map(|(p1, p2)| (p1 - p2).powf(2.0)).sum::<f64>().sqrt()).collect();
            assert_ne!(results.into_iter().filter(|&r| distances[r] < radius).collect::<Vec<usize>>().len(), 0, "failed to serch objects");
        }
    }
}
