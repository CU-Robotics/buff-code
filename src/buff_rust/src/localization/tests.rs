#[allow(unused_imports)]
use crate::localization::{data_structures::*, quadtree::*};
#[allow(unused_imports)]
use std::time::Instant;

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

    pub fn insert_with_debug(qt: &mut QuadTree, pose: ArenaObject) -> usize {
        let t = Instant::now();
        let ret = qt.insert(pose);
        println!("Insert duration: {} us", t.elapsed().as_micros());
        if ret {
            return 1;
        }

        return 0;
    }

    // basic list search for nearby points
    pub fn get_nearby_points(point: &Vec<f64>, radius: &f64, points: &Vec<Vec<f64>>) -> Vec<usize> {
        (0..points.len())
            .filter(|i| {
                points[*i]
                    .iter()
                    .zip(point.iter())
                    .map(|(p1, p2)| (p1 - p2).powf(2.0))
                    .sum::<f64>()
                    .sqrt()
                    <= *radius
            })
            .collect()
    }

    pub fn sort_by_distance(point: &Vec<f64>, points: &Vec<Vec<f64>>) -> Vec<usize> {
        let mut sorted = (0..points.len()).collect::<Vec<usize>>();

        sorted.sort_by_key(|i| {
            points[*i]
                .iter()
                .zip(point.iter())
                .map(|(p1, p2)| ((p1 - p2).powf(2.0) * 10000.0) as i128)
                .sum::<i128>()
        });

        sorted
    }

    pub fn combine_nearby_and_sorted(
        nearby_points: &Vec<usize>,
        sorted_points: Vec<usize>,
    ) -> Vec<usize> {
        sorted_points
            .into_iter()
            .filter(|sorted| {
                nearby_points
                    .iter()
                    .map(|p| if *sorted == *p { 1 } else { 0 })
                    .sum::<i16>()
                    > 0
            })
            .collect()
    }

    pub fn test_search(
        qt: &mut QuadTree,
        test_point: &Vec<f64>,
        radius: f64,
        all_points: &Vec<Vec<f64>>,
    ) {
        let mut t = Instant::now();
        let results = qt.obstacles_near(test_point.clone(), radius);
        println!(
            "Quadtree found {} objects in {} us",
            results.len(),
            t.elapsed().as_micros()
        );

        let distances: Vec<f64> = all_points
            .iter()
            .map(|point| {
                test_point
                    .iter()
                    .zip(point.iter())
                    .map(|(p1, p2)| (p1 - p2).powf(2.0))
                    .sum::<f64>()
                    .sqrt()
            })
            .collect();

        t = Instant::now();
        let nearby_test = get_nearby_points(&test_point, &radius, &all_points);
        let sorted_test = sort_by_distance(&test_point, &all_points);
        let test_results = combine_nearby_and_sorted(&nearby_test, sorted_test.clone());
        println!(
            "Test found {} objects in {} us",
            test_results.len(),
            t.elapsed().as_micros()
        );

        assert_ne!(
            results
                .clone()
                .into_iter()
                .filter(|&r| distances[r] <= radius)
                .collect::<Vec<usize>>()
                .len(),
            0,
            "failed to find objects within {} meters of {:?}",
            radius,
            test_point
        );

        assert_eq!(
            results, test_results,
            "basic search and quadtree search returned different results\n\t {:?} {}",
            test_point, radius
        );

        // results are not exactly the same just check sum of the indices
        assert_eq!(
            results.iter().sum::<usize>(),
            test_results.iter().sum::<usize>(),
            "basic search and quadtree search returned different results\n\t {:?} {}",
            test_point,
            radius
        );
    }

    #[test]
    pub fn quadtree_basic() {
        let zero = vec![0.0; 3];
        let points = vec![
            vec![0.0, 0.0, 0.0],
            vec![1.0, 1.0, 0.0],
            vec![0.05, 10.0, 0.0],
            vec![10.0, 10.0, 0.0],
            vec![-10.0, 0.05, 0.0],
            vec![10.0, 1.0, 0.0],
            vec![-1.5, 1.0, 0.0],
            vec![0.25, 0.25, 0.0],
        ];

        let mut qt = QuadTree::new();
        // objects[0] is self, make a mock point to account for it
        // let pose = ArenaObject::new_robot(points[0].clone(), zero.clone());
        let pose1 = ArenaObject::new_robot(points[1].clone(), zero.clone());
        let pose2 = ArenaObject::new_tag(points[2].clone(), zero.clone(), 0.0);
        let pose3 = ArenaObject::new_wall(points[3].clone(), zero.clone(), 5.0, 5.0);
        let pose4 = ArenaObject::new_wall(points[4].clone(), zero.clone(), 5.0, 5.0);
        let pose5 = ArenaObject::new_robot(points[5].clone(), zero.clone());
        let pose6 = ArenaObject::new_robot(points[6].clone(), zero.clone());
        let pose7 = ArenaObject::new_robot(points[7].clone(), zero.clone());

        // count insertions, the constructor inserts self at 0
        // self doesn't necesarily stay as the head
        let mut insertion_count = 1;
        // insertion_count += if qt.insert(pose) { 1 } else { 0 };
        insertion_count += insert_with_debug(&mut qt, pose1);
        insertion_count += insert_with_debug(&mut qt, pose2);
        insertion_count += insert_with_debug(&mut qt, pose3);
        insertion_count += insert_with_debug(&mut qt, pose4);
        insertion_count += insert_with_debug(&mut qt, pose5);
        insertion_count += insert_with_debug(&mut qt, pose6);
        insertion_count += insert_with_debug(&mut qt, pose7);

        qt.print();

        // check number of elements inserted
        assert_eq!(
            qt.objects.len(),
            insertion_count,
            "mismatch in count of objects inserted"
        );

        // large search radii can break the search, needs investigation.
        // maybe rebuild the tree with the search point as the head
        // this way the search will have shortest path from search point to
        // every node. Also increasing the number of children leads to
        // less traversing but paths can lead to very similar endpoints.
        // with a large radius the endpoints of two branches can be in the
        // same search, but may be out of order.
        points.iter().for_each(|point| {
            test_search(&mut qt, point, 0.01, &points);
            test_search(&mut qt, point, 0.5, &points);
            test_search(&mut qt, point, 5.0, &points);
        });

        // test_search(&mut qt, &points[3], 10.0, &points);

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
    }

    #[test]
    pub fn quadtree_display() {
        let zero = vec![0.0; 3];
        let points = vec![
            vec![0.0, 0.0, 0.0],
            vec![1.0, 1.0, 0.0],
            vec![0.05, 10.0, 0.0],
            vec![10.0, 10.0, 0.0],
            vec![-10.0, 0.05, 0.0],
            vec![10.0, 1.0, 0.0],
            vec![-1.5, 1.0, 0.0],
            vec![0.25, 0.25, 0.0],
        ];

        let mut qt = QuadTree::new();
        // objects[0] is self, make a mock point to account for it
        // let pose = ArenaObject::new_robot(points[0].clone(), zero.clone());
        let pose1 = ArenaObject::new_robot(points[1].clone(), zero.clone());
        let pose2 = ArenaObject::new_tag(points[2].clone(), zero.clone(), 0.0);
        let pose3 = ArenaObject::new_wall(points[3].clone(), zero.clone(), 5.0, 5.0);
        let pose4 = ArenaObject::new_wall(points[4].clone(), zero.clone(), 5.0, 5.0);
        let pose5 = ArenaObject::new_robot(points[5].clone(), zero.clone());
        let pose6 = ArenaObject::new_robot(points[6].clone(), zero.clone());
        let pose7 = ArenaObject::new_robot(points[7].clone(), zero.clone());

        // count insertions, the constructor inserts self at 0
        // self doesn't necesarily stay as the head
        let mut insertion_count = 1;
        // insertion_count += if qt.insert(pose) { 1 } else { 0 };
        insertion_count += insert_with_debug(&mut qt, pose1);
        insertion_count += insert_with_debug(&mut qt, pose2);
        insertion_count += insert_with_debug(&mut qt, pose3);
        insertion_count += insert_with_debug(&mut qt, pose4);
        insertion_count += insert_with_debug(&mut qt, pose5);
        insertion_count += insert_with_debug(&mut qt, pose6);
        insertion_count += insert_with_debug(&mut qt, pose7);

        qt.print();

        let t = Instant::now();
        while t.elapsed().as_secs() < 5 {
            qt.display()
        }
    }
}
