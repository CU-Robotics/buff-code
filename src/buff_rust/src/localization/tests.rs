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
        let mut qt = QuadTree::new();
        let pose = ArenaObject::default_robot();
        let pose1 = ArenaObject::new_robot(vec![1.0, 0.0, 0.0], vec![0.0, 0.0, 0.0]);
        let pose2 = ArenaObject::new_tag(vec![0.5, 0.0, 0.0], vec![0.0, 0.0, 0.0], 0.0);
        let pose3 = ArenaObject::new_wall(vec![0.0, 12.0, 0.0], vec![0.0, 0.0, 0.0], 5.0, 5.0);
        let pose4 = ArenaObject::new_wall(vec![0.5, -10.0, 0.0], vec![0.0, 0.0, 0.0], 5.0, 5.0);
        let pose5 = ArenaObject::new_robot(vec![0.25, 0.25, 0.0], vec![0.0, 0.0, 0.0]);

        qt.insert(pose);
        qt.insert(pose1);
        qt.insert(pose2);
        qt.insert(pose3);
        qt.insert(pose4);
        qt.insert(pose5);

        let head = &mut qt.head;

        match head {
            &mut Some(ref mut node) => {
                match &mut node.children[0] {
                    &mut Some(ref mut node1) => match &node1.children[3] {
                        &Some(_) => {}
                        &None => panic!("failed to insert x2 node"),
                    },
                    &mut None => panic!("failed to insert x1 node"),
                }

                match &mut node.children[3] {
                    &mut Some(_) => {}
                    &mut None => panic!("failed to insert x3 node"),
                }

                match &mut node.children[1] {
                    &mut Some(_) => {}
                    &mut None => panic!("failed to insert x4 node"),
                }
            }
            &mut None => panic!("failed to set head"),
        }

        assert!(qt.obstacles_near(vec![0.0, 0.0, 0.0], 100.0).len() == 6);
        assert_eq!(qt.obstacles_near(vec![0.0, 0.0, 0.0], 100.0), vec![0, 1, 2, 5, 4, 3]);

        assert!(qt.obstacles_near(vec![0.0, 12.0, 0.0], 1.0).len() == 1);
        assert_eq!(qt.obstacles_near(vec![0.0, 12.0, 0.0], 1.0), vec![3]);

        assert!(qt.obstacles_near(vec![0.0, -10.0, 0.0], 1.0).len() == 1);
        assert_eq!(qt.obstacles_near(vec![0.0, -10.0, 0.0], 1.0), vec![4]);
    }
}
