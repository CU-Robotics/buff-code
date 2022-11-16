#![allow(unused_imports)]
use crate::buff_util::buff_utils::*;

#[cfg(test)]
pub mod byu_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn load_yaml() {
        /*
            Use the penguin yaml file to test some loading functions
        */

        let byu = BuffYamlUtil::new("penguin");

        assert_eq!(byu.load_string("robot_type"), "infantry");

        assert_eq!(
            byu.load_string_list("motor_index"),
            vec!["xn_drive", "xp_drive", "yn_drive", "yp_drive", "yaw", "pitch", "feeder"]
        );

        assert_eq!(
            byu.load_integer_matrix("motor_can_index"),
            vec![
                vec![2, 1, 2],
                vec![2, 1, 1],
                vec![2, 0, 2],
                vec![1, 0, 3],
                vec![2, 2, 2],
                vec![2, 2, 1],
                vec![2, 0, 1]
            ]
        );
    }
}
