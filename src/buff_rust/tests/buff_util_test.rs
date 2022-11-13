#![allow(unused_imports)]
use buff_rust::buff_rust::buff_utils::*;
use rosrust_msg::std_msgs;
use std::{
    env,
    sync::{Arc, RwLock},
    time::Instant,
};

#[cfg(test)]
pub mod bu_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn load_yaml() {
        /*
            Use the penguin yaml file to test some loading functions
        */

        let byu = BuffYamlUtil::new("penguin".to_string());

        assert_eq!(byu.load_string("robot".to_string()), "infantry");

        assert_eq!(
            byu.load_string_list("motor_index".to_string()),
            vec!["xn_drive", "xp_drive", "yn_drive", "yp_drive", "yaw", "pitch", "feeder"]
        );

        assert_eq!(
            byu.load_integer_matrix("motor_can_index".to_string()),
            vec![
                vec![1, 0, 0],
                vec![1, 0, 1],
                vec![1, 0, 2],
                vec![1, 0, 3],
                vec![2, 2, 2],
                vec![2, 2, 1],
                vec![2, 0, 1]
            ]
        );
    }
}
