#![allow(unused_imports)]
use crate::{
    teensy_comms::data_structures::*,
    utilities::{buffers::*, loaders::*},
};
use rand::Rng;
use std::env;

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
            vec!["xn_drive", "xp_drive", "yn_drive", "yp_drive", "pitch", "yaw", "feeder"]
        );

        assert_eq!(
            byu.load_integer_matrix("motor_can_index"),
            vec![
                vec![0, 0, 0],
                vec![0, 0, 0],
                vec![0, 0, 0],
                vec![0, 0, 0],
                vec![2, 1, 5],
                vec![1, 1, 0],
                vec![2, 0, 2]
            ]
        );
    }
}

#[cfg(test)]
pub mod buffer_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn page_buffer_test() {
        /*
            test the hid buffer object
        */
        let mut page_buff = PageBuffer::new();
        page_buff.put(123);
        page_buff.put(234);
        // does put increment seek_ptr?
        assert_eq!(page_buff.seek_ptr, 2);
        // does put actually put the data?
        assert_eq!(page_buff.data[0], 123);
        assert_eq!(page_buff.data[1], 234);

        for _ in 0..62 {
            page_buff.seek(None);
        }
        // does seek wrap around?
        assert_eq!(page_buff.seek_ptr, 0);
        // does seek return the value it seeked past?
        assert_eq!(page_buff.seek(None), 123);
        assert_eq!(page_buff.seek(None), 234);

        page_buff.data[61] = 52;
        // does seek return the value it seeked past when it's passed an index?
        assert_eq!(page_buff.seek(Some(61)), 52);
        // does seek set seek_ptr appropriately when it's passed an index?
        assert_eq!(page_buff.seek_ptr, 62);

        page_buff.puts(vec![6, 7, 8]);
        // does puts modify seek_ptr appropriately?
        assert_eq!(page_buff.seek_ptr, 1);
        // does puts actually put the data there?
        assert_eq!(page_buff.data[62], 6);
        assert_eq!(page_buff.data[63], 7);
        assert_eq!(page_buff.data[0], 8);

        page_buff.seek_ptr = 59;
        // does check_of return false when it obviously doesn't overflow?
        assert_eq!(page_buff.check_of(0), false);
        // does check_of return false when it's just one away from overflowing?
        assert_eq!(page_buff.check_of(5), false);
        // does check_of return true when it overflows by one?
        assert_eq!(page_buff.check_of(6), true);
        // does check_of return true when it obviously does overflow?
        assert_eq!(page_buff.check_of(20), true);

        page_buff.reset();
        let new_page_buff = PageBuffer::new();
        assert_eq!(page_buff.data, new_page_buff.data);
        assert_eq!(page_buff.update_flag, new_page_buff.update_flag);
        assert_eq!(page_buff.seek_ptr, new_page_buff.seek_ptr);
    }

    ///
    /// Test the byte buffer
    ///
    #[test]
    pub fn basic_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: u8 = rng.gen();
        let i: usize = rng.gen_range(0..63);

        let mut buffer = ByteBuffer::new(64);
        // buffer.print_data();
        buffer.put(i, n1);
        assert_eq!(
            buffer.get(i),
            n1,
            "[{}] Failed get check {} != {}",
            i,
            n1,
            buffer.get(i)
        );
    }

    #[test]
    pub fn intermediate_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: Vec<u8> = vec![rng.gen(); 10];
        let i: usize = rng.gen_range(0..53);

        let mut buffer = ByteBuffer::new(64);
        // buffer.print_data();
        buffer.puts(i, n1.clone());

        assert_eq!(
            buffer.get(i),
            n1[0],
            "[{}] Failed get check {} != {}",
            i,
            n1[0],
            buffer.get(i)
        );
        assert_eq!(
            buffer.get(i + 1),
            n1[1],
            "[{}] Failed get check {} != {}",
            i + 1,
            n1[1],
            buffer.get(i + 1)
        );
        assert_eq!(
            buffer.get(i + 2),
            n1[2],
            "[{}] Failed get check {} != {}",
            i + 2,
            n1[2],
            buffer.get(i + 2)
        );
        assert_eq!(
            buffer.get(i + 3),
            n1[3],
            "[{}] Failed get check {} != {}",
            i + 3,
            n1[3],
            buffer.get(i + 3)
        );
    }

    #[test]
    pub fn get_float_byte_buffer() {
        let n1: Vec<u8> = vec![0x40, 0x49, 0xf, 0xdb];

        let mut buffer = ByteBuffer::new(64);
        buffer.puts(2, n1.clone());
        buffer.print_data();

        assert_eq!(
            buffer.get_float(2),
            3.1415927410125732,
            "Failed Float check {} != {}",
            3.1415927410125732,
            buffer.get_float(2)
        );
    }
}

pub mod report_tests {
    use super::*;

    #[test]
    pub fn status_report() {
        let report = RobotStatus::from_byu(BuffYamlUtil::new("penguin"));
        env::set_var("ROBOT_NAME", "penguin");
        report.save();
    }
}
