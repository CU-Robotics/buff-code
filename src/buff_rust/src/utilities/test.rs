#![allow(unused_imports)]
use crate::utilities::{buffers::*, loaders::*};
use rand::Rng;

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
                vec![2, 0, 2],
                vec![2, 1, 1],
                vec![2, 0, 0],
                vec![2, 2, 2],
                vec![2, 2, 1],
                vec![2, 0, 1]
            ]
        );
    }
}

#[cfg(test)]
pub mod buffer_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    ///
    /// Test the byte buffer
    ///
    #[test]
    pub fn basic_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: u8 = rng.gen();
        let i;
        u8 = rng.gen_range(0..63);

        let buffer = ByteBuffer::new();
        buffer.print_data();
        buffer.put(i, n1);
        assert_eq!(
            buffer.get(i),
            n1,
            format("[{}] Failed check {} != {}", i, n1, buffer.get(i))
        );
    }

    #[test]
    pub fn intermediate_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: u8 = vec![rng.gen(); 10];
        let i;
        u8 = rng.gen_range(0..53);

        let buffer = ByteBuffer::new();
        buffer.print_data();
        buffer.puts(i, n1);
        assert_eq!(
            buffer.get(i),
            n1[0],
            format("[{}] Failed check {} != {}", i, n1[0], buffer.get(i))
        );
        assert_eq!(
            buffer.get(i + 1),
            n1[1],
            format(
                "[{}] Failed check {} != {}",
                i + 1,
                n1[1],
                buffer.get(i + 1)
            )
        );
        assert_eq!(
            buffer.get(i + 2),
            n1[2],
            format(
                "[{}] Failed check {} != {}",
                i + 2,
                n1[2],
                buffer.get(i + 2)
            )
        );
        assert_eq!(
            buffer.get(i + 3),
            n1[3],
            format(
                "[{}] Failed check {} != {}",
                i + 3,
                n1[3],
                buffer.get(i + 3)
            )
        );
    }
}
