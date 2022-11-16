#![allow(unused_imports)]
use crate::teensy_comms::buff_hid::*;
use std::{
    env,
    sync::{Arc, RwLock},
    time::Instant,
};

#[cfg(test)]
pub mod dead_comms_test {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn hid_buffer_test() {
        /*
            test the hid buffer object
        */
        let mut hid_buf = HidBuffer::new();
        hid_buf.put(123);
        hid_buf.put(234);
        assert_eq!(hid_buf.seek_ptr, 2);
        assert_eq!(hid_buf.data[0], 123);
        assert_eq!(hid_buf.data[1], 234);

        for i in 0..62 {
            hid_buf.seek(None);
        }
        assert_eq!(hid_buf.seek_ptr, 0);
        assert_eq!(hid_buf.seek(None), 123);
        assert_eq!(hid_buf.seek(None), 234);

        hid_buf.seek(Some(61));
        assert_eq!(hid_buf.seek_ptr, 62);

        hid_buf.puts(vec![6, 7, 8]);
        assert_eq!(hid_buf.seek_ptr, 1);
        assert_eq!(hid_buf.data[62], 6);
        assert_eq!(hid_buf.data[63], 7);
        assert_eq!(hid_buf.data[0], 8);

        hid_buf.seek_ptr = 59;
        assert_eq!(hid_buf.check_of(0), false);
        // fails; bug?
        assert_eq!(hid_buf.check_of(5), false);
        assert_eq!(hid_buf.check_of(6), true);
        assert_eq!(hid_buf.check_of(20), true);
    }
}
