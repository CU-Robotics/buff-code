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
        // does put increment seek_ptr?
        assert_eq!(hid_buf.seek_ptr, 2);
        // does put actually put the data?
        assert_eq!(hid_buf.data[0], 123);
        assert_eq!(hid_buf.data[1], 234);

        for _ in 0..62 {
            hid_buf.seek(None);
        }
        // does seek wrap around?
        assert_eq!(hid_buf.seek_ptr, 0);
        // does seek return the value it seeked past?
        assert_eq!(hid_buf.seek(None), 123);
        assert_eq!(hid_buf.seek(None), 234);

        hid_buf.data[61] = 52;
        // does seek return the value it seeked past when it's passed an index?
        assert_eq!(hid_buf.seek(Some(61)), 52);
        // does seek set seek_ptr appropriately when it's passed an index?
        assert_eq!(hid_buf.seek_ptr, 62);

        hid_buf.puts(vec![6, 7, 8]);
        // does puts modify seek_ptr appropriately?
        assert_eq!(hid_buf.seek_ptr, 1);
        // does puts actually put the data there?
        assert_eq!(hid_buf.data[62], 6);
        assert_eq!(hid_buf.data[63], 7);
        assert_eq!(hid_buf.data[0], 8);

        hid_buf.seek_ptr = 59;
        // does check_of return false when it obviously doesn't overflow?
        assert_eq!(hid_buf.check_of(0), false);
        // does check_of return false when it's just one away from overflowing?
        assert_eq!(hid_buf.check_of(5), false);
        // does check_of return true when it overflows by one?
        assert_eq!(hid_buf.check_of(6), true);
        // does check_of return true when it obviously does overflow?
        assert_eq!(hid_buf.check_of(20), true);

        hid_buf.reset();
        let new_hid_buf = HidBuffer::new();
        // does resetting an HidBuffer put it in the same state as a new HidBuffer?
        assert_eq!(hid_buf.data, new_hid_buf.data);
        assert_eq!(hid_buf.update_flag, new_hid_buf.update_flag);
        assert_eq!(hid_buf.seek_ptr, new_hid_buf.seek_ptr);
    }
}
