#![allow(unused_imports)]
use crate::teensy_comms::buff_hid::*;
use std::{
    env,
    sync::{Arc, RwLock},
    time::Instant,
};

#[cfg(test)]
pub mod dead_teensy_comms_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn read_write_test() {
        /*
            Attempt to connect and read/write from teensy
        */

        let mut layer = HidLayer::new();

        let t = Instant::now();
        while t.elapsed().as_secs() < 5 {
            layer.read();
            layer.input.print_data();
            layer.write();
        }
    }
}
