#![allow(unused_imports)]
use crate::teensy_comms::buff_hid::*;
use std::{
    env,
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

#[cfg(test)]
pub mod dead_teensy_comms_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    pub fn write_with_debug(layer: &mut HidLayer, sys_time: Instant) {
        let m = layer.output.get(0);
        let t = Instant::now();
        layer.write();
        layer.output.print_data();

        println!(
            "\t[{}] Layer write success: <{}, {} us>",
            sys_time.elapsed().as_millis(),
            m,
            t.elapsed().as_micros()
        );
    }

    pub fn read_with_debug(layer: &mut HidLayer, sys_time: Instant) -> usize {
        let n = layer.read();
        layer.input.print_data();
        println!(
            "\t[{}] Layer read success: <{}, {} us>",
            sys_time.elapsed().as_millis(),
            layer.input.get(0),
            layer.input.get_i32(60)
        );
        return n;
    }

    pub fn validate_input_packet(layer: &mut HidLayer, sys_time: Instant) -> u8 {
        let mode = layer.input.get(0);
        let timer = layer.input.get_i32(60);

        if mode == 255 {
            println!(
                "\t[{}] Config Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                layer.input.get(0),
                timer
            );
        } 
        else if mode == 0 {
            println!(
                "\t[{}] Idle Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                layer.input.get(0),
                timer
            );
        }
        else {
            println!(
                "\t[{}] Data Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                layer.input.get(0),
                timer
            );
        }

        if timer == 0 {
            println!("\tTeensy does not recognize the connection!");
        }

        return mode;
    }

    pub fn watch_for_packet(layer: &mut HidLayer, packet_id: u8, timeout: u128) {
        let mut loopt;
        let mut mode = 0;
        let t = Instant::now();

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match layer.read() {
                64 => {
                    if validate_input_packet(layer, t) == packet_id {
                        mode = packet_id;
                        layer.input.print_data();
                    }
                }
                0 => {
                    println!("\tNo Packet available");
                }
                _ => {
                    panic!("\tCorrupt packet!");
                }
            }

            layer.set_output_bytes(0, vec![0]);
            layer.write();

            while loopt.elapsed().as_micros() < 1000 {}
        }

        assert_eq!(
            packet_id,
            mode,
            "\t[{}] Requested Teensy Packet not found\n\n",
            t.elapsed().as_millis()
        );
        println!(
            "\t[{}] Requested Teensy Packet found\n\n",
            t.elapsed().as_millis()
        )
    }

    pub fn watch_for_packet_data(
        layer: &mut HidLayer,
        packet_id: u8,
        timeout: u128,
        index: usize,
        n: usize,
    ) {
        let mut loopt;
        let mut sum = 0.0;
        let t = Instant::now();

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match layer.read() {
                64 => {
                    if validate_input_packet(layer, t) == packet_id {
                        sum = layer
                            .input
                            .gets(index, n)
                            .into_iter()
                            .map(|x| x as f64)
                            .sum::<f64>();
                        layer.input.print_data();
                    }
                }
                0 => {
                    println!("\tNo Packet available");
                }
                _ => {
                    panic!("\tCorrupt packet!");
                }
            }

            layer.set_output_bytes(0, vec![0]);
            layer.write();

            while loopt.elapsed().as_micros() < 1000 {}
        }

        assert!(
            sum != 0.0,
            "\t[{}] Requested Teensy Data Empty\n\n",
            t.elapsed().as_millis()
        );
        println!(
            "\t[{}] Requested Teensy Data found\n\n",
            t.elapsed().as_millis()
        )
    }

    pub fn packet_request_test(layer: &mut HidLayer, packet_id: u8) {
        println!("Testing packet request {}:...", packet_id);
        layer.set_output_bytes(0, vec![packet_id]);
        layer.write();
        watch_for_packet(layer, packet_id, 6);
    }

    pub fn initializer_test(layer: &mut HidLayer) {
        let initializers = layer.robot_report.load_initializers();
        println!("\nTesting initializers:...\n\t{:?}", initializers);
        layer.set_output_bytes(0, vec![255]);
        layer.set_output_bytes(1, initializers[0].clone());
        layer.write();
        layer.output.print_data();
        watch_for_packet(layer, 255, 5);
    }

    pub fn imu_connection_test(layer: &mut HidLayer) {
        println!("\nTesting sensor access:...\n");
        layer.set_output_bytes(0, vec![3]);
        layer.write();
        watch_for_packet_data(layer, 3, 5, 1, 36);
    }

    pub fn motor_feedback_test(layer: &mut HidLayer) {
        println!("\nTesting motor feedback:...\n");
        layer.set_output_bytes(0, vec![1, 0]);
        layer.write();
        watch_for_packet_data(layer, 1, 10, 1, 24);
    }

    #[test]
    pub fn hid_comms_tests() {
        /*
            Attempt to connect and read/write from teensy
        */

        let mut layer = HidLayer::new();

        layer.init_comms();

        initializer_test(&mut layer);

        packet_request_test(&mut layer, 1);
        packet_request_test(&mut layer, 2);

        imu_connection_test(&mut layer);

        motor_feedback_test(&mut layer);

    }
}
