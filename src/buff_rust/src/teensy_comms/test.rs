#![allow(unused_imports)]
extern crate hidapi;
use hidapi::{HidApi, HidDevice};

use crate::{
    teensy_comms::buff_hid::*,
    utilities::{data_structures::*, loaders::*},
};

use std::{
    env,
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

#[cfg(test)]
pub mod comms_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    pub fn write_with_debug(writer: &mut HidWriter, sys_time: Instant) {
        let m = writer.output.get(0);
        let t = Instant::now();
        writer.write();
        writer.output.print_data();

        println!(
            "\t[{}] Writer success: <{}, {} us>",
            sys_time.elapsed().as_millis(),
            m,
            t.elapsed().as_micros()
        );
    }

    pub fn read_with_debug(reader: &mut HidReader, sys_time: Instant) -> usize {
        let n = reader.read();
        reader.input.print_data();
        println!(
            "\t[{}] Reader success: <{}, {} us>",
            sys_time.elapsed().as_millis(),
            reader.input.get(0),
            reader.input.get_i32(60)
        );
        return n;
    }

    pub fn validate_input_packet(reader: &mut HidReader, sys_time: Instant) -> u8 {
        let mode = reader.input.get(0);
        let timer = reader.input.get_i32(60);

        if mode == 255 {
            println!(
                "\t[{}] Config Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        } else if mode == 0 {
            println!(
                "\t[{}] Idle Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        } else {
            println!(
                "\t[{}] Data Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        }

        if timer == 0 {
            println!("\tTeensy does not recognize the connection!");
        }

        return mode;
    }

    pub fn watch_for_packet(reader: &mut HidReader, packet_id: u8, timeout: u128) {
        let mut loopt;
        let mode = 0;
        let t = Instant::now();

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match reader.read() {
                64 => {
                    if reader.input.get(0) == 0 && reader.input.get_i32(60) == 0 {
                        continue;
                    } else if reader.input.get_i32(60) > 1000 {
                        println!("\tTeensy cycle time is over the limit");
                    }

                    if reader.input.get(0) == packet_id {
                        println!("\tTeenys report {} reply received", packet_id);
                        return;
                    }
                }
                0 => {
                    println!("\tNo Packet available");
                }
                _ => {
                    panic!("\tCorrupt packet!");
                }
            }

            // writer.write();
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
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        index: usize,
        n: usize,
    ) {
        let mut loopt;
        let mut sum = -1.0;
        let t = Instant::now();

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match reader.read() {
                64 => {
                    if reader.input.get(0) == 0 && reader.input.get_i32(60) == 0 {
                        continue;
                    } else if reader.input.get_i32(60) > 1000 {
                        println!("\tTeensy cycle time is over the limit");
                    }

                    if validate_input_packet(reader, t) == packet_id {
                        sum = reader
                            .input
                            .gets(index, n)
                            .into_iter()
                            .map(|x| x as f64)
                            .sum::<f64>();
                        // reader.input.print_data();
                        break;
                    }
                }
                0 => {
                    println!("\tNo Packet available");
                }
                _ => {
                    panic!("\tCorrupt packet!");
                }
            }

            // writer.write();
            while loopt.elapsed().as_micros() < 1000 {}
        }

        assert!(
            sum >= 0.0,
            "\t[{}] Requested Teensy Packet not found\n\n",
            t.elapsed().as_millis()
        );
        assert!(
            sum > 0.0,
            "\t[{}] Requested Teensy Data Empty\n\n",
            t.elapsed().as_millis()
        );
        println!(
            "\t[{}] Requested Teensy Data found\n\n",
            t.elapsed().as_millis()
        )
    }

    pub fn watch_for_no_packet_data(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        index: usize,
        n: usize,
    ) {
        let mut loopt;
        let mut sum = -1.0;
        let t = Instant::now();

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match reader.read() {
                64 => {
                    if reader.input.get(0) == 0 && reader.input.get_i32(60) == 0 {
                        continue;
                    } else if reader.input.get_i32(60) > 1000 {
                        println!("\tTeensy cycle time is over the limit");
                    }

                    if validate_input_packet(reader, t) == packet_id {
                        sum = reader
                            .input
                            .gets(index, n)
                            .into_iter()
                            .map(|x| x as f64)
                            .sum::<f64>();
                        // reader.input.print_data();
                        break;
                    }
                }
                0 => {
                    println!("\tNo Packet available");
                }
                _ => {
                    panic!("\tCorrupt packet!");
                }
            }

            // writer.write();
            while loopt.elapsed().as_micros() < 1000 {}
        }

        assert!(
            sum >= 0.0,
            "\t[{}] Requested Teensy Packet not found\n\n",
            t.elapsed().as_millis()
        );
        assert!(
            sum == 0.0,
            "\t[{}] Requested Teensy Data not empty\n\n",
            t.elapsed().as_millis()
        );
        println!(
            "\t[{}] Requested Teensy Data empty\n\n",
            t.elapsed().as_millis()
        )
    }

    pub fn packet_request_test(reader: &mut HidReader, writer: &mut HidWriter, packet_id: u8) {
        println!("Testing packet request {}:...", packet_id);
        writer.send_report(packet_id, vec![0]);
        watch_for_packet(reader, packet_id, 5);
    }

    pub fn initializer_test(reader: &mut HidReader, writer: &mut HidWriter) {
        let mut robot_status = BuffBotStatusReport::new("penguin");
        let initializers = robot_status.load_initializers();
        println!("\nTesting initializers:...");
        initializers.iter().for_each(|init| {
            // println!("{:?}", init);
            writer.send_report(255, init[1..].to_vec());
            watch_for_packet(reader, 255, 5);
        });
    }

    pub fn imu_connection_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting imu access:...\n");
        writer.send_report(3, vec![0]);
        watch_for_packet_data(reader, 3, 5, 2, 36);
        println!("IMU data: {:?}", reader.input.get_floats(2, 9));
    }

    pub fn dr16_connection_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting dr16 access:...\n");
        writer.send_report(3, vec![1]);
        watch_for_packet_data(reader, 3, 5, 2, 24);
        println!("DR16 data {:?}", reader.input.get_floats(2, 6));
    }

    pub fn motor_feedback_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting motor feedback:...\n");
        writer.send_report(1, vec![1]);
        watch_for_packet_data(reader, 1, 10, 2, 49);
        println!("Motor 5 data: {:?}", reader.input.get_floats(14, 3));
    }

    pub fn can_control_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting can control:...\n");
        let bytes = f32::to_be_bytes(0.2).to_vec();
        // set up control output for motor 4 & 5
        writer.send_report(
            2,
            vec![
                0, 1, bytes[0], bytes[1], bytes[2], bytes[3], bytes[0], bytes[1], bytes[2],
                bytes[3],
            ],
        );
        watch_for_packet(reader, 2, 5);

        let t = Instant::now();
        while t.elapsed().as_secs() < 1 {}

        // zero things out
        writer.send_report(2, vec![0, 1]);
    }

    pub fn gimbal_input_control_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting gimbal input control:...\n");
        let mut t = Instant::now();
        while t.elapsed().as_secs() < 3 {}
        // set up control output for motor 4, 5 & 6
        writer.output.puts(0, vec![2, 2]);
        writer.output.put_float(3, 500.0);
        writer.output.put_float(7, 500.0);
        writer.output.put_float(11, 500.0);
        writer.write();
        watch_for_packet(reader, 2, 5);

        t = Instant::now();
        while t.elapsed().as_secs() < 3 {}

        // check chassis controllers
        writer.send_report(2, vec![1]);
        watch_for_no_packet_data(reader, 2, 5, 3, 48);

        // check gimbal controllers
        writer.send_report(2, vec![1, 1]);
        watch_for_packet_data(reader, 2, 5, 15, 12);

        // zero things out
        writer.send_report(2, vec![2]);
        watch_for_packet(reader, 2, 5);
    }

    #[test]
    pub fn teensy_test() {
        /*
            Attempt to connect and read/write from teensy
        */

        let byu = BuffYamlUtil::new("penguin");

        let vid = byu.load_u16("teensy_vid");
        let pid = byu.load_u16("teensy_pid");

        let mut hidapi = HidApi::new().expect("Failed to create API instance");
        let mut reader = HidReader::new(&mut hidapi, vid, pid);
        let mut writer = HidWriter::new(&mut hidapi, vid, pid);

        initializer_test(&mut reader, &mut writer);

        packet_request_test(&mut reader, &mut writer, 1);
        // imu_connection_test(&mut reader, &mut writer);
        // dr16_connection_test(&mut reader, &mut writer);
        motor_feedback_test(&mut reader, &mut writer);
        // can_control_test(&mut reader, &mut writer);
        gimbal_input_control_test(&mut reader, &mut writer);
    }
}
