#![allow(unused_imports)]
extern crate hidapi;
use hidapi::{HidApi, HidDevice};

use crate::{
    teensy_comms::{buff_hid::*, data_structures::*},
    utilities::loaders::*,
};

use std::{
    env,
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

#[allow(dead_code)]
const VERBOSITY: usize = 1;

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

        if mode == 255 && VERBOSITY > 0 {
            println!(
                "\t[{}] Config Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        } else if mode == 0 && VERBOSITY > 0 {
            println!(
                "\t[{}] Idle Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        } else if VERBOSITY > 0 {
            println!(
                "\t[{}] Data Packet detected: <{}, {} us>",
                sys_time.elapsed().as_millis(),
                reader.input.get(0),
                timer
            );
        }

        if timer == 0 && VERBOSITY > 0 {
            println!("\tTeensy does not recognize the connection!");
        }

        return mode;
    }

    pub fn watch_for_packet(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut loopt;
        let mode = 0;
        let teensy_f = 1.0 / 600000000.0;
        let t = Instant::now();
        let mut teensy_elapsed_time;

        while t.elapsed().as_millis() < timeout {
            loopt = Instant::now();

            match reader.read() {
                64 => {
                    teensy_elapsed_time =
                        teensy_f * (reader.input.get_i32(60) as f64 - teensy_cyccnt);
                    teensy_cyccnt = reader.input.get_i32(60) as f64;

                    if reader.input.get(0) == 0 && teensy_cyccnt == 0.0 {
                        continue;
                    } else if teensy_elapsed_time > 0.001 {
                        if VERBOSITY > 0 {
                            println!(
                                "\tTeensy cycle time is over the limit {}",
                                teensy_elapsed_time
                            );
                        }
                    }

                    if reader.input.get(0) == packet_id {
                        if VERBOSITY > 1 {
                            println!("\tTeenys report {} reply received", packet_id);
                        }
                        return teensy_cyccnt;
                    }
                }
                0 => {
                    if VERBOSITY > 0 {
                        println!("\tNo Packet available");
                    }
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

        return teensy_cyccnt;
    }

    pub fn watch_for_packet_data(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        index: usize,
        n: usize,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut sum = -1.0;
        let t = Instant::now();

        teensy_cyccnt = watch_for_packet(reader, packet_id, timeout, teensy_cyccnt);

        if validate_input_packet(reader, t) == packet_id {
            sum = reader
                .input
                .gets(index, n)
                .into_iter()
                .map(|x| x as f64)
                .sum::<f64>();
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
        return teensy_cyccnt;
    }

    pub fn watch_for_report(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        report: Vec<u8>,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut sum = -1.0;
        let t = Instant::now();

        teensy_cyccnt = watch_for_packet(reader, packet_id, timeout, teensy_cyccnt);

        if validate_input_packet(reader, t) == packet_id {
            sum = reader
                .input
                .data
                .iter()
                .zip(report.iter())
                .map(|(x, r)| (*x - *r) as f64)
                .sum::<f64>();
        }

        assert!(
            sum >= 0.0,
            "\t[{}] Requested Teensy Packet does not match\n\n",
            t.elapsed().as_millis()
        );

        return teensy_cyccnt;
    }

    pub fn watch_for_no_packet_data(
        reader: &mut HidReader,
        packet_id: u8,
        timeout: u128,
        index: usize,
        n: usize,
        mut teensy_cyccnt: f64,
    ) -> f64 {
        let mut sum = -1.0;
        let t = Instant::now();

        teensy_cyccnt = watch_for_packet(reader, packet_id, timeout, teensy_cyccnt);

        if validate_input_packet(reader, t) == packet_id {
            sum = reader
                .input
                .gets(index, n)
                .into_iter()
                .map(|x| x as f64)
                .sum::<f64>();
        }

        assert!(
            sum == 0.0,
            "\t[{}] Requested Teensy Packet was empty\n\n",
            t.elapsed().as_millis()
        );
        assert!(
            sum != 0.0,
            "\t[{}] Requested  Packet was not empty\n\n",
            t.elapsed().as_millis()
        );

        return teensy_cyccnt;
    }

    pub fn packet_request_test(
        reader: &mut HidReader,
        writer: &mut HidWriter,
        packet_id: u8,
        data: Vec<u8>,
        teensy_cyccnt: f64,
    ) -> f64 {
        if VERBOSITY > 1 {
            println!("Testing packet request {}:...", packet_id);
        }
        writer.send_report(packet_id, data);
        return watch_for_packet(reader, packet_id, 5, teensy_cyccnt);
    }

    pub fn initializer_test(reader: &mut HidReader, writer: &mut HidWriter) {
        let mut robot_status = RobotStatus::new("penguin");
        let initializers = robot_status.load_initializers();
        let mut teensy_cyccnt = 0.0;
        println!("\nTesting initializers:...");
        initializers.iter().for_each(|init| {
            writer.send_report(INITIALIZER_REPORT_ID, init[1..].to_vec());
            teensy_cyccnt = watch_for_report(
                reader,
                INITIALIZER_REPORT_ID,
                5,
                init[..3].to_vec(),
                teensy_cyccnt,
            );
        });
    }

    pub fn imu_connection_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting imu access:...\n");
        writer.send_report(SENSOR_REPORT_ID, vec![0]);
        watch_for_packet_data(reader, SENSOR_REPORT_ID, 5, 2, 36, 0.0);
        println!("IMU data: {:?}", reader.input.get_floats(2, 9));
    }

    pub fn dr16_connection_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting dr16 access:...\n");
        writer.send_report(SENSOR_REPORT_ID, vec![2]);
        watch_for_packet_data(reader, SENSOR_REPORT_ID, 5, 2, 24, 0.0);
        println!("DR16 data {:?}", reader.input.get_floats(2, 6));
    }

    pub fn motor_feedback_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting motor feedback:...\n");
        writer.send_report(MOTOR_REPORT_ID, vec![1]);
        watch_for_packet_data(reader, MOTOR_REPORT_ID, 10, 2, 49, 0.0);
        println!("Motor 5 data: {:?}", reader.input.get_floats(14, 3));
    }

    pub fn can_control_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting can control:...\n");
        let bytes = f32::to_be_bytes(0.2).to_vec();
        // set up control output for motor 4 & 5
        writer.send_report(
            CONTROLLER_REPORT_ID,
            vec![
                MOTOR_INIT_SWITCH_MODE,
                0,
                bytes[0],
                bytes[1],
                bytes[2],
                bytes[3],
                bytes[0],
                bytes[1],
                bytes[2],
                bytes[3],
            ],
        );
        watch_for_packet(reader, CONTROLLER_REPORT_ID, 5, 0.0);

        let t = Instant::now();
        while t.elapsed().as_secs() < 1 {}

        // zero things out
        writer.send_report(CONTROLLER_REPORT_ID, vec![MOTOR_INIT_SWITCH_MODE, 0]);
    }

    pub fn gimbal_input_control_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting gimbal input control:...\n");
        let mut t = Instant::now();
        let mut teensy_cyccnt = 0.0;
        while t.elapsed().as_secs() < 3 {}
        // set up control output for motor 4, 5 & 6
        writer.output.puts(0, vec![CONTROLLER_REPORT_ID, 2]);
        writer.output.put_float(3, 1.0);
        writer.output.put_float(7, 0.0);
        writer.output.put_float(11, 0.0);
        writer.write();
        teensy_cyccnt = watch_for_packet(reader, CONTROLLER_REPORT_ID, 5, teensy_cyccnt);

        t = Instant::now();
        while t.elapsed().as_secs() < 3 {}

        // check chassis controllers
        writer.send_report(CONTROLLER_REPORT_ID, vec![CONTROLLER_REQUEST_SWITCH_MODE]);
        teensy_cyccnt =
            watch_for_no_packet_data(reader, CONTROLLER_REPORT_ID, 5, 3, 48, teensy_cyccnt);

        // check gimbal controllers
        writer.send_report(
            CONTROLLER_REPORT_ID,
            vec![CONTROLLER_REQUEST_SWITCH_MODE, 1],
        );
        teensy_cyccnt =
            watch_for_packet_data(reader, CONTROLLER_REPORT_ID, 5, 15, 12, teensy_cyccnt);

        // zero things out
        writer.send_report(CONTROLLER_REPORT_ID, vec![2]);
        watch_for_packet(reader, CONTROLLER_REPORT_ID, 5, teensy_cyccnt);
    }

    pub fn latency_test(reader: &mut HidReader, writer: &mut HidWriter) {
        println!("\nTesting latency alignment:...\n");
        let t = Instant::now();
        // mock request for controller 0 & 1 status
        let mut teensy_cyccnt = 0.0;
        while t.elapsed().as_millis() < 900 {
            teensy_cyccnt = packet_request_test(
                reader,
                writer,
                CONTROLLER_REPORT_ID,
                vec![CONTROLLER_REQUEST_SWITCH_MODE, 0, 0, 1],
                teensy_cyccnt,
            )
        }
        // mock request for controller 2 & 3 status
        while t.elapsed().as_millis() < 900 {
            teensy_cyccnt = packet_request_test(
                reader,
                writer,
                CONTROLLER_REPORT_ID,
                vec![CONTROLLER_REQUEST_SWITCH_MODE, 0, 2, 3],
                teensy_cyccnt,
            )
        }
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

        latency_test(&mut reader, &mut writer);
        // imu_connection_test(&mut reader, &mut writer);
        // dr16_connection_test(&mut reader, &mut writer);
        // motor_feedback_test(&mut reader, &mut writer);
        // can_control_test(&mut reader, &mut writer);
        // gimbal_input_control_test(&mut reader, &mut writer);
    }
}
