#include "hid_parser.h"



HID_Parser::HID_Parser(){
	imu = new LSM6DSOX();
	receiver = new DR16();
	rm_can_ux = new RM_CAN_Interface();
}

HID_Parser::HID_Parser(LSM6DSOX* lsm6dsox, DR16* dr16, RM_CAN_Interface* can){
	imu = lsm6dsox;
	receiver = dr16;
	rm_can_ux = can;
}

void HID_Parser::initializer_report_switch() {
	// input_report->print_packet();
	output_report->put(0, input_report.get(0));
	// Fill init device handlers
	for (size_t i = 0; i < MAX_NUM_RM_MOTORS; i++) {
		byte tmp[3];
		input_report->rgets(tmp, (3 * i) + 1, 3);
		rm_can_ux->set_index(i, tmp);

		if (rm_can_ux.motor_index[i].can_bus > 0) {
			output_report->put(rm_can_ux.motor_index[i].can_bus, (3 * i) + 1);
			output_report->put(rm_can_ux.motor_index[i].motor_type, (3 * i) + 2);
			output_report->put(rm_can_ux.motor_index[i].esc_id, (3 * i) + 3);
		}
	}
}

void HID_Parser::spin_until_initialized(){
	output_report->put(0, 255); // begin in idle status
	output_report->write();

	while (rm_can_ux.motor_index[0].can_bus < 0) {
		if (input_report->read() > 0) {
			initializer_report_switch();
		}
		delayMicroseconds(1000);
	}
}

void hid_input_switch(){
  /*
    Check if there is an HID input packet, 
    if there is one check the packet request
    and build the packet.
    @param:
      None
    @return:
      None
  */
  int32_t delta_us;
  switch (incoming_report.read()) {
    case 64:
      blink();

      switch (incoming_report.get(0)) {
        case 0:
          
          break;

        case 1:
          outgoing_report.put(0, 1);
          // Fill with can bus 1 info
          break;

        case 2:
          outgoing_report.put(0, 2);
          // Fill with can bus 2 info
          break;

        case 3:
          outgoing_report.put(0, 3);
          // Fill with DR16/IMU info (1:36)
          for (int i = 0; i < 9; i++){
            // floats take 32 bits (4 bytes), 
            // first byte of packet is an identifier
            // (4 * i) + 1 will offset accordingly
            outgoing_report.put_float((4 * i) + 1, imu.data[i]);
          }

          // DR16 data is int16_t[10] (38:58)
          for (int i = 0; i < 10; i++) {
            outgoing_report.put_float((2 * i) + 38, receiver.data[i]);
          }
          break;

        default:
          // On intial connection the HID packet is
          // set to init_mode
          outgoing_report.put(0, 255);
          break;
      }
      // clock cycles since loop timer
      delta_us = DURATION_US(hid_timer, ARM_DWT_CYCCNT);
      hid_timer = ARM_DWT_CYCCNT;

      // set the duration of the loop as the last 4 values
      // in the packet and write the packet
      outgoing_report.put_int32(60, delta_us);
      break;

    default:
      outgoing_report.put(0, 255);
      outgoing_report.put_int32(60, 0);
      return;
  }

  outgoing_report.write();
}