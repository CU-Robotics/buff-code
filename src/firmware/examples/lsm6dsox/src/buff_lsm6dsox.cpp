#include "buff_lsm6dsox.h"

Buff_LSM6DSOX::Buff_LSM6DSOX() {
	/*
		Set up the jawns and the jimmys, config is hardcoded atm
		but that can change to use a more global definition.
	*/	

	// LSM6DSOX Setup
	lsm6dsox.begin_I2C();
	lsm6dsox.setAccelRange(IMU_A_RANGE);
	lsm6dsox.setGyroRange(IMU_G_RANGE);
	lsm6dsox.setAccelDataRate(IMU_A_DATA_RATE);
	lsm6dsox.setGyroDataRate(IMU_G_DATA_RATE);

	/*!
		@brief Sets the INT1 and INT2 pin activation mode
		@param active_low true to set the pins  as active high, false to set the
		mode to active low
		@param open_drain true to set the pin mode as open-drain, false to set the
		mode to push-pull
	*/
	lsm6dsox.configIntOutputs(false, true);

	/*!
		@brief Enables and disables the data ready interrupt on INT 1.
		@param drdy_temp true to output the data ready temperature interrupt
		@param drdy_g true to output the data ready gyro interrupt
		@param drdy_xl true to output the data ready accelerometer interrupt
		@param step_detect true to output the step detection interrupt (default off)
		@param wakeup true to output the wake up interrupt (default off)
	*/
	lsm6dsox.configInt1(false, false, false, false, false);

	/*!
		@brief Enables and disables the data ready interrupt on INT 2.
		@param drdy_temp true to output the data ready temperature interrupt
		@param drdy_g true to output the data ready gyro interrupt
		@param drdy_xl true to output the data ready accelerometer interrupt
	*/
	lsm6dsox.configInt2(false, false, false);

	// LIS3MDL Setup
	lis3mdl.begin_I2C();
	lis3mdl.setRange(IMU_M_RANGE);
	lis3mdl.setDataRate(IMU_M_DATA_RATE);
	lis3mdl.setPerformanceMode(IMU_M_PERFORMANCE);
	lis3mdl.setOperationMode(IMU_M_OP_MODE);
	
	lis3mdl.configInterrupt(false, false, false, // enable z axis
											true, // polarity
											false, // don't latch
											true); // enabled!
}

// void Buff_LSM6DSOX::read_lsm6dsox(){
// 	/*
// 		Get the jawns from the jimmys
// 	*/
// 	// unsigned long read_start = micros();
// 	// if (lsm6dsox.accelerationAvailable()){
// 	// 	lsm6dsox.readAcceleration(data[0], data[1], data[2]);
// 	// }
// 	lsm6dsox._read();
// 	// data[0] = 
// 	// data[1] = 
// 	// data[2] = 

// 	// Serial.print("LSM6DSOX accel read time: "); Serial.println(micros() - read_start);
// }

// void Buff_LSM6DSOX::read_lsm6dsox(){
//   // get raw readings
//   Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_OUTX_L_G, 12);

//   uint8_t raw_buffer[14];
//   data_reg.read(raw_buffer, 14);

//   rawTemp = raw_buffer[1] << 8 | raw_buffer[0];
//   temperature = (rawTemp / temperature_sensitivity) + 25.0;

//   data[3] = raw_buffer[3] << 8 | raw_buffer[2];
//   data[4] = raw_buffer[5] << 8 | raw_buffer[4];
//   data[5] = raw_buffer[7] << 8 | raw_buffer[6];

//   data[0] = raw_buffer[9] << 8 | raw_buffer[8];
//   data[1] = raw_buffer[11] << 8 | raw_buffer[10];
//   data[2] = raw_buffer[13] << 8 | raw_buffer[12];

//   float gyro_scale = 1; // range is in milli-dps per bit!
//   switch (gyroRangeBuffered) {
//   case ISM330DHCX_GYRO_RANGE_4000_DPS:
//     gyro_scale = 140.0;
//     break;
//   case LSM6DS_GYRO_RANGE_2000_DPS:
//     gyro_scale = 70.0;
//     break;
//   case LSM6DS_GYRO_RANGE_1000_DPS:
//     gyro_scale = 35.0;
//     break;
//   case LSM6DS_GYRO_RANGE_500_DPS:
//     gyro_scale = 17.50;
//     break;
//   case LSM6DS_GYRO_RANGE_250_DPS:
//     gyro_scale = 8.75;
//     break;
//   case LSM6DS_GYRO_RANGE_125_DPS:
//     gyro_scale = 4.375;
//     break;
//   }

//   gyroX = rawGyroX * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
//   gyroY = rawGyroY * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
//   gyroZ = rawGyroZ * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;

//   float accel_scale = 1; // range is in milli-g per bit!
//   switch (accelRangeBuffered) {
//   case LSM6DS_ACCEL_RANGE_16_G:
//     accel_scale = 0.488;
//     break;
//   case LSM6DS_ACCEL_RANGE_8_G:
//     accel_scale = 0.244;
//     break;
//   case LSM6DS_ACCEL_RANGE_4_G:
//     accel_scale = 0.122;
//     break;
//   case LSM6DS_ACCEL_RANGE_2_G:
//     accel_scale = 0.061;
//     break;
//   }

//   accX = rawAccX * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
//   accY = rawAccY * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
//   accZ = rawAccZ * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
// }

void Buff_LSM6DSOX::read_lsm6dsox_accel(){
	/*
		Get the jawns from the jimmys
	*/
	// unsigned long read_start = micros();
	// if (lsm6dsox.accelerationAvailable()){
	// 	lsm6dsox.readAcceleration(buffer[0], buffer[1], buffer[2]);
	// }
	lsm6dsox.readAcceleration(data[0], data[1], data[2]);

	// Serial.print("LSM6DSOX accel read time: "); Serial.println(micros() - read_start);
}

void Buff_LSM6DSOX::read_lsm6dsox_gyro(){
	/*
		Get the jawns from the jimmys
	*/
	// unsigned long read_start = micros();
	// if (lsm6dsox.gyroscopeAvailable()){
	// 	lsm6dsox.readGyroscope(data[3], data[4], data[5]);
	// }
	lsm6dsox.readGyroscope(data[3], data[4], data[5]);
	// Serial.print("LSM6DSOX gyro read time: "); Serial.println(micros() - read_start);
}

void Buff_LSM6DSOX::read_lis3mdl(){
	/*
		Get the jawns from the jimmys
	*/
	// unsigned long read_start = micros();
	// lis3mdl.read();
	// if (lis3mdl.magneticFieldAvailable()){
	// 	lis3mdl.readMagneticField(data[6], data[7], data[8]);
	// }
	lis3mdl.readMagneticField(data[6], data[7], data[8]);

	// Serial.print("LIS3MDL mag read time: "); Serial.println(micros() - read_start);
}

typedef union
{
	float number;
	uint8_t bytes[4];
} FLOATBYTE_t;

void Buff_LSM6DSOX::pretty_print_data(){
	/*
		In case you want to show the jawns coming from the jimmys.
	*/
	Serial.println("=========== Buff_LSM6DSOX data ===========");

	FLOATBYTE_t float_byte;

	Serial.print("Acceleration Raw:\n");
	Serial.printf("\t%f\t%f\t%f\n", data[0], data[1], data[2]);
	float_byte.number = data[0];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
	float_byte.number = data[1];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
	float_byte.number = data[2];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);

	Serial.print("Gyroscope Raw\n");
	Serial.printf("\t%f\t%f\t%f\n", data[3], data[4], data[5]);
	float_byte.number = data[3];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
	float_byte.number = data[4];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
	float_byte.number = data[5];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);

	Serial.print("Magnetometer\n");
	Serial.printf("\t%f\t%f\t%f\n", data[6], data[7], data[8]);
	float_byte.number = data[6];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
	float_byte.number = data[7];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
	float_byte.number = data[8];
	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);


	// Serial.print("Gyroscope\t");
	// Serial.print(data[9]); Serial.print("\t"); 
	// Serial.print(data[10]); Serial.print("\t"); 
	// Serial.println(data[11]);

	// Serial.print("Magnetometer\t");
	// Serial.print(data[12]); Serial.print("\t"); 
	// Serial.print(data[13]); Serial.print("\t"); 
	// Serial.println(data[14]);
	
	Serial.println("=======================================");
	
}