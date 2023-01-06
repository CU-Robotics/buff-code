#include "lsm6dsox.h"

LSM6DSOX::LSM6DSOX() {
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

void LSM6DSOX::read_lsm6dsox_accel(){
	/*
		Get the jawns from the jimmys
	*/
	lsm6dsox.readAcceleration(data[0], data[1], data[2]);

	// Serial.print("LSM6DSOX accel read time: "); Serial.println(micros() - read_start);
}

void LSM6DSOX::read_lsm6dsox_gyro(){
	/*
		Get the jawns from the jimmys
	*/
	lsm6dsox.readGyroscope(data[3], data[4], data[5]);
	// Serial.print("LSM6DSOX gyro read time: "); Serial.println(micros() - read_start);
}

void LSM6DSOX::read_lis3mdl(){
	/*
		Get the jawns from the jimmys
	*/
	lis3mdl.readMagneticField(data[6], data[7], data[8]);

	// Serial.print("LIS3MDL mag read time: "); Serial.println(micros() - read_start);
}


// typedef union
// {
// 	float number;
// 	uint8_t bytes[4];
// } FLOATBYTE_t;


// void LSM6DSOX::pretty_print_data(){
// 	/*
// 		In case you want to show the jawns coming from the jimmys.
// 	*/
	
// 	Serial.println("=========== LSM6DSOX data ===========");

// 	FLOATBYTE_t float_byte;

// 	Serial.print("Acceleration Raw:\n");
// 	Serial.printf("\t%f\t%f\t%f\n", data[0], data[1], data[2]);
// 	float_byte.number = data[0];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
// 	float_byte.number = data[1];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
// 	float_byte.number = data[2];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);

// 	Serial.print("Gyroscope Raw\n");
// 	Serial.printf("\t%f\t%f\t%f\n", data[3], data[4], data[5]);
// 	float_byte.number = data[3];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
// 	float_byte.number = data[4];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
// 	float_byte.number = data[5];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);

// 	Serial.print("Magnetometer\n");
// 	Serial.printf("\t%f\t%f\t%f\n", data[6], data[7], data[8]);
// 	float_byte.number = data[6];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
// 	float_byte.number = data[7];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
// 	float_byte.number = data[8];
// 	Serial.printf("\t%X\t%X\t%X\t%X\n", float_byte.bytes[0], float_byte.bytes[1], float_byte.bytes[2], float_byte.bytes[3]);
	
// 	Serial.println("=======================================");
	
// }