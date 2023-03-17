#include "unity.h"
#include <Arduino.h>
#include "buff_cpp/timing.h"
#include "buff_cpp/loggers.h"
#include "sensors/lsm6dsox.h"

int imu_dev_cnt = 0;

LSM6DSOX imu;


int test_imu_read(void) {
	int errors = 0;
	// check
	Serial.println("\tAccelerometer Test: ...");
	timer_set(0);
	imu.read_lsm6dsox_accel();
	timer_mark(0);

	Serial.println("\tGyroscope Test: ...");
	timer_set(0);
	imu.read_lsm6dsox_gyro();
	timer_mark(0);

	Serial.println("\tMagnetometer Test: ...");
	timer_set(0);
	imu.read_lis3mdl();
	timer_mark(0);
	
	for (int i = 0; i < 9; i++) {
		errors += float_neq(0.0, abs(imu.data[i]), "Failed");
		errors += float_nan(imu.data[i]);		
	}
	Serial.println();

	return errors;
}


int test_imu_bias(void) {
	Serial.printf("\n\tTesting IMU bias, please leave it completely flat and still:...\n");
	delay(2500);
	// check
	size_t iters = 50;
	float accel_sum[3] = {0.0, 0.0, 0.0};
	float gyro_sum[3] = {0.0, 0.0, 0.0};
	float mag_sum[3] = {0.0, 0.0, 0.0};
	float mag_x[iters];
	float mag_y[iters];
	float mag_z[iters];

	for (size_t i = 0; i < iters; i++) {
		timer_set(0);

		imu.read_lsm6dsox_accel();

		accel_sum[0] += imu.data[0];
		accel_sum[1] += imu.data[1];
		accel_sum[2] += imu.data[2];

		// Linear acceleration zero-g level offset accuracy +/- 0.02 g
		if (abs(imu.data[0]) > 0.02) {
			Serial.printf("\tHigh X Acceleration: %f\n", imu.data[0]);
		}
		if (abs(imu.data[1]) > 0.02) {
			Serial.printf("\tHigh Y Acceleration: %f\n", imu.data[1]);
		}
		if (abs(1 - (imu.data[2])) > 0.02) {
			Serial.printf("\tHigh Z Acceleration: %f\n", imu.data[2]);
		}

		imu.read_lsm6dsox_gyro();

		gyro_sum[0] += imu.data[3];
		gyro_sum[1] += imu.data[4];
		gyro_sum[2] += imu.data[5];

		imu.read_lis3mdl();

		mag_sum[0] += imu.data[6];
		mag_sum[1] += imu.data[7];
		mag_sum[2] += imu.data[8];

		mag_x[i] = imu.data[6];
		mag_y[i] = imu.data[7];
		mag_z[i] = imu.data[8];

		timer_wait_us(0, 1000);
	}

	// Gyroscope zero-rate level offset accuracy +/- 1 dps
	if (abs(gyro_sum[0] / iters) > 1) {
		Serial.printf("\tHigh X Rotation: %f (device is not still)\n", gyro_sum[0] / iters);
	}
	if (abs(gyro_sum[1] / iters) > 1) {
		Serial.printf("\tHigh Y Rotation: %f (device is not still)\n", gyro_sum[1] / iters);
	}
	if (abs(gyro_sum[2] / iters) > 1) {
		Serial.printf("\tHigh Z Rotation: %f (device is not still)\n", gyro_sum[2] / iters);
	}

	// Magnetometer Zero-gauss level +/- 1 gauss
	for (size_t i= 0; i < iters; i++) {
		if (abs((mag_sum[0] / iters) - mag_x[i]) > 1) {
			Serial.printf("\tHigh X Feild: %d (device is not still or there is magnetetic interference?)\n", mag_sum[0] / iters);
		}
		if (abs((mag_sum[1] / iters) - mag_y[i]) > 1) {
			Serial.printf("\tHigh Y Feild: %d (device is not still or there is magnetetic interference?)\n", mag_sum[1] / iters);
		}
		if (abs((mag_sum[2] / iters) - mag_z[i]) > 1) {
			Serial.printf("\tHigh Z Feild: %d (device is not still or there is magnetetic interference?)\n", mag_sum[2] / iters);
		}
	}
	

	Serial.printf("\n\tAccelerometer Bias (g / ms):\n\t%f\t%f\t%f\n",
		accel_sum[0] / iters, accel_sum[1] / iters, accel_sum[2] / iters);
	Serial.printf("\n\tGyroscope Bias (degrees / ms):\n\t%f\t%f\t%f\n",
		gyro_sum[0] / iters, gyro_sum[1] / iters, gyro_sum[2] / iters);
	Serial.printf("\n\tMagnetometer Bias (guass / ms):\n\t%f\t%f\t%f\n",
		mag_sum[0] / iters, mag_sum[1] / iters, mag_sum[2] / iters);

	Serial.println();

	return 0;
}


int run_imu_tests(void) {

	int errors = 0;
	errors += test_imu_read();
	errors += test_imu_bias();
	// UNITY_BEGIN();
	// RUN_TEST(test_imu_read);
	// RUN_TEST(test_imu_bias);

	// return UNITY_END();
	return errors;
}

// Runs once
int main() {
	while (!Serial) {};
	Serial.println("Start lsm6dsox tests");
	delay(2000);

	int errors = run_imu_tests();

	Serial.println("Finished tests");
	Serial.printf("%i failed\n", errors);

	return 0;
}
