#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>

#ifndef BUFF_LSM6DSOX_H
#define BUFF_LSM6DSOX_H

#define IMU_DOF 9

/** The accelerometer data range 
typedef enum accel_range {
	LSM6DS_ACCEL_RANGE_2_G,
	LSM6DS_ACCEL_RANGE_4_G,
	LSM6DS_ACCEL_RANGE_8_G,
	LSM6DS_ACCEL_RANGE_16_G
} lsm6ds_accel_range_t; */
#define IMU_A_RANGE LSM6DS_ACCEL_RANGE_4_G

/** The gyro data range
typedef enum gyro_range {
	LSM6DS_GYRO_RANGE_125_DPS = 0b0010,
	LSM6DS_GYRO_RANGE_250_DPS = 0b0000,
	LSM6DS_GYRO_RANGE_500_DPS = 0b0100,
	LSM6DS_GYRO_RANGE_1000_DPS = 0b1000,
	LSM6DS_GYRO_RANGE_2000_DPS = 0b1100,
	ISM330DHCX_GYRO_RANGE_4000_DPS = 0b0001
} lsm6ds_gyro_range_t; */
#define IMU_G_RANGE LSM6DS_GYRO_RANGE_1000_DPS

/** The accelerometer data rate
typedef enum data_rate {
	LSM6DS_RATE_SHUTDOWN,
	LSM6DS_RATE_12_5_HZ,
	LSM6DS_RATE_26_HZ,
	LSM6DS_RATE_52_HZ,
	LSM6DS_RATE_104_HZ,
	LSM6DS_RATE_208_HZ,
	LSM6DS_RATE_416_HZ,
	LSM6DS_RATE_833_HZ,
	LSM6DS_RATE_1_66K_HZ,
	LSM6DS_RATE_3_33K_HZ,
	LSM6DS_RATE_6_66K_HZ,
} lsm6ds_data_rate_t; */
#define IMU_A_DATA_RATE LSM6DS_RATE_1_66K_HZ
#define IMU_G_DATA_RATE LSM6DS_RATE_1_66K_HZ

/** The high pass filter bandwidth
typedef enum hpf_range {
	LSM6DS_HPF_ODR_DIV_50 = 0,
	LSM6DS_HPF_ODR_DIV_100 = 1,
	LSM6DS_HPF_ODR_DIV_9 = 2,
	LSM6DS_HPF_ODR_DIV_400 = 3,
} lsm6ds_hp_filter_t; */
#define IMU_HPF LSM6DS_HPF_ODR_DIV_400

/** The magnetometer ranges
typedef enum {
	LIS3MDL_RANGE_4_GAUSS = 0b00,  ///< +/- 4g (default value)
	LIS3MDL_RANGE_8_GAUSS = 0b01,  ///< +/- 8g
	LIS3MDL_RANGE_12_GAUSS = 0b10, ///< +/- 12g
	LIS3MDL_RANGE_16_GAUSS = 0b11, ///< +/- 16g
} lis3mdl_range_t; */
#define IMU_M_RANGE LIS3MDL_RANGE_8_GAUSS

/** The magnetometer data rate, includes FAST_ODR bit
typedef enum {
	LIS3MDL_DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
	LIS3MDL_DATARATE_1_25_HZ = 0b0010,  ///<  1.25 Hz
	LIS3MDL_DATARATE_2_5_HZ = 0b0100,   ///<  2.5 Hz
	LIS3MDL_DATARATE_5_HZ = 0b0110,     ///<  5 Hz
	LIS3MDL_DATARATE_10_HZ = 0b1000,    ///<  10 Hz
	LIS3MDL_DATARATE_20_HZ = 0b1010,    ///<  20 Hz
	LIS3MDL_DATARATE_40_HZ = 0b1100,    ///<  40 Hz
	LIS3MDL_DATARATE_80_HZ = 0b1110,    ///<  80 Hz
	LIS3MDL_DATARATE_155_HZ = 0b0001,   ///<  155 Hz (FAST_ODR + UHP)
	LIS3MDL_DATARATE_300_HZ = 0b0011,   ///<  300 Hz (FAST_ODR + HP)
	LIS3MDL_DATARATE_560_HZ = 0b0101,   ///<  560 Hz (FAST_ODR + MP)
	LIS3MDL_DATARATE_1000_HZ = 0b0111,  ///<  1000 Hz (FAST_ODR + LP)
} lis3mdl_dataRate_t; */
#define IMU_M_DATA_RATE LIS3MDL_DATARATE_1000_HZ

/** The magnetometer performance mode
typedef enum {
	LIS3MDL_LOWPOWERMODE = 0b00,  ///< Low power mode
	LIS3MDL_MEDIUMMODE = 0b01,    ///< Medium performance mode
	LIS3MDL_HIGHMODE = 0b10,      ///< High performance mode
	LIS3MDL_ULTRAHIGHMODE = 0b11, ///< Ultra-high performance mode
} lis3mdl_performancemode_t; */
#define IMU_M_PERFORMANCE LIS3MDL_ULTRAHIGHMODE

/** The magnetometer operation mode
typedef enum {
	LIS3MDL_CONTINUOUSMODE = 0b00, ///< Continuous conversion
	LIS3MDL_SINGLEMODE = 0b01,     ///< Single-shot conversion
	LIS3MDL_POWERDOWNMODE = 0b11,  ///< Powered-down mode
} lis3mdl_operationmode_t; */
#define IMU_M_OP_MODE LIS3MDL_CONTINUOUSMODE

class LSM6DSOX{

	public:
		LSM6DSOX();
		void read_lis3mdl();
		void read_lsm6dsox();
		void read_lsm6dsox_accel();
		void read_lsm6dsox_gyro();
		void pretty_print_data();
		void get_angles();

		float data[9];

		//rad angles, theta(pitch), phi(roll), psi(yaw)
		float theta;
		float phi;
		float psi;

		//degree angles
		float pitch;
		float roll;
		float yaw;



	private:
		// Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
		// Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

		Adafruit_LIS3MDL lis3mdl;
		Adafruit_LSM6DSOX lsm6dsox;
};

#endif