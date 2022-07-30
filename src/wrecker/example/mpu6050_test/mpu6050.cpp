#include "MPU6050.h"

	////////// Initializers //////////

MPU6050::MPU6050(int ado)
{
  Wire.begin();
  
  one = 1;
  six = 6;
  state = 0;
  
	if (ado == 0)
		address = 0x68;
	else
		address = 0x69;
}

void MPU6050::begin(mpu6050_gyro_range gRange, mpu6050_accel_range aRange)
{
  Wire.beginTransmission(address);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  state = Wire.endTransmission();

  test_WhoAmI();
}

  ////////// Settings and Gettings //////////

uint8_t MPU6050::getState()
{
  return state;
}

uint8_t MPU6050::getAddress()
{
  return address;
}

  ////////// Diagnostic Functions //////////


  ////////// Mid-level I/O //////////

void MPU6050::readRawAccel(Vector* data)
{  
  Wire.beginTransmission(address);
  Wire.write(ACCEL_XOUT_H);
  state = Wire.endTransmission(false);
  Wire.requestFrom(address, six);
  while (Wire.available() < 6);
  data->X = (Wire.read() << 8 | Wire.read());
  data->Y = (Wire.read() << 8 | Wire.read());
  data->Z = (Wire.read() << 8 | Wire.read());
}

void MPU6050::readRawGyro(Vector* data)
{  
  Wire.beginTransmission(address);
  Wire.write(GYRO_XOUT_H);
  state = Wire.endTransmission(false);
  Wire.requestFrom(address, six);
  while (Wire.available() < 6);
  data->X = (Wire.read() << 8 | Wire.read());
  data->Y = (Wire.read() << 8 | Wire.read());
  data->Z = (Wire.read() << 8 | Wire.read());
}

	////////// Low-level I/O //////////

uint8_t MPU6050::readRegister(uint8_t reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	state = Wire.endTransmission(false);
	Wire.requestFrom(address, one);
  if (Wire.available() > 0)
    return Wire.read();
  return 0;
}

void MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	state = Wire.endTransmission();
}

	////////// Tests //////////

void MPU6050::test_WhoAmI()
{
  /*
   *  This test checks if the microcontroller can read the mpu's WhoAmI register.
   *  It also checks if the adress returned makes sense; If the state reads no error (ie =0),
   *  it will be replaced by the inverse validity of the WhoAmI register (0=no error, 1=WhoAmI error).
   */
	uint8_t val = readRegister(WHO_AM_I_MPU6050);
 
  if (state == 0)
	  state = !(val == address);
}