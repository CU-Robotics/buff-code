#include "MPU6050.h"

MPU6050 mpu(0);
Vector rawA, rawG;

void printVector(Vector* vect, String title)
{
  /*
   * A helper for printing Vectors to the Serial Console
   */
  Serial.print(title + " (");
  Serial.print(vect->X); Serial.print(", "); Serial.print(vect->Y); Serial.print(", "); Serial.print(vect->Z); Serial.println(")");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial.println("Initialized Program!");

  mpu.begin();
  Serial.println("Initialized mpu6050");
}

void loop() {

  
  mpu.readRawAccel(&rawA);
  mpu.readRawGyro(&rawG);
  mpu.test_WhoAmI();
  
  Serial.print("WhoAmI test: "); Serial.println(mpu.getState());
  printVector(&rawA, "MPU6050 Accel Raw: ");
  printVector(&rawG, "MPU6050 Gyro Raw: ");
}