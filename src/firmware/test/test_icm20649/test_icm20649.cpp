#include "unity.h"
#include "buff_cpp/timing.cpp"
#include "sensors/icm20649.cpp"

icm20649 imu;

void setUp(void) {

}

void tearDown(void) {

}

void test_imu_read(void) {
    // check
	Serial.println("\tRead Test: ...");
	timer_set(0);
	imu.readSensor();
	timer_mark(0);
	
    TEST_ASSERT(0.0 < abs(imu.getAccelX));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getAccelX);
    TEST_ASSERT(0.0 < abs(imu.getAccelY));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getAccelY);
    TEST_ASSERT(0.0 < abs(imu.getAccelZ));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getAccelZ);
    TEST_ASSERT(0.0 < abs(imu.getGyroX));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getGyroX);
    TEST_ASSERT(0.0 < abs(imu.getGyroY));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getGyroY);
    TEST_ASSERT(0.0 < abs(imu.getGyroZ));
    TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getGyroZ);
	Serial.println();
}

int run_imu_tests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_imu_read);

    return UNITY_END();
}

void setup() {
    delay(2000);
    run_imu_tests();
}

void loop() {}