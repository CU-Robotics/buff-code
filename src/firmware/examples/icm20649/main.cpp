// #include "unity.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/loggers.h"
#include "sensors/icm20649.h"

ICM20649 imu;

void setUp(void) {

}

void tearDown(void) {

}

int test_imu_read(void) {
    // check
	Serial.println("\tRead Test: ...");
	timer_set(0);
	imu.readSensor();
	timer_mark(0);
    Serial.println();
	
    return float_leq(0.0, abs(imu.getAccelX()), "get accel failed check");
    // TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getAccelX);
    // TEST_ASSERT(0.0 < abs(imu.getAccelY));
    // TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getAccelY);
    // TEST_ASSERT(0.0 < abs(imu.getAccelZ));
    // TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getAccelZ);
    // TEST_ASSERT(0.0 < abs(imu.getGyroX));
    // TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getGyroX);
    // TEST_ASSERT(0.0 < abs(imu.getGyroY));
    // TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getGyroY);
    // TEST_ASSERT(0.0 < abs(imu.getGyroZ));
    // TEST_ASSERT_FLOAT_IS_NOT_NAN(imu.getGyroZ);
}

int run_imu_tests(void) {
    int errors = 0;
    errors += test_imu_read();
    return errors;
    // UNITY_BEGIN();
    // RUN_TEST(test_imu_read);

    // return UNITY_END();
}

int main() {
    // Wait ~2 seconds before the Unity test runner
    // establishes connection with a board Serial interface
    while (!Serial) {};
    Serial.println("Start icm20649 tests");
    delay(2000);

    int errors = run_imu_tests();

    Serial.println("Finished tests");
    Serial.printf("%i failed\n", errors);

    return 0;
}