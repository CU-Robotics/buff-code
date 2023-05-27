#include "sensors/icm20649.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/loggers.h"

int main() {
	while(!Serial) {}
	Serial.println("Start IMU Tests");

	ICM20649 lower;
	ICM20649 upper;
	lower.init(GIMBAL_IMU_ADDR);
	upper.init(CHASSIS_IMU_ADDR);

	Serial.println("IMU init success");

	while (1) {
		timer_set(0);
		lower.read_accel();
		lower.read_gyro();
		fancy_vec(lower.data, 6);
		upper.read_accel();
		upper.read_gyro();
		fancy_vec(upper.data, 6);
		timer_wait_us(0, 1000);
	}
}