#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "railChassis.h"
#include "drivers/c620.h"
#include "algorithms/PID_Filter.h"

RailChassis::RailChassis() {
  
}

void RailChassis::setup(C_RailChassis *data, S_Robot *r_state) {
  config = data;
  state = r_state;

  leftDriveMotor.init(1, 1);
  rightDriveMotor.init(2, 1);
}

void RailChassis::update(unsigned long deltaTime) {
    int x = state->driverInput.d - state->driverInput.a;

    leftDriveMotor.setPower(x);
    rightDriveMotor.setPower(-x);
}

void RailChassis::drive(float speed) {

}