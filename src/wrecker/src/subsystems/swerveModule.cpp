#include <Arduino.h>

#include "state/state.h"
#include "state/config.h"
#include "swerveModule.h"


SwerveModule::SwerveModule() {
  
}

void SwerveModule::setup(C_SwerveModule *data, S_Robot *r_state) {
  config = data;
  state = r_state;
}

void SwerveModule::calibrate() {
  
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  
}