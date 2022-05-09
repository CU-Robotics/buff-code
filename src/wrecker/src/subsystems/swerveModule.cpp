#include <Arduino.h>

#include "swerveModule.h"

#include "../state/config.h"
#include "../state/state.h"

void SwerveModule::setup(C_SwerveModule *config, S_Robot *state) {
  this->config = config;
  this->state = state;
}

void SwerveModule::calibrate() {
  
}

void SwerveModule::update(float speed, float angle, float deltaTime) {
  
}