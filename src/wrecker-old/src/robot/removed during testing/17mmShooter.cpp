#include "17mmShooter.h"
#include "RobotInput.h"
#include "RobotConfig.h"
#include "M2006.h"
#include "PIDController.h"

17mmShooter::InfantryShooter() {

}

void 17mmShooter::init(RobotConfig *config) {
  this->config = config;

  this->feederMotor = new m2006(0);
  this->topFlywheelMotor = new Snail(0);
  this->bottomFlywheelMotor = new Snail(1);

  this->feederController = new PIDController(this->config->17mmShooter.PID.P, this->config->17mmShooter.PID.I, this->config->17mmShooter.PID.D, this->config->17mmShooter.PID.F);
}

void 17mmShooter::loop(RobotInput input, float deltaTime) {
  float feedrate = 0;

  // Firing
  if (input.keyMouse1()) {
    feedrate = this->config->17mmShooter.feedrate;

    // DO LATER - Ref System limiting
    // Infantry
    /* if (input.getRobotClass() == 0) {
      switch (input.getRobotLevel()) {
        case 1:

        case 2:

        case 3:
      }
    }*/
  }

  float PIDOutput = this->feederController.calculate(this->feederMotor.rpm, feedrate, deltaTime);
  this->feederMotor.setPower(PIDOutput);
  
  this->topFlywheelMotor.setPower(this->config->17mmShooter.flywheelPower);
  this->bottomFlywheelMotor.setPower(this->config->17mmShooter.flywheelPower);
}