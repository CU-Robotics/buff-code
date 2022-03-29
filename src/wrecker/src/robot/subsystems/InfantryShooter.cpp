#include "InfantryShooter.h"
#include "RobotInput.h"
#include "M2006.h"

InfantryShooter::InfantryShooter() {

}

void InfantryShooter::init() {
  this->feederMotor = new m2006(0);
  this->topFlywheelMotor = new Snail(0);
  this->bottomFlywheelMotor = new Snail(1);
}

void InfantryShooter::loop(RobotInput input, RobotConstants constants) {
  if (input.keyMouse1()) {
    
  }
}