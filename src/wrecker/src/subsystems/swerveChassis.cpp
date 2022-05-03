#include <Arduino.h>

#include "swerveChassis.h"

#include "swerveModule.h"
#include "../state/config.h"
#include "../state/state.h"

void SwerveChassis::setup(C_SwerveChassis config) {
  this->config = config;
  this->drivebaseRadius = sqrt(pow(config.drivebaseLength, 2) + pow(config.drivebaseWidth, 2));
}

void SwerveChassis::loop(float deltaTime) {
  
}

void SwerveChassis::calibrate() {
      fr_offset = find_match(motorFR.getAngle(), fr_alignment, 9);
      fl_offset = find_match(motorFL.getAngle(), fl_alignment, 9);
      br_offset = find_match(motorBR.getAngle(), br_alignment, 9);
      bl_offset = find_match(motorBL.getAngle(), bl_alignment, 9);
      fr_rollover = 0;
      fl_rollover = 0;
      br_rollover = 0;
      bl_rollover = 0;
}

void SwerveChassis::drive(float driveX, float driveY, float spin, float deltaTime) {
    float A = driveX - spin * (this->config.drivebaseLength / this->drivebaseRadius);
    float B = driveX + spin * (this->config.drivebaseLength / this->drivebaseRadius);
    float C = driveY - spin * (this->config.drivebaseWidth / this->drivebaseRadius);
    float D = driveY + spin * (this->config.drivebaseWidth / this->drivebaseRadius);

    float speedFR = sqrt(pow(B, 2) + pow(C, 2));
    float speedFL = sqrt(pow(B, 2) + pow(D, 2));
    float speedBL = sqrt(pow(A, 2) + pow(D, 2));
    float speedBR = sqrt(pow(A, 2) + pow(C, 2));
    float speedMax = std::max({speedFR, speedFL, speedBL, speedBR});

    if (speedMax > 1) {
        speedFR /= speedMax;
        speedFL /= speedMax;
        speedBL /= speedMax;
        speedBR /= speedMax;
    }

    float angleFR = radiansToDegrees(atan2(B, C));
    float angleFL = radiansToDegrees(atan2(B, D));
    float angleBL = radiansToDegrees(atan2(A, D));
    float angleBR = radiansToDegrees(atan2(A, C));
}

void SwerveChassis::driveSimple(float driveX, float driveY, float deltaTime) {

}

float SwerveChassis::radiansToDegrees(float radians) {
    double degrees;
    degrees = radians * 180 / PI;
    return degrees;
}