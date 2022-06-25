#include "shooter42.h"

#include "state/state.h"
#include "state/config.h"
#include "drivers/c620.h"
#include "drivers/flywheel.h"
#include "algorithms/PID_Filter.h"

Shooter42::Shooter42() {

}

void Shooter42::setup(C_Shooter42 *config, S_Robot *state) {
    this->config = config;
    this->state = state;

    this->feedMotor.init(2, 2);

    this->config->feedPIDVel.K[0] = 0.0005;

    this->config->feedPIDPos.continuous = true;
    this->config->feedPIDPos.K[0] = 0.0125;
}

void Shooter42::update(unsigned long deltaTime) {
    if (state->driverInput.b && !calibrated)
        calibrated = true;

    float rawPos = feedMotor.getAngle();
    if (calibrated) {
        float difference = this->prevPos - rawPos;
        if (difference < -180)
        this->posRollover--;
        else if (difference > 180)
        this->posRollover++;
    }
    this->prevPos = rawPos;

    // Spin flywheels
    int shooterOn = 0;
    shooterOn = state->refSystem.shooter_on;

    if (calibrated) {
        if (shooterOn && shooterClear) {
            fw_2.setPower(1.0);
        } else if (!shooterOn) {
            shooterTimer = 0;
            shooterClear = false;
            fw_2.reset();
        } else {
            shooterTimer += deltaTime;
        }

        if (shooterTimer > 5000000) // 5 sec
            shooterClear = true;

        // Feeding
        if (state->driverInput.ctrl && mouseUp) {
            pos -= 90 * 36;
            mouseUp = false;
        } else if (state->driverInput.f)
            state->shooter42.feedPIDVel.R = 60 * 36;
        else if (!state->driverInput.ctrl)
            mouseUp = true;

        // Feed PID
        float angle = this->feedMotor.getAngle() + (this->posRollover * 360);
        state->shooter42.feedPIDPos.R = pos;
        PID_Filter(&config->feedPIDPos, &state->shooter42.feedPIDPos, angle, deltaTime);

        if (!state->driverInput.f) {
            state->shooter42.feedPIDVel.R = state->shooter42.feedPIDPos.Y * 100;
        }
        PID_Filter(&config->feedPIDVel, &state->shooter42.feedPIDVel, feedMotor.getRpm(), deltaTime);
        this->feedMotor.setPower(state->shooter42.feedPIDVel.Y);
    }
}