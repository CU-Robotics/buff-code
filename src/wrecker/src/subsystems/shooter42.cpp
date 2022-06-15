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

    this->feedMotor.init(1, 2);
    this->bottomFlywheel.init(29);
    this->topFlywheel.init(28);

    this->config->feedPIDVel.K[0] = 0.0005;

    this->config->feedPIDPos.continuous = true;
    this->config->feedPIDPos.K[0] = 1.2;
}

void Shooter42::update(unsigned long deltaTime) {
    if (state->driverInput.b && !calibrated)
        calibrated = true;

    if (calibrated) {
        // Spin flywheels
        this->topFlywheel.setPower(0.5);
        this->bottomFlywheel.setPower(0.5);

        /// Death reset
        if (state->driverInput.v) {
            this->bottomFlywheel.init(29);
            this->topFlywheel.init(28);
        }

        // Feeding
        if (state->driverInput.mouseLeft && mouseUp) {
            pos += 90 * 36;
            if (pos >= 360 * 36) {
                pos -= 360 * 36;
            }
            mouseUp = false;
        } else if (state->driverInput.f)
            state->shooter17.feedPID.R = 60 * 36;
        else
            mouseUp = true;

        // Feed PID
        // state->shooter42.feedPIDPos.R = pos;
        // PID_Filter(&config->feedPIDPos, &state->shooter42.feedPIDPos, feedMotor.getAngle(), deltaTime);
        // this->feedMotor.setPower(state->shooter17.feedPID.Y);
        // Serial.println(state->shooter17.feedPID.Y);

        // PID_Filter(&config->feedPIDVel, &state->shooter42.feedPIDVel, feedMotor.getRpm(), deltaTime);
        // this->feedMotor.setPower(state->shooter17.feedPID.Y);
        // Serial.println(state->shooter17.feedPID.Y);
    }
}