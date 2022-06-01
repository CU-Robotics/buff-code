#include "shooter.h"

#include "state/state.h"
#include "state/config.h"
#include "drivers/c620.h"
#include "drivers/flywheel.h"
#include "algorithms/PID_Filter.h"

Shooter::Shooter() {

}

void Shooter::setup(C_Shooter17 *config, S_Robot *state) {
    this->config = config;
    this->state = state;

    this->feedMotor.init(1, 2);
    this->bottomFlywheel.init(29);
    this->topFlywheel.init(28);
}

void Shooter::update(unsigned long deltaTime) {
    if (state->driverInput.b && !calibrated) {
        calibrated = true;
    }

    this->config->feedPID.K[0] = 0.0005;
    state->Shooter17.feedPID.R = -2160;
    PID_Filter(&config->feedPID, &state->Shooter17.feedPID, feedMotor.getRpm(), deltaTime);

    if (calibrated) {
        this->topFlywheel.setPower(0.2);
        this->bottomFlywheel.setPower(0.2);

        if (state->driverInput.f) {
            this->feedMotor.setPower(state->Shooter17.feedPID.Y);
            Serial.print(state->Shooter17.feedPID.Y);
            Serial.print(" - ");
            Serial.print(feedMotor.getRpm());
            Serial.println();
        }
        else
            this->feedMotor.setPower(0.0);
    }
}