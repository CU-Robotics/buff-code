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

    this->config->feedPID.K[0] = 0.0005;
}

void Shooter::update(unsigned long deltaTime) {
    if (state->driverInput.b && !calibrated)
        calibrated = true;

    if (calibrated) {
        Serial.println("enabled");
        // Spin flywheels
        this->topFlywheel.setPower(0.4);
        this->bottomFlywheel.setPower(0.4);

        /// Death reset
        if (state->driverInput.v) {
            this->bottomFlywheel.init(29);
            this->topFlywheel.init(28);
        }

        // Mode switching
        if (state->driverInput.q) {
            this->state->shooter17.mode = 0;
        } else if (state->driverInput.e) {
            this->state->shooter17.mode = 1;
        } else if (state->driverInput.r) {
            this->state->shooter17.mode = 2;
        }

        // Feeding
        if (state->driverInput.mouseLeft) {
            switch(this->state->shooter17.mode) {
                case 0:
                    state->shooter17.feedPID.R = -60 * 36;
                    break;
                case 1:
                    state->shooter17.feedPID.R = -90 * 36;
                    break;
                case 2:
                    state->shooter17.feedPID.R = -120 * 36;
                    break;
                default:
                    state->shooter17.feedPID.R = -60 * 36;
            }
        } else if (state->driverInput.f)
            state->shooter17.feedPID.R = 60 * 36;
        else
            state->shooter17.feedPID.R = 0;

        Serial.print(state->shooter17.feedPID.R);
        Serial.print(" - ");

        // Feed PID
        PID_Filter(&config->feedPID, &state->shooter17.feedPID, feedMotor.getRpm(), deltaTime);
        this->feedMotor.setPower(state->shooter17.feedPID.Y);
        Serial.println(state->shooter17.feedPID.Y);
    }
}