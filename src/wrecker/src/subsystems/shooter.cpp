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
        // Spin flywheels
        int shooterOn = 0;
        if (state->robot == 3 || state->robot == 7) {
            shooterOn = state->refSystem.shooter_on;
        } else if (state->robot == 1) {
            shooterOn = state->refSystem.gimbal_on;
        }
        if (shooterOn && shooterClear) {
            this->topFlywheel.setPower(0.8);
            this->bottomFlywheel.setPower(0.8);
        } else if (!shooterClear) {
            shooterTimer += deltaTime;
        } else {
            shooterTimer = 0;
            shooterClear = false;
        }

        if (shooterTimer > 1000000) // 1 sec
            shooterClear = true;

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
                    state->shooter17.feedPID.R = -config->feedRPMLow * 36;
                    break;
                case 1:
                    state->shooter17.feedPID.R = -config->feedRPMHigh * 36;
                    break;
                case 2:
                    state->shooter17.feedPID.R = -config->feedRPMBurst * 36;
                    break;
                default:
                    state->shooter17.feedPID.R = -config->feedRPMLow * 36;
            }
        } else if (state->driverInput.f)
            state->shooter17.feedPID.R = config->feedRPMLow * 36;
        else
            state->shooter17.feedPID.R = 0;

        // Feed PID
        PID_Filter(&config->feedPID, &state->shooter17.feedPID, feedMotor.getRpm(), deltaTime);
        this->feedMotor.setPower(state->shooter17.feedPID.Y);
        Serial.println(state->shooter17.feedPID.Y);
    }
}