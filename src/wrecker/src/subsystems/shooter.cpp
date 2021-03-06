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

    this->config->feedPID.K[0] = 0.0005;
}

void Shooter::update(unsigned long deltaTime) {
    if (state->driverInput.b && !calibrated)
        calibrated = true;

    // Spin flywheels
    int shooterOn = 0;
    if (state->robot == 3 || state->robot == 7) {
        shooterOn = state->refSystem.shooter_on;
    } else if (state->robot == 1) {
        shooterOn = state->refSystem.gimbal_on;
    }

    if (calibrated || (state->robot == 7 && state->driverInput.s2 == 2)) {
        if (shooterOn && shooterClear) {
            if (state->robot == 7) {
                fw_2.setPower(0.5);
                fw_1.setPower(0.5);
            } else {
                if (state->robot == 3 && state->driverInput.s2 == 2) {
                    // 1v1
                    fw_2.setPower(0.5);
                    fw_1.setPower(0.5);
                } else {
                    // 3v3
                    if (state->robot != 1) {
                        fw_2.setPower(0.8);
                    }
                        fw_1.setPower(0.8);
                }
            }
        } else if (!shooterOn) {
            shooterTimer = 0;
            shooterClear = false;
            if (state->robot != 1) {
                fw_2.reset();
            }
            fw_1.reset();
        } else {
            shooterTimer += deltaTime;
        }

        if (shooterTimer > 4500000) // 4.5 sec
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
        if (state->robot == 7) {
            if (state->gimbal.tracking) {
                state->shooter17.feedPID.R = -config->feedRPMLow * 36;
            } else {
                state->shooter17.feedPID.R = 0;
            }
        } else {
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
        }

        // Feed PID
        PID_Filter(&config->feedPID, &state->shooter17.feedPID, feedMotor.getRpm(), deltaTime);
        this->feedMotor.setPower(state->shooter17.feedPID.Y);

    } else if (state->robot == 7 && state->driverInput.s2 != 2) {
        fw_1.setPower(0.0);
        fw_2.setPower(0.0);
        this->feedMotor.setPower(0.0);     
    }
    else {
        this->feedMotor.setPower(0.0);       
    }
}