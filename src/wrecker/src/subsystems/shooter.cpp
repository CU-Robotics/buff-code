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

    this->feedMotor.init(2, 2);

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

    if (true) {//(calibrated && state->driverInput.s1 != 1) || ((state->robot == 7 && state->driverInput.s2 == 2) || (state->robot == 7 && state->driverInput.s1 == 2))) {
        if (state->robot != 7 && state->driverInput.s1 == 1) {
            fw_1.reset();
            fw_2.reset();
            shooterTimer = 0;
            shooterClear = false;
        } else if (shooterOn && shooterClear) {
            // Sentry
            if (state->robot == 7) {
                fw_2.setPower(0.6);
                fw_1.setPower(0.6);
                Serial.println ("Flywheels should be spinning");
            } else {
                if (state->robot == 3 && state->driverInput.s2 == 2) {
                    // 1v1
                    fw_2.setPower(0.56);
                    fw_1.setPower(0.56);
                } else {
                    // 3v3
                    if (state->robot == 1) {
                        fw_2.setPower(0.3);
                        fw_1.setPower(0.3);
                    } else {
                        fw_2.setPower(0.56);
                        fw_1.setPower(0.56);
                    }
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

        int timeout = 4000000; // 4 sec
        if (state->robot == 1) {
            timeout = 6000000; // 6 sec on the Hero
        }
        if (shooterTimer > timeout) // 4.5 sec
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
            if (state->gimbal.tracking || state->driverInput.s1 == 2) {
                state->shooter17.feedPID.R = -config->feedRPMLow * 36;
            } else {
                state->shooter17.feedPID.R = 0;
            }
        } else {
            if (true) {//state->driverInput.mouseLeft) {
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
        state->shooter17.feedPID.R = -450 * 36;
        PID_Filter(&config->feedPID, &state->shooter17.feedPID, feedMotor.getRpm(), deltaTime);
        this->feedMotor.setPower(state->shooter17.feedPID.Y);

    } else if (state->robot == 7 && state->driverInput.s2 != 2) {
        this->feedMotor.setPower(0.0);     
    } else {
        this->feedMotor.setPower(0.0);       
    }
}