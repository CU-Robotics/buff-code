use std::time::Instant;

pub struct PidController {
    /*
        A controller implementation for a PID filter,
        tracks the error and stores the gain. The struct impl
        an update funtion that returns the lastest output.
    */
    pub gain: Vec<f64>,
    pub acc_error: f64,
    pub prev_error: f64,
}

impl PidController {
    pub fn new(gain: Vec<f64>) -> PidController {
        PidController {
            gain: gain,
            acc_error: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn update(&mut self, error: f64) -> f64 {
        /*
            Apply a PID filter to the reference and feedback.
            @param
                the most recently measured error
            @return
                the dot product of [P, I, D] (gains) and [e, sum(e), de] (current derivitive and integral of error)
        */

        self.acc_error += error;

        let error_vec = vec![error, self.acc_error, error - self.prev_error];

        self.prev_error = error;

        error_vec
            .iter()
            .zip(self.gain.iter())
            .map(|(x, y)| x * y)
            .sum()
    }
}

pub struct CascadedPidController {
    pub rate: u128,
    pub timestamp: Instant,
    pub v_reference: f64,
    pub position_controller: PidController,
    pub velocity_controller: PidController,
}

impl CascadedPidController {
    pub fn new(gains: Vec<f64>) -> CascadedPidController {
        CascadedPidController {
            rate: 100,
            timestamp: Instant::now(),
            v_reference: 0.0,
            position_controller: PidController::new(gains[..3].to_vec()),
            velocity_controller: PidController::new(gains[3..].to_vec()),
        }
    }

    pub fn update(&mut self, error: f64, feedback: f64) -> f64 {
        // position updates slower
        let micros = self.timestamp.elapsed().as_micros();
        if micros > 1e6 as u128 / self.rate {
            self.timestamp = Instant::now();
            self.v_reference = self.position_controller.update(error);
        }

        self.velocity_controller.update(self.v_reference - feedback)
    }
}

pub struct StateController {
    /*
        uses a reference and gain matrix
        to derive a reference for each motor controller.
    */
    pub gain_matrix: Vec<Vec<f64>>,
}

impl StateController {
    pub fn new(k: Vec<Vec<f64>>) -> StateController {
        StateController { gain_matrix: k }
    }

    pub fn update(&self, state: Vec<f64>) -> Vec<f64> {
        /*
            Update the controllers to track the reference input.
            Use the inertial feedback to improve the tracking and estimate.
            @param
                reference_control: could be the remote input or autonomous descision making
                inertial_feedback: measured chassis motion (psych!)
            @return
                control law applied to the the state reference
        */
        self.gain_matrix
            .iter()
            .map(|k| k.iter().zip(state.iter()).map(|(k, s)| k * s).sum::<f64>())
            .collect()
    }
}
