use std::time::Instant;

pub struct PidController {
    /*
        A controller implementation for a PID filter,
        tracks the error and stores the gain. The struct impl
        an update funtion that returns the lastest output.
    */
    pub gain: Vec<f64>,
    pub err_hist: Vec<f64>,
}

impl PidController {
    pub fn new(gain: Vec<f64>) -> PidController {
        PidController {
            gain: gain,
            err_hist: vec![],
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

        let error_vec;

        if self.err_hist.len() == 0 {
            error_vec = vec![error, 0.0, 0.0];
        } else {
            error_vec = vec![
                error,
                error + self.err_hist.iter().sum::<f64>(),
                error - self.err_hist.last().unwrap(),
            ];
        }

        if self.err_hist.len() > 5 {
            self.err_hist = self.err_hist[1..].to_vec();
        }

        self.err_hist.push(error);

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
    pub i_reference: f64,
    pub outer_controller: PidController,
    pub inner_controller: PidController,
}

impl CascadedPidController {
    pub fn new(gains: Vec<f64>, rate: u128) -> CascadedPidController {
        CascadedPidController {
            rate: rate,
            timestamp: Instant::now(),
            i_reference: 0.0,
            outer_controller: PidController::new(gains[..3].to_vec()),
            inner_controller: PidController::new(gains[3..].to_vec()),
        }
    }

    pub fn update(&mut self, error: f64, feedback: f64) -> f64 {
        // position updates slower
        let micros = self.timestamp.elapsed().as_micros();
        if micros > 1e6 as u128 / self.rate {
            self.timestamp = Instant::now();
            self.i_reference = self.outer_controller.update(error);
        }

        self.inner_controller.update(self.i_reference - feedback)
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
