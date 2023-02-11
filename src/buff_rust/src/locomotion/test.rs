#![allow(unused_imports)]
use crate::locomotion::controllers::*;
use crate::utilities::loaders::*;

#[cfg(test)]
pub mod dead_control_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn test_simple_pid() {
        let mut pid = PidController::new(vec![1.0, 0.0, 0.0]);

        assert_eq!(pid.err_hist.len(), 0, "Failed to initialize error history");
        // P controller initial unit response
        assert_eq!(pid.update(1.0), 1.0, "Failed basic P with step input");
        assert_eq!(
            pid.err_hist.iter().sum::<f64>(),
            1.0,
            "Failed to accumulate error (P)"
        );
        assert_eq!(
            pid.err_hist.last().unwrap().clone(),
            1.0,
            "Failed to store previous error (P)"
        );

        assert_eq!(pid.err_hist.len(), 1, "Failed to update error history");

        // D controller unit response
        pid.gain = vec![0.0, 1.0, 0.0];
        assert_eq!(pid.update(9.0), 10.0, "Failed basic I with step input");
        assert_eq!(
            pid.err_hist.iter().sum::<f64>(),
            10.0,
            "Failed to accumulate error (I)"
        );
        assert_eq!(
            pid.err_hist.last().unwrap().clone(),
            9.0,
            "Failed to store previous error (I)"
        );

        // I controller unit response
        pid.gain = vec![0.0, 0.0, 1.0];
        assert_eq!(pid.update(10.0), 1.0, "Failed basic D with step input");
        assert_eq!(
            pid.err_hist.iter().sum::<f64>(),
            20.0,
            "Failed to accumulate error (D)"
        );
        assert_eq!(
            pid.err_hist.last().unwrap().clone(),
            10.0,
            "Failed to store previous error (D)"
        );
    }

    #[test]
    pub fn test_state_space() {
        let byu = BuffYamlUtil::new("penguin");
        let k = byu.load_float_matrix("chassis_inverse_kinematics");
        let sc = StateController::new(k);

        // input
        // [mode, shooter active, xdot, ydot, omega, yaw, phi]

        // output
        // [xn drive, xp drive, yn drive, yp drive, yaw, phi, gamma]

        // Test Null
        assert_eq!(
            sc.update(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "Failed null test"
        );

        // Test x direction of chassis
        assert_eq!(
            sc.update(vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            vec![1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
            "Failed X direction test"
        );

        // Test y direction of chassis
        assert_eq!(
            sc.update(vec![0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            vec![0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0],
            "Failed Y direction test"
        );

        // Test rotation of chassis
        assert_eq!(
            sc.update(vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
            vec![1.0, 1.0, 1.0, 1.0, 0.0, -1.0, 0.0],
            "Failed Omega direction test"
        );

        // Test gimbal yaw
        assert_eq!(
            sc.update(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            "Failed yaw test"
        );

        // Test gimbal pitch
        assert_eq!(
            sc.update(vec![0.0, 0.0, 0.0, 0.0, 1.0, 0.0]),
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            "Failed pitch test"
        );

        // Test shooter flag
        assert_eq!(
            sc.update(vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "Failed gamma test"
        );
    }
}
