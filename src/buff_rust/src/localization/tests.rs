#![allow(unused_imports)]

use crate::localization::swerve::{Body3D, CartesianNode};
use nalgebra::Vector3; //Quaternion, UnitQuaternion,

static ERROR_VEC: Vector3<f64> = Vector3::new(1.0e-6, 1.0e-6, 1.0e-6);

#[cfg(test)]
pub mod localization_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    macro_rules! relative_eq {
        // macth like arm for macro
        ($a:expr,$b:expr,$c:expr) => {
            // macro expand to this code
            {
                // $a and $b will be templated using the value/variable provided to macro
                assert!(
                    ($a - $b).abs() < $c,
                    "`(left ~= right)`\nerror: {:?}\nleft: {:?}\nright: {:?}\n",
                    $c,
                    $a,
                    $b
                )
            }
        };
    }

    macro_rules! vec3 {
        ($a:expr,$b:expr,$c:expr) => {{
            Vector3::new($a, $b, $c)
        }};
    }

    #[test]
    fn basic_x_tf() {
        let vector = Vector3::new(1.0, 0.0, 0.0);
        let mut node = CartesianNode::default();

        assert_eq!(vector, node.isometry.transform_vector(&vector));
    }

    #[test]
    fn x_rotated_x_tf() {
        let vector = Vector3::new(1.0, 0.0, 0.0);
        let mut node = CartesianNode::new(
            [0.0, 0.0, 0.0],
            [std::f64::consts::PI, 0.0, 0.0],
            "test_node".to_string(),
            vec![],
        );

        let eulers = node.isometry.rotation.euler_angles();
        relative_eq!(
            vec3!(std::f64::consts::PI, 0.0f64, 0.0f64),
            vec3!(eulers.0, eulers.1, eulers.2),
            ERROR_VEC
        );
        relative_eq!(vector, node.isometry.transform_vector(&vector), ERROR_VEC);
    }

    #[test]
    fn y_rotated_x_tf() {
        let vector = Vector3::new(1.0, 0.0, 0.0);
        let mut node = CartesianNode::new(
            [0.0, 0.0, 0.0],
            [0.0, std::f64::consts::PI, 0.0],
            "test_node".to_string(),
            vec![],
        );

        // assert_eq!((0.0, 3.1415, 0.0), node.isometry.rotation.euler_angles());
        relative_eq!(
            vec3!(-1.0, 0.0, 0.0),
            node.isometry.transform_vector(&vector),
            ERROR_VEC
        );
    }

    #[test]
    fn y_rotated_z_translated_x_tf() {
        let vector = Vector3::new(1000.0, 0.0, 0.0);
        let mut node = CartesianNode::new(
            [0.0, 0.0, -1.0],
            [0.0, std::f64::consts::PI / 2.0, 0.0],
            "test_node".to_string(),
            vec![],
        );

        let eulers = node.isometry.rotation.euler_angles();
        relative_eq!(
            vec3!(0.0f64, std::f64::consts::PI / 2.0, 0.0f64),
            vec3!(eulers.0, eulers.1, eulers.2),
            ERROR_VEC
        );

        relative_eq!(
            node.isometry.transform_vector(&vector),
            vec3!(0.0, 0.0, -1000.0),
            ERROR_VEC
        );
    }

    // #[test]
    // fn load_urdf() {
    //     let urdf_text =
    //         urdf_rs::utils::convert_xacro_to_urdf("/home/mdyse/buff-code/buffpy/config/robots/penguin".to_string() + "/buffbot.xacro").unwrap();
    //     let robot_urdf = urdf_rs::read_from_string(&urdf_text).unwrap();

    //     robot_urdf.joints.iter().for_each(|j| {
    //         println!("{} {:?}", j.name, j.origin.rpy);
    //     });
    // }

    #[test]
    fn init_body() {
        let body = Body3D::from_urdf(
            "/home/mdyse/buff-code/buffpy/config/robots/penguin".to_string(),
            false,
        );
    }

    #[test]
    fn test_fl_rotations() {
        let mut body = Body3D::from_urdf(
            "/home/mdyse/buff-code/buffpy/config/robots/penguin".to_string(),
            false,
        );

        let fls_idx = body
            .names
            .iter()
            .position(|name| name == "fl_steer")
            .unwrap();

        let fld_idx = body
            .names
            .iter()
            .position(|name| name == "fl_drive")
            .unwrap();

        let mut eulers = body.frames[fls_idx].isometry.rotation.euler_angles();
        relative_eq!(
            vec3!(0.0f64, 0.0f64, std::f64::consts::PI / 4.0),
            vec3!(eulers.0, eulers.1, eulers.2),
            ERROR_VEC
        );

        eulers = body.frames[fld_idx].isometry.rotation.euler_angles();
        relative_eq!(
            vec3!(std::f64::consts::PI / 2.0, 0.0f64, 0.0f64),
            vec3!(eulers.0, eulers.1, eulers.2),
            ERROR_VEC
        );

        relative_eq!(
            body.frames[fld_idx]
                .isometry
                .transform_vector(&Vector3::new(0.0, 0.0, 100.0)),
            vec3!(0.0, -100.0, 0.0),
            ERROR_VEC
        );
    }

    #[test]
    fn test_fl_integration() {
        let mut body = Body3D::from_urdf(
            "/home/mdyse/buff-code/buffpy/config/robots/penguin".to_string(),
            false,
        );

        let fls_idx = body
            .names
            .iter()
            .position(|name| name == "fl_steer")
            .unwrap();

        let fld_idx = body
            .names
            .iter()
            .position(|name| name == "fl_drive")
            .unwrap();

        for i in 0..1000 {
            body.frames[fld_idx].omega = Vector3::new(0.0, 0.0, 1.0);
            let (r, p, y) = body.frames[fld_idx].isometry.rotation.euler_angles();

            // body.frames[fld_idx].omega;
            let bomega = body.frames[fld_idx]
                .isometry
                .inverse_transform_vector(&body.frames[fld_idx].omega);

            println!(
                "initial angles {:?}",
                body.frames[fld_idx].isometry.rotation.euler_angles()
            );
            // println!("delta {:?}", bomega);
            // println!("Rotation Axis {:?}", body.frames[fld_idx].isometry.rotation.axis_angle());
            // assert_eq!((bomega[0] * 100000.0).round() / 100000.0, 0.0);
            // assert_eq!((bomega[2] * 100000.0).round() / 100000.0, 0.0);

            let dt = 1.0;

            // Using Quaternions
            let mut coords = body.frames[fld_idx].isometry.rotation.coords;
            coords[0] += dt
                * 0.5
                * ((-coords[0] * bomega[0]) + (-coords[1] * bomega[1]) + (-coords[2] * bomega[2]));
            coords[1] += dt
                * 0.5
                * ((coords[3] * bomega[0]) + (-coords[2] * bomega[1]) + (coords[1] * bomega[2]));
            coords[2] += dt
                * 0.5
                * ((coords[2] * bomega[0]) + (coords[3] * bomega[1]) + (-coords[0] * bomega[2]));
            coords[3] += dt
                * 0.5
                * ((-coords[1] * bomega[0]) + (coords[0] * bomega[1]) + (coords[3] * bomega[2]));
            body.frames[fld_idx].isometry.rotation =
                UnitQuaternion::from_quaternion(Quaternion { coords: coords });

            // Using Euler Angles
            // body.frames[fld_idx].isometry.rotation = UnitQuaternion::from_euler_angles(
            //     r + (bomega[0] * dt),
            //     p + (bomega[1] * dt),
            //     y + (bomega[2] * dt),
            // );

            println!(
                "final angles {:?}",
                body.frames[fld_idx].isometry.rotation.euler_angles()
            );
            let eulers = body.frames[fld_idx].isometry.rotation.euler_angles();
            assert!(eulers.2.abs() < 1.0e-1);
        }
    }
}
