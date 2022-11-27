use buff_rust::localization::estimators::*;

fn main() {
    env_logger::init();

    rosrust::init("buff_estimation");

    let mut kee = KinematicEncoderEstimator::new();

    kee.spin();
}
