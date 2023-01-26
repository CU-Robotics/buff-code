use buff_rust::localization::estimators::*;

fn main() {
    env_logger::init();

    rosrust::init("buff_estimation");

    let mut rse = RobotStateEstimator::new();

    rse.spin();
}
