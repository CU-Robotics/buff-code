use buff_rust::teensy_comms::buff_hid::*;

fn main() {
    let mut layer = HidLayer::new();

    layer.init_comms();
    layer.spin();
}
