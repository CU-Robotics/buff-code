// use crate::hid::buff_hid::HidLayer;

#[cfg(test)]
pub mod hid_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn basic_hid() {
        let mut layer = HidLayer::new();
    }
}
