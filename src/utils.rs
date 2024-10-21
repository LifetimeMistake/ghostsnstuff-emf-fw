use esp_idf_hal::ledc::Resolution;

use crate::vector::Vector3;

const _: () = if Resolution::Bits1 as u32 != 0 {
    panic!("Resolution::Bits1 has to be 0");
};

pub fn ledc_resolution_to_u32(resolution: Resolution) -> u32 {
    let pow = (resolution as u32) + 1;
    (2 as u32).pow(pow)
}

pub const SUCCESS_TUNE_NOTES: [(u32, u32); 3] = [
    (523, 200),  // C5 note for 200ms
    (659, 200),  // E5 note for 200ms
    (784, 300),  // G5 note for 300ms
];

pub const ERROR_TUNE_NOTES: [(u32, u32); 3] = [
    (330, 300),  // E4 note for 300ms
    (261, 300),  // C4 note for 300ms
    (196, 500),  // G3 note for 500ms
];

pub fn vectors_almost_equal(last: &Vector3, current: &Vector3) -> bool {
    let threshold = 0.05; // Define a small threshold to filter out noise
    (last.x - current.x).abs() < threshold &&
    (last.y - current.y).abs() < threshold &&
    (last.z - current.z).abs() < threshold
}