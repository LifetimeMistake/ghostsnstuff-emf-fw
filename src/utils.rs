use esp_idf_hal::ledc::Resolution;

const _: () = if Resolution::Bits1 as u32 != 0 {
    panic!("Resolution::Bits1 has to be 0");
};

pub fn ledc_resolution_to_u32(resolution: Resolution) -> u32 {
    let pow = (resolution as u32) + 1;
    (2 as u32).pow(pow)
}