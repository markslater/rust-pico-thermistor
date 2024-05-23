pub struct Thermistor {
    a: f64,
    b: f64,
    c: f64,
    fixed_resistor_ohms: f64,
    reference_voltage: f64,
    adc_max: f64
}

impl Thermistor {

    pub fn new(adc_resolution_bits: u8) -> Thermistor {
        return Thermistor {
            a: 0_f64,
            b: 0_f64,
            c: 0_f64,
            fixed_resistor_ohms: 10_000_f64,
            reference_voltage: 3.3,
            adc_max: 2_u16.pow(adc_resolution_bits as u32) as f64
        };
    }

    pub fn voltage(&self, adc_count: u16) -> f64 {
        self.reference_voltage * (adc_count as f64 / self.adc_max)
    }
}