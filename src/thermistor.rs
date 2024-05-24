pub struct Thermistor {
    _a: f64,
    _b: f64,
    _c: f64,
    fixed_resistor_ohms: f64,
    reference_voltage: f64,
    adc_max: f64
}

impl Thermistor {

    pub fn new(adc_resolution_bits: u8) -> Thermistor {
        return Thermistor {
            _a: 0_f64,
            _b: 0_f64,
            _c: 0_f64,
            fixed_resistor_ohms: 10_000_f64,
            reference_voltage: 3.3,
            adc_max: (2_u16.pow(adc_resolution_bits as u32) - 1) as f64
        };
    }

    pub fn voltage(&self, adc_count: u16) -> f64 {
        self.reference_voltage * (adc_count as f64 / self.adc_max)
    }

    pub fn thermistor_resistance(&self, adc_count: u16) -> f64 {
        let voltage = self.voltage(adc_count);
        self.fixed_resistor_ohms * (1_f64 / ((self.reference_voltage/voltage) - 1_f64))
    }
}

#[cfg(test)]
mod tests {
    use crate::{thermistor};

    #[test]
    fn can_get_temperature() {
        let thermistor = thermistor::Thermistor::new(12);
        assert_eq!(thermistor.thermistor_resistance(0), 0.0);
    }
}