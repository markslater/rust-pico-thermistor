use libm::log;

pub struct Thermistor {
    _a: f64,
    b: f64,
    _c: f64,
    fixed_resistor_ohms: f64,
    reference_voltage: f64,
    adc_max: f64
}

impl Thermistor {

    pub fn new(adc_resolution_bits: u8, fixed_resistor_ohms: f64, reference_voltage: f64, b: f64) -> Thermistor {
        return Thermistor {
            _a: 0_f64,
            b,
            _c: 0_f64,
            fixed_resistor_ohms,
            reference_voltage,
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

    pub fn temperature_degrees_centigrade(&self, adc_count: u16) -> f64 {
        let thermistor_resistance = self.thermistor_resistance(adc_count);
        -273.15 + 1.0/(1.0/298.15 + log(thermistor_resistance / self.fixed_resistor_ohms) / self.b)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::thermistor;

    #[test]
    fn can_get_thermistor_resistance_0() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 3650_f64);
        assert_eq!(thermistor.thermistor_resistance(0), 0.0);
    }

    #[test]
    fn can_get_thermistor_resistance_infinity() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 3650_f64);
        assert_eq!(thermistor.thermistor_resistance(4095), f64::INFINITY);
    }

    #[test]
    fn can_get_thermistor_resistance_equal_to_fixed_resistance() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 3650_f64);
        assert_relative_eq!(thermistor.thermistor_resistance(2047), 9995_f64, epsilon = 0.2);
    }

    #[test]
    fn can_get_sample_temperature() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 3650_f64);
        assert_relative_eq!(thermistor.temperature_degrees_centigrade(2152), 22.5, epsilon = 0.2);
    }
}