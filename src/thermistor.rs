use libm::log;

pub struct Thermistor {
    fixed_resistor_ohms: f64,
    reference_voltage: f64,
    adc_max: f64,
    a: f64,
    b: f64,
    c: f64,
}

impl Thermistor {

    pub fn new(adc_resolution_bits: u8, fixed_resistor_ohms: f64, reference_voltage: f64, a: f64, b: f64, c: f64) -> Thermistor {
        return Thermistor {
            fixed_resistor_ohms,
            reference_voltage,
            adc_max: (2_u16.pow(adc_resolution_bits as u32) - 1) as f64,
            a,
            b,
            c,
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
        let ln_r = log(thermistor_resistance);
        let temperature_kelvin = 1_f64 / (self.a + self.b * ln_r + self.c * ln_r * ln_r * ln_r);
        temperature_kelvin - 273.15
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::thermistor;

    #[test]
    fn can_get_thermistor_resistance_0() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 1.284850279e-3, 2.076544735e-4, 2.004280704e-7);
        assert_eq!(thermistor.thermistor_resistance(0), 0.0);
    }

    #[test]
    fn can_get_thermistor_resistance_infinity() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 1.284850279e-3, 2.076544735e-4, 2.004280704e-7);
        assert_eq!(thermistor.thermistor_resistance(4095), f64::INFINITY);
    }

    #[test]
    fn can_get_thermistor_resistance_equal_to_fixed_resistance() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 1.284850279e-3, 2.076544735e-4, 2.004280704e-7);
        assert_relative_eq!(thermistor.thermistor_resistance(2047), 9995_f64, epsilon = 0.2);
    }

    #[test]
    fn can_get_sample_temperature() {
        let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 1.284850279e-3, 2.076544735e-4, 2.004280704e-7);
        assert_relative_eq!(thermistor.temperature_degrees_centigrade(2152), 22.7, epsilon = 0.1);
    }
}