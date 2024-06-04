//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to uppercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
//noinspection ALL
#[cfg(not(test))]
use panic_halt as _;// Used to demonstrate writing formatted strings
use core::fmt::Write;

// GPIO traits
use embedded_hal::digital::OutputPin;
// use cortex_m::prelude::_embedded_hal_adc_OneShot;
use heapless::String;
// The macro for our start-up function
#[cfg(not(test))]
use rp_pico::entry;
// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;
// USB Device support
use usb_device::{class_prelude::*, prelude::*};
// USB Communications Class Device support
use usbd_serial::SerialPort;

mod thermistor;

const _THERMISTOR_A: f64 = 1.284850279e-3;
const _THERMISTOR_B: f64 = 2.076544735e-4;
const _THERMISTOR_C: f64 = 2.004280704e-7;


/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then echoes any characters
/// received over USB Serial.
#[cfg_attr(not(test), entry)]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

    let mut pin_0_fifo = adc.build_fifo()
        .clock_divider(0, 0) // sample as fast as possible (500ksps. This is the default)
        .set_channel(&mut adc_pin_0)
        .start();


    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let thermistor = thermistor::Thermistor::new(12, 10_000_f64, 3.3, 3650_f64);

    let mut said_hello = false;
    let mut led_on = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            let _ = serial.write(text.as_bytes());
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
        if (timer.get_counter().ticks() % 1_000_000) < 500_000 {
            if led_on {
                led_pin.set_low().unwrap();
                led_on = false;
            }
        } else {
            if !led_on {
                led_pin.set_high().unwrap();
                led_on = true;
                let pin_0_adc_counts: u16 = pin_0_fifo.read(); // actually only 12 bits of data
                let temperature = thermistor.temperature_degrees_centigrade(pin_0_adc_counts);
                // let log_thermistor_resistance = log(thermistor_resistance);
                // let c_term = THERMISTOR_C * (log_thermistor_resistance);
                // let revised_temperature: f64 = THERMISTOR_A + (THERMISTOR_B * log_thermistor_resistance) + (c_term * c_term * c_term);
                let mut text: String<64> = String::new();
                // writeln!(&mut text, "Voltage: {voltage:.3}\r\n").unwrap();
                // writeln!(&mut text, "Thermistor resistance: {thermistor_resistance:.1} Ω\r\n").unwrap();
                // writeln!(&mut text, "Revised temperature: {revised_temperature:.1}\r\n").unwrap();
                writeln!(&mut text, "ADC value: {pin_0_adc_counts}\r\n").unwrap();
                writeln!(&mut text, "Temperature: {temperature:.1}°C\r\n").unwrap();
                let _ = serial.write(text.as_bytes());
            }
        }
    }
}