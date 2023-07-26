//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod sync;
mod types;

use bsp::entry;
use defmt_rtt as _;
use panic_probe as _;

use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use once_cell::unsync::Lazy;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use self::sync::NullLock;
use self::types::BspData;
// use sparkfun_pro_micro_rp2040 as bsp;

extern "C" {
    fn setup();
    fn r#loop();
}

#[entry]
fn main() -> ! {
    info!("Program start");
    unsafe {
        setup();
        loop {
            r#loop();
        }
    }
}

// End of file

// Constants valid for all boards
const LOW: u8 = 0x0;
const HIGH: u8 = 0x1;

#[allow(unused)]
const INPUT: u8 = 0x0;
#[allow(unused)]
const OUTPUT: u8 = 0x1;
#[allow(unused)]
const INPUT_PULLUP: u8 = 0x2;

// Only valid for "standard" variant of Arduino
const LED_BUILTIN: u8 = 13;

static BSP_DATA: NullLock<Lazy<BspData>> = NullLock::new(Lazy::new(BspData::new));

#[no_mangle]
pub extern "C" fn pinMode(pin: u8, mode: u8) {
    info!("Setting pin {} to mode {}", pin, mode);
}

#[no_mangle]
pub extern "C" fn digitalWrite(pin: u8, val: u8) {
    BSP_DATA.lock(|bsp_data| match pin {
        LED_BUILTIN => {
            if val == LOW {
                bsp_data.led.set_low().unwrap();
            } else if val == HIGH {
                bsp_data.led.set_high().unwrap();
            } else {
                defmt::unreachable!("Invalid value to set to LED");
            }
        }
        _ => defmt::todo!("Implement write for other pins"),
    });
}

#[no_mangle]
pub extern "C" fn delay(amt_ms: u64) {
    BSP_DATA.lock(|bsp_data| bsp_data.delay.delay_ms(amt_ms as u32));
}
