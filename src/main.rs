//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use core::cell::RefCell;
use defmt_rtt as _;
use panic_probe as _;

use cortex_m::{
    delay::Delay,
    interrupt::{self, Mutex},
};
use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use once_cell::unsync::Lazy;
use rp_pico::{
    hal::{
        clocks::{init_clocks_and_plls, ClocksManager},
        Clock, Sio, Watchdog,
    },
    pac, Pins,
};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
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
const HIGH: u8 = 0x0;

#[allow(unused)]
const INPUT: u8 = 0x0;
#[allow(unused)]
const OUTPUT: u8 = 0x1;
#[allow(unused)]
const INPUT_PULLUP: u8 = 0x2;

// Only valid for "standard" variant of Arduino
const LED_BUILTIN: u8 = 13;

// Pico specific info
const EXTERNAL_XTAL_FREQ: u32 = 12_000_000;

#[allow(unused)]
pub struct BspData {
    watchdog: Watchdog,
    clocks: ClocksManager,
    delay: Delay,
}

impl BspData {
    fn new() -> Self {
        let mut pac = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let clocks = init_clocks_and_plls(
            EXTERNAL_XTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        let delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
        Self {
            watchdog,
            clocks,
            delay,
        }
    }
}

fn pins() -> Pins {
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);
    Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    )
}

static BSP_DATA: Mutex<Lazy<RefCell<BspData>>> =
    Mutex::new(Lazy::new(|| RefCell::new(BspData::new())));

#[no_mangle]
pub extern "C" fn pinMode(pin: u8, mode: u8) {
    info!("Setting pin {} to mode {}", pin, mode);
}

#[no_mangle]
pub extern "C" fn digitalWrite(pin: u8, val: u8) {
    let pins = pins();
    match pin {
        LED_BUILTIN => {
            let mut led = pins.led.into_push_pull_output();
            if val == LOW {
                led.set_low().unwrap();
            } else if val == HIGH {
                led.set_high().unwrap();
            } else {
                defmt::unreachable!("Invalid value to set to LED");
            }
        }
        _ => defmt::todo!("Implement write for other pins"),
    }
}

#[no_mangle]
pub extern "C" fn delay(amt_ms: u64) {
    interrupt::free(|cs| {
        let mut bsp_data = BSP_DATA.borrow(cs).borrow_mut();
        bsp_data.delay.delay_ms(amt_ms as u32);
    });
}
