use cortex_m::delay::Delay;
use cortex_m::peripheral::*;
use rp_pico::hal::clocks::{init_clocks_and_plls, ClocksManager};
use rp_pico::hal::gpio::{bank0::*, Output, Pin, PinId, PushPull};
use rp_pico::hal::{Clock, Sio, Watchdog};
use rp_pico::pac;
use rp_pico::pac::*;

#[allow(non_snake_case)]
pub struct Peripherals {
    pub ADC: ADC,
    pub BUSCTRL: BUSCTRL,
    pub DMA: DMA,
    pub I2C0: I2C0,
    pub I2C1: I2C1,
    pub IO_QSPI: IO_QSPI,
    pub PADS_QSPI: PADS_QSPI,
    pub PIO0: PIO0,
    pub PIO1: PIO1,
    pub PPB: PPB,
    pub PSM: PSM,
    pub PWM: PWM,
    pub RESETS: RESETS,
    pub ROSC: ROSC,
    pub RTC: RTC,
    pub SPI0: SPI0,
    pub SPI1: SPI1,
    pub SYSCFG: SYSCFG,
    pub SYSINFO: SYSINFO,
    pub TBMAN: TBMAN,
    pub TIMER: TIMER,
    pub UART0: UART0,
    pub UART1: UART1,
    pub USBCTRL_DPRAM: USBCTRL_DPRAM,
    pub USBCTRL_REGS: USBCTRL_REGS,
    pub VREG_AND_CHIP_RESET: VREG_AND_CHIP_RESET,
    pub XIP_CTRL: XIP_CTRL,
    pub XIP_SSI: XIP_SSI,
}

#[allow(non_snake_case)]
pub struct CorePeripherals {
    pub CBP: CBP,
    pub CPUID: CPUID,
    pub DCB: DCB,
    pub DWT: DWT,
    pub FPB: FPB,
    pub FPU: FPU,
    pub ICB: ICB,
    pub ITM: ITM,
    pub MPU: MPU,
    pub NVIC: NVIC,
    pub SAU: SAU,
    pub SCB: SCB,
    pub TPIU: TPIU,
}

pub struct Pins {
    pub gpio0: Pin<Gpio0, <Gpio0 as PinId>::Reset>,
    pub gpio1: Pin<Gpio1, <Gpio1 as PinId>::Reset>,
    pub gpio2: Pin<Gpio2, <Gpio2 as PinId>::Reset>,
    pub gpio3: Pin<Gpio3, <Gpio3 as PinId>::Reset>,
    pub gpio4: Pin<Gpio4, <Gpio4 as PinId>::Reset>,
    pub gpio5: Pin<Gpio5, <Gpio5 as PinId>::Reset>,
    pub gpio6: Pin<Gpio6, <Gpio6 as PinId>::Reset>,
    pub gpio7: Pin<Gpio7, <Gpio7 as PinId>::Reset>,
    pub gpio8: Pin<Gpio8, <Gpio8 as PinId>::Reset>,
    pub gpio9: Pin<Gpio9, <Gpio9 as PinId>::Reset>,
    pub gpio10: Pin<Gpio10, <Gpio10 as PinId>::Reset>,
    pub gpio11: Pin<Gpio11, <Gpio11 as PinId>::Reset>,
    pub gpio12: Pin<Gpio12, <Gpio12 as PinId>::Reset>,
    pub gpio13: Pin<Gpio13, <Gpio13 as PinId>::Reset>,
    pub gpio14: Pin<Gpio14, <Gpio14 as PinId>::Reset>,
    pub gpio15: Pin<Gpio15, <Gpio15 as PinId>::Reset>,
    pub gpio16: Pin<Gpio16, <Gpio16 as PinId>::Reset>,
    pub gpio17: Pin<Gpio17, <Gpio17 as PinId>::Reset>,
    pub gpio18: Pin<Gpio18, <Gpio18 as PinId>::Reset>,
    pub gpio19: Pin<Gpio19, <Gpio19 as PinId>::Reset>,
    pub gpio20: Pin<Gpio20, <Gpio20 as PinId>::Reset>,
    pub gpio21: Pin<Gpio21, <Gpio21 as PinId>::Reset>,
    pub gpio22: Pin<Gpio22, <Gpio22 as PinId>::Reset>,
    pub b_power_save: Pin<Gpio23, <Gpio23 as PinId>::Reset>,
    pub vbus_detect: Pin<Gpio24, <Gpio24 as PinId>::Reset>,
    pub gpio26: Pin<Gpio26, <Gpio26 as PinId>::Reset>,
    pub gpio27: Pin<Gpio27, <Gpio27 as PinId>::Reset>,
    pub gpio28: Pin<Gpio28, <Gpio28 as PinId>::Reset>,
    pub voltage_monitor: Pin<Gpio29, <Gpio29 as PinId>::Reset>,
}

const EXTERNAL_XTAL_FREQ: u32 = 12_000_000;

#[allow(unused)]
pub struct BspData {
    pub watchdog: Watchdog,
    pub clocks: ClocksManager,
    pub delay: Delay,
    pub peripherals: Peripherals,
    pub core_peripherals: CorePeripherals,
    pub pins: Pins,
    pub led: Pin<Gpio25, Output<PushPull>>,
}

impl BspData {
    pub fn new() -> Self {
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

        let sio = Sio::new(pac.SIO);
        let rpp = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let led = rpp.led.into_push_pull_output();

        let peripherals = Peripherals {
            ADC: pac.ADC,
            BUSCTRL: pac.BUSCTRL,
            DMA: pac.DMA,
            I2C0: pac.I2C0,
            I2C1: pac.I2C1,
            IO_QSPI: pac.IO_QSPI,
            PADS_QSPI: pac.PADS_QSPI,
            PIO0: pac.PIO0,
            PIO1: pac.PIO1,
            PPB: pac.PPB,
            PSM: pac.PSM,
            PWM: pac.PWM,
            RESETS: pac.RESETS,
            ROSC: pac.ROSC,
            RTC: pac.RTC,
            SPI0: pac.SPI0,
            SPI1: pac.SPI1,
            SYSCFG: pac.SYSCFG,
            SYSINFO: pac.SYSINFO,
            TBMAN: pac.TBMAN,
            TIMER: pac.TIMER,
            UART0: pac.UART0,
            UART1: pac.UART1,
            USBCTRL_DPRAM: pac.USBCTRL_DPRAM,
            USBCTRL_REGS: pac.USBCTRL_REGS,
            VREG_AND_CHIP_RESET: pac.VREG_AND_CHIP_RESET,
            XIP_CTRL: pac.XIP_CTRL,
            XIP_SSI: pac.XIP_SSI,
        };

        let core_peripherals = CorePeripherals {
            CBP: core.CBP,
            CPUID: core.CPUID,
            DCB: core.DCB,
            DWT: core.DWT,
            FPB: core.FPB,
            FPU: core.FPU,
            ICB: core.ICB,
            ITM: core.ITM,
            MPU: core.MPU,
            NVIC: core.NVIC,
            SAU: core.SAU,
            SCB: core.SCB,
            TPIU: core.TPIU,
        };

        let pins = Pins {
            gpio0: rpp.gpio0,
            gpio1: rpp.gpio1,
            gpio2: rpp.gpio2,
            gpio3: rpp.gpio3,
            gpio4: rpp.gpio4,
            gpio5: rpp.gpio5,
            gpio6: rpp.gpio6,
            gpio7: rpp.gpio7,
            gpio8: rpp.gpio8,
            gpio9: rpp.gpio9,
            gpio10: rpp.gpio10,
            gpio11: rpp.gpio11,
            gpio12: rpp.gpio12,
            gpio13: rpp.gpio13,
            gpio14: rpp.gpio14,
            gpio15: rpp.gpio15,
            gpio16: rpp.gpio16,
            gpio17: rpp.gpio17,
            gpio18: rpp.gpio18,
            gpio19: rpp.gpio19,
            gpio20: rpp.gpio20,
            gpio21: rpp.gpio21,
            gpio22: rpp.gpio22,
            b_power_save: rpp.b_power_save,
            vbus_detect: rpp.vbus_detect,
            gpio26: rpp.gpio26,
            gpio27: rpp.gpio27,
            gpio28: rpp.gpio28,
            voltage_monitor: rpp.voltage_monitor,
        };

        Self {
            watchdog,
            clocks,
            delay,
            peripherals,
            core_peripherals,
            led,
            pins,
        }
    }
}
