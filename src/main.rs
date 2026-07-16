#![no_std]
#![no_main]

use core::panic::PanicInfo;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use rp2040_hal as hal;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();

    loop {
        led_pin.set_low().unwrap();
        // timer.delay_ms(50);
        // led_pin.set_low().unwrap();
        // timer.delay_ms(50);
    }
}
