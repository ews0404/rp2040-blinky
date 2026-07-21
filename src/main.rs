#![no_std]
#![no_main]

use embedded_hal::pwm::SetDutyCycle;
use rp2040_hal::Clock;

// HAL (not BSP)
use embedded_io::Write;
use rp2040_hal as hal;

// gpio
use embedded_hal::digital::{InputPin, OutputPin};

// USB serial driver
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

// UART
use hal::fugit::RateExtU32;
use hal::uart::{DataBits, StopBits, UartConfig};

// local panic handler def
use core::panic::PanicInfo;
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

// this is done by pico BSP, but must be done manually for HAL
#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// crystal oscillator speed
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[hal::entry]
fn main() -> ! {
    // init chip
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
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // GPIO
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut btn_pin = pins.gpio5.into_pull_up_input();

    // USB
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16C0, 0x27DD))
        .strings(&[StringDescriptors::default()
            .manufacturer("OpenStar")
            .product("my serial port")
            .serial_number("666")])
        .unwrap()
        .device_class(0x02) // CDC class ID
        .build();
    let mut usb_rx_buff = [0u8; 64];

    // UART
    let uart_pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(), // uart Tx, pin 1
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(), // uart Rx, pin 2
    );
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    let mut uart_rx_buff = [0u8; 64];

    // ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_2 = hal::adc::AdcPin::new(pins.gpio28).unwrap();


    // PWM
    {
        // init the reference frequencies, with quadrature phase
        use rp2040_hal::pwm::Slices;
        let mut pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
        const PWM_TOP: u16 = 124;
        const PWM_DIV: u8 = 1;

        // txf0
        pwm_slices.pwm1.set_div_int(PWM_DIV);
        pwm_slices.pwm1.set_top(PWM_TOP);
        pwm_slices.pwm1.channel_a.set_duty_cycle(PWM_TOP / 2);
        pwm_slices.pwm1.channel_a.output_to(pins.gpio2);

        // txf1
        pwm_slices.pwm2.set_div_int(PWM_DIV);
        pwm_slices.pwm2.set_top(PWM_TOP);
        pwm_slices.pwm2.channel_a.set_duty_cycle(PWM_TOP / 2);
        pwm_slices.pwm2.channel_a.output_to(pins.gpio4);

        // add quadrature offset
        for _i in 0..PWM_TOP / 4 {
            pwm_slices.pwm2.advance_phase();
        }

        // enable both PWM slices simultaneously
        pwm_slices.enable_simultaneous(0xFF);
    }

    // main loop
    let mut timestamp = timer.get_counter();
    let mut write_buf = [0u8; 64];
    loop {
        // call every 10 ms to keep USB alive
        if usb_dev.poll(&mut [&mut serial]) {
            match serial.read(&mut usb_rx_buff) {
                Ok(0) => {}
                Ok(count) => {
                    // blink LED high on activity (cant see it on release version), echo message in caps
                    led_pin.set_high();

                    // convert lower case letters to caps, ugly hack
                    for x in &mut usb_rx_buff[..count] {
                        if (*x >= 0x61) && (*x <= 0x7A) {
                            *x &= 0xDF;
                        }
                    }

                    // needless uart write/read step (uart0 Tx and Rx pins soldered together)
                    uart.write_full_blocking(&usb_rx_buff);
                    uart.read_full_blocking(&mut uart_rx_buff).unwrap();

                    // back out the usb
                    let _ = serial.write(&uart_rx_buff);

                    // write done, turn off LED
                    led_pin.set_low();
                }
                Err(_e) => {}
            }
        }

        // send timestamp and adc_2/button reading
        if (timer.get_counter() - timestamp).to_millis() >= 1000 {
            // adc read
            let adc_2_val: u16 = adc.read(&mut adc_pin_2).unwrap();

            // button read
            let btn_val = btn_pin.is_high().unwrap();

            // write into byte buffer
            write!(&mut write_buf[..], "t:{}, b:{}, a:{}\r\n", timestamp, btn_val, adc_2_val)
                .expect("can't write to buffer");
            let _ = serial.write(&write_buf);
            for x in &mut write_buf[..64] {
                *x = 0xF;
            } // overwrite buffer with non-zero values

            // next loop
            timestamp = timer.get_counter();
        }
    }
}
