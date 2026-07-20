#![no_std]
#![no_main]

// HAL (not BSP)
use rp2040_hal as hal;
use embedded_io::Write;

// gpio
use embedded_hal::digital::OutputPin;

// USB serial driver
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

// UART
use hal::fugit::RateExtU32;
use hal::clocks::Clock;
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
    // init chip, set up clocks
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

    // gpio init
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // usb serial init
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
        .serial_number("666")
        ]).unwrap()
        .device_class(0x02) // CDC class ID
        .build();
    let mut usb_rx_buff=[0u8; 64];

    // uart init
     let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function(),
    );
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    let mut uart_rx_buff=[0u8; 64];

    let mut timestamp = timer.get_counter();
    let mut write_buf=[0u8; 64];
    loop {

        // call every 10 ms to keep USB alive
        if usb_dev.poll(&mut [&mut serial]){
            match serial.read(&mut usb_rx_buff){
                Ok(0)=>{}
                Ok(count)=>{

                    // blink LED high on activity (cant see it on release version), echo message in caps
                    led_pin.set_high();
                    for x in &mut usb_rx_buff[..count] {
                        if (*x>=0x61)&&(*x<=0x7A){ *x &= 0xDF; }
                    }

                    // needless uart write/read step
                    uart.write_full_blocking(&usb_rx_buff);
                    uart.read_full_blocking(& mut uart_rx_buff).unwrap();

                    // back out the usb
                    let _=serial.write(&uart_rx_buff);
                    led_pin.set_low();
                }
                Err(_e)=>{}
            }
        }

        // send something
        if(timer.get_counter()-timestamp).to_millis()>=1000{
            write!(&mut write_buf[..], "{}\r\n", timestamp).expect("can't write to buffer");
            let _=serial.write(&write_buf);
            timestamp=timer.get_counter();
        }
    }
}
