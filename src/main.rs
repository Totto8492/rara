#![no_std]
#![no_main]
#![warn(clippy::pedantic)]
#![warn(clippy::nursery)]

use embedded_hal::digital::OutputPin;
use hal::{
    entry,
    gpio::PinState,
    multicore::{Multicore, Stack},
    pac, Clock, Sio, Watchdog,
};
use panic_rtt_target as _;
use rp2040_hal as hal;
use rtt_target::rprintln;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

static mut CORE1_STACK: Stack<4096> = Stack::new();

const CMD_LED_OFF: u32 = 0;
const CMD_LED_ON: u32 = 1;

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();
    rprintln!("INIT");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::bank0::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let led = pins.gpio0.into_push_pull_output_in_state(PinState::Low);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let core1 = &mut mc.cores()[1];
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_task(led))
        .unwrap();

    loop {
        delay.delay_ms(500);
        sio.fifo.write(CMD_LED_ON);
        delay.delay_ms(500);
        sio.fifo.write(CMD_LED_OFF);
    }
}

fn core1_task(mut led: impl OutputPin) {
    let pac = unsafe { pac::Peripherals::steal() };
    let mut sio = Sio::new(pac.SIO);

    loop {
        cortex_m::asm::wfe();
        if let Some(cmd) = sio.fifo.read() {
            match cmd {
                CMD_LED_OFF => led.set_low().unwrap(),
                CMD_LED_ON => led.set_high().unwrap(),
                _ => unreachable!(),
            };
        };
    }
}
