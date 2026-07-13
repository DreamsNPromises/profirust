//! Minimal PIO test: toggle GPIO2 every ~1 second.
//! If this works, PIO is alive. If not, we know PIO config is the problem.
//!
//! Build and flash this separately, then check GPIO2 with logic analyzer / LED.

#![no_std]
#![no_main]
use bsp::hal::{
    self,
    clocks::init_clocks_and_plls,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico as bsp;

use rp2040_hal::{
    gpio::FunctionPio0,
    pio::{PIOExt, PIOBuilder, Buffers, SM0},
};
use arrayvec::ArrayVec;
use embedded_hal::digital::v2::OutputPin;

mod panic_handler;
mod logger;
mod time;


// PIO program: just toggle SET pin 0, wait ~1M cycles, repeat
//
//   .wrap_target
//       set pins, 1       ; GPIO2 HIGH
//       set x, 31         ; outer loop
//   outer:
//       set y, 31         ; inner loop
//   inner:
//       jmp y-- inner     ; 32 * 32 = 1024 iterations
//       jmp x-- outer
//       set pins, 0       ; GPIO2 LOW
//       set x, 31
//   outer2:
//       set y, 31
//   inner2:
//       jmp y-- inner2
//       jmp x-- outer2
//   .wrap
//
// At clock_div = 125MHz/9600 ≈ 13020: 1024 * 13020 ≈ 13.3M PIO clocks between toggles
// Actually we just want visible blinking, so let's use a bigger divider.

fn make_blink_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    let mut code: ArrayVec<u16, { pio::RP2040_MAX_PROGRAM_SIZE }> = ArrayVec::new();
    //  0: set pins, 1     = 101_00000_00000001 = 0xA001
    //  1: set x, 31       = 101_00001_00011111 = 0xA11F
    //  2: set y, 31       = 101_00010_00011111 = 0xA21F
    //  3: jmp y-- 3       = 000_100_00000_00011 = 0x1003
    //  4: jmp x-- 2       = 000_010_00000_00010 = 0x0802
    //  5: set pins, 0     = 101_00000_00000000 = 0xA000
    //  6: set x, 31       = 101_00001_00011111 = 0xA11F
    //  7: set y, 31       = 101_00010_00011111 = 0xA21F
    //  8: jmp y-- 8       = 000_100_00000_01000 = 0x1008
    //  9: jmp x-- 7       = 000_010_00000_00111 = 0x0807
    code.try_push(0xA001).unwrap(); //  0: set pins, 1
    code.try_push(0xA11F).unwrap(); //  1: set x, 31
    code.try_push(0xA21F).unwrap(); //  2: set y, 31
    code.try_push(0x1003).unwrap(); //  3: jmp y-- 3
    code.try_push(0x0802).unwrap(); //  4: jmp x-- 2
    code.try_push(0xA000).unwrap(); //  5: set pins, 0
    code.try_push(0xA11F).unwrap(); //  6: set x, 31
    code.try_push(0xA21F).unwrap(); //  7: set y, 31
    code.try_push(0x1008).unwrap(); //  8: jmp y-- 8
    code.try_push(0x0807).unwrap(); //  9: jmp x-- 7

    pio::Program {
        code,
        origin: None,
        wrap: pio::Wrap { source: 9, target: 0 },
        side_set: pio::SideSet::default(),
    }
}

#[bsp::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED on GPIO25 — to confirm code is running
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    // GPIO2 → PIO0 function
    let _de_pio = pins.gpio2.into_mode::<FunctionPio0>();

    // Split PIO0
    let (mut pio, sm0, _sm1, _sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);

    // Install blink program
    let program = make_blink_program();
    let installed = pio.install(&program).unwrap();

    // Use a very slow clock so we can see blinking on a multimeter/analyzer
    // clk_div ≈ 125MHz / 1000Hz = 125000 → ~1ms per PIO clock
    // 1024 iterations = ~1 second per toggle
    let builder = PIOBuilder::from_installed_program(installed)
        .set_pins(2, 1)                          // GPIO2 = SET pin 0
        .clock_divisor_fixed_point(62500, 0)      // ~2000 Hz PIO clock, visible blinking
        .buffers(Buffers::RxTx);

    let (sm_stopped, _rx, _tx) = builder.build(sm0);
    let _sm_running = sm_stopped.start();  // starts toggling immediately

    // Blink LED in main loop to confirm CPU is alive
    let mut led_on = true;
    loop {
        led_on = !led_on;
        if led_on {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
        cortex_m::asm::delay(12_000_000); // ~1 second at 125MHz
    }
}