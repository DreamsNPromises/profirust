#![no_std]
#![no_main]

use rp_pico as bsp;
use bsp::hal::{
    self,
    clocks::init_clocks_and_plls,
    gpio::FunctionPio0,
    pio::PIOExt,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use panic_halt as _;

#[bsp::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);
    let external_xtal_freq_hz = 12_000_000u32;

    let _clocks = init_clocks_and_plls(
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

    // GPIO2 → PIO0
    let pio_pin = pins.gpio2.into_function::<FunctionPio0>();
    let pio_pin_id = pio_pin.id().num;

    // Собираем программу PIO через Assembler (без макроса pio_asm)
    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    // Настраиваем пин как выход (однократно)
    a.set(pio::SetDestination::PINDIRS, 1);
    a.bind(&mut wrap_target);
    a.set_with_delay(pio::SetDestination::PINS, 1, 0);
    a.set_with_delay(pio::SetDestination::PINS, 0, 0);
    a.bind(&mut wrap_source);
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // Инициализация PIO0
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();

    // Создаём state machine с медленным тактированием (видимое мигание)
    // Делитель: 125 MHz / 62500 = 2000 Hz PIO clock
    let (mut sm, _, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(pio_pin_id, 1)
        .clock_divisor_fixed_point(1, 0)
        .build(sm0);

    // **Важно:** явно установить пин как выход для PIO
    sm.set_pindirs([(pio_pin_id, hal::pio::PinDir::Output)]);

    sm.start();

    loop {}
}