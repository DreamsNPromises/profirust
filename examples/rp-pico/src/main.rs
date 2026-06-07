#![no_std]
#![no_main]

use bsp::hal::{self, clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use rp_pico as bsp;

use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use profirust::{dp, fdl, phy, Baudrate};

mod logger;
mod panic_handler;
mod time;

const IO_ADDRESS: u8 = 3;
const SLAVE_IDENT: u16 = 0x0008;
const MASTER_ADDRESS: u8 = 1;
const BAUDRATE: Baudrate = Baudrate::B500000;

#[bsp::entry]
fn main() -> ! {
    logger::init();

    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    unsafe { time::init(timer); }

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().ok();

    // ===== USB =====
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Rahix Automation")
            .product("PROFIRUST PICO")
            .serial_number("PICO01")])
        .unwrap()
        .device_class(2)
        .build();

    // Макрос для сброса логов (избегает проблем с lifetime)
    macro_rules! flush {
        () => {
            logger::drain(|buf| serial.write(buf).unwrap_or(0));
            usb_dev.poll(&mut [&mut serial]);
        };
    }

    let usb_start = timer.get_counter();
    loop {
        usb_dev.poll(&mut [&mut serial]);
        if serial.dtr() {
            break; // хост открыл порт
        }
        if timer.get_counter().ticks() - usb_start.ticks() > 10_000_000 {
            break; // таймаут, едем дальше без хоста
        }
    }

    // Стартовые сообщения
    log::info!("=== PICO PROFIBUS MASTER ===");
    log::info!("Master addr: {}, Slave addr: {}", MASTER_ADDRESS, IO_ADDRESS);
    log::info!("Slave ident: 0x{:04x}", SLAVE_IDENT);
    log::info!("Baudrate: {:?}", BAUDRATE);
    flush!();

    // ===== UART =====
    let uart_pins = (
        pins.gpio0.into_function(),
        pins.gpio1.into_function(),
    );
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS);

    let mut dir_pin = pins.gpio2.into_push_pull_output();
    dir_pin.set_high().ok();

    let mut phy_buffer = [0u8; 256];
    let mut phy = phy::Rp2040Phy::new(
        uart, dir_pin, &clocks.peripheral_clock,
        &mut phy_buffer[..], BAUDRATE,
    ).unwrap();
    log::info!("PHY UART initialized");
    flush!();

    // ===== DP Master =====
    let mut buffer_inputs = [0u8; 16];
    let mut buffer_outputs = [0u8; 16];
    let mut buffer_diagnostics = [0u8; 64];
    let mut storage: [dp::PeripheralStorage; 1] = Default::default();
    let mut dp_master = dp::DpMaster::new(&mut storage[..]);

    let options = profirust::dp::PeripheralOptions {
        ident_number: SLAVE_IDENT,
        user_parameters: None,
        config: Some(&[0x1f, 0x2f]),
        max_tsdr: 100,
        fail_safe: false,
        ..Default::default()
    };

    let io_handle = dp_master.add(
        dp::Peripheral::new(IO_ADDRESS, options,
            &mut buffer_inputs[..], &mut buffer_outputs[..])
            .with_diag_buffer(&mut buffer_diagnostics[..]),
    );

    let fdl_params = fdl::ParametersBuilder::new(MASTER_ADDRESS, BAUDRATE)
        .watchdog_timeout(profirust::time::Duration::from_secs(1))
        .slot_bits(300)
        .max_retry_limit(3)
        .build_verified(&dp_master);

    let mut fdl_master = fdl::FdlActiveStation::new(fdl_params);

    log::info!("Init complete, entering main loop");
    flush!();

    // ===== MAIN LOOP =====
    let mut init = false;
    let mut last = profirust::time::Instant::ZERO;
    let mut bus_error_count = 0u32;

    loop {
        let now = time::now().unwrap();

        if !init && now.secs() > 2 {
            log::info!("Setting master ONLINE + OPERATE");
            flush!();

            fdl_master.set_online();
            dp_master.enter_operate();
            init = true;
        }

        fdl_master.poll(now, &mut phy, &mut dp_master);
        let events = dp_master.take_last_events();

        {
            let io_station = dp_master.get_mut(io_handle);
            if events.cycle_completed && io_station.is_running() {
                // data exchange active
            }
        }

        if last.secs() != now.secs() {
            let io_station = dp_master.get_mut(io_handle);
            if io_station.is_running() {
                let inp = io_station.pi_i();
                let adc_val = (inp[0] as u16) << 8 | inp[1] as u16;
                log::info!(
                    "SLAVE OK! ADC: {}, DIP: 0x{:02x}, bytes: {:02x} {:02x} {:02x} {:02x}",
                    adc_val, inp[2], inp[0], inp[1], inp[2], inp[3]
                );
                led_pin.toggle().ok();
            } else {
                bus_error_count += 1;
                log::info!(
                    "Waiting for slave #{}... (t={}s, errors={})",
                    IO_ADDRESS, now.secs(), bus_error_count
                );
                led_pin.toggle().ok();
            }
            flush!();
        }

        usb_dev.poll(&mut [&mut serial]);
        last = now;
    }
}