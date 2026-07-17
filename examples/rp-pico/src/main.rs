#![no_std]
#![no_main]

use bsp::hal::{self, clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use rp_pico as bsp;

use embedded_hal::digital::v2::ToggleableOutputPin;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use profirust::{dp, fdl, phy, Baudrate};

use hal::multicore::{Multicore, Stack};
use rp2040_hal::Clock;

static mut CORE1_STACK: Stack<4096> = Stack::new();

mod logger;
mod panic_handler;
mod time;
mod overclock;

const IO_ADDRESS: u8 = 3;
const SLAVE_IDENT: u16 = 0x0008;
const MASTER_ADDRESS: u8 = 2;
const BAUDRATE: Baudrate = Baudrate::B12000000;

#[bsp::entry]
fn main() -> ! {
    logger::init();
    log::info!("Booting...");

    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    let clocks = overclock::init_clocks_192mhz(
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.VREG_AND_CHIP_RESET,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    unsafe { time::init(timer); }

    log::info!(
        "System clock: {} Hz",
        clocks.system_clock.freq().to_Hz(),
    );

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // ===== UART =====
    let uart_pins = (
        pins.gpio0.into_function(),
        pins.gpio1.into_function(),
    );
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS);
    let dir_pin = pins.gpio2.into_push_pull_output();
    let mut phy_buffer = [0u8; 512];
    let mut phy = phy::Rp2040Phy::new(
        uart,
        dir_pin,
        &clocks.peripheral_clock,
        timer,
        &mut phy_buffer[..],
        BAUDRATE,
    ).unwrap();

    // ===== USB =====
    let usbctrl_regs = pac.USBCTRL_REGS;
    let usbctrl_dpram = pac.USBCTRL_DPRAM;
    let usb_clock = clocks.usb_clock;
    let resets = pac.RESETS;

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let _ = core1.spawn(unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK.mem) }, move || {
        let mut resets = resets;
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            usbctrl_regs,
            usbctrl_dpram,
            usb_clock,
            true,
            &mut resets,
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

        loop {
            logger::drain(|buf| serial.write(buf).unwrap_or(0));
            usb_dev.poll(&mut [&mut serial]);
        }
    });

    // ===== DP Master =====
    let mut buffer_inputs = [0u8; 16];
    let mut buffer_outputs = [0u8; 16];
    let mut buffer_diagnostics = [0u8; 6];

    let mut storage: [dp::PeripheralStorage; 1] = Default::default();
    let mut dp_master = dp::DpMaster::new(&mut storage[..]);

    let options = profirust::dp::PeripheralOptions {
        ident_number: SLAVE_IDENT,
        user_parameters: Some(&[]),
        config: Some(&[0x1f, 0x2f]),

        max_tsdr: match BAUDRATE {
            profirust::Baudrate::B9600 => 60,
            profirust::Baudrate::B19200 => 60,
            profirust::Baudrate::B45450 => 250,
            profirust::Baudrate::B93750 => 60,
            profirust::Baudrate::B187500 => 60,
            profirust::Baudrate::B500000 => 100,
            profirust::Baudrate::B1500000 => 150,
            profirust::Baudrate::B3000000 => 250,
            profirust::Baudrate::B6000000 => 450,
            profirust::Baudrate::B12000000 => 800,
            b => panic!(
                "Peripheral \"B-8DI/8DO      DP             \" does not support baudrate {b:?}!"
            ),
        },

        fail_safe: false,
        ..Default::default()
    };

    let io_handle = dp_master.add(
        dp::Peripheral::new(
            IO_ADDRESS,
            options,
            &mut buffer_inputs[..],
            &mut buffer_outputs[..]
        )
        .with_diag_buffer(&mut buffer_diagnostics[..]),
    );

    let mut fdl_master = fdl::FdlActiveStation::new(
        fdl::ParametersBuilder::new(MASTER_ADDRESS, BAUDRATE)
            .watchdog_timeout(profirust::time::Duration::from_secs(2))
            .slot_bits(20000)
            .highest_station_address(3)
            .max_retry_limit(3)
            .build_verified(&dp_master),
    );

    // ===== MAIN LOOP =====
    let mut init = false;
    let mut last = profirust::time::Instant::ZERO;

    let mut poll_cycle = 0u32;
    let mut stat_counter = 0u32;

    log::info!(
        "System clock: {} Hz",
        clocks.system_clock.freq().to_Hz(),
    );

    let mut poll_count: u64 = 0u64;
    let mut last_report = time::now().unwrap();

    loop {
        let now = time::now().unwrap();

        if !init && now.secs() > 1 {
            fdl_master.set_online();
            dp_master.enter_operate();
            init = true;
        }

        // for _ in 0..2000 {
        //     fdl_master.poll(now, &mut phy, &mut dp_master);
        // }
        //
        // stat_counter += 1;
        // if stat_counter >= 100 {
        //     dp_master.statistics().log_summary();
        //     stat_counter = 0;
        // }
        //
        // last = now;

        // for _ in 0..1000 {
        //     fdl_master.poll(now, &mut phy, &mut dp_master);
        // }
        //
        // if now - last >= profirust::time::Duration::from_millis(500) {
        //     dp_master.statistics().log_summary();
        //     last = now;
        // }

        for _ in 0..128 {
            fdl_master.poll(now, &mut phy, &mut dp_master);
        }
        poll_count += 1;

        if now - last_report >= profirust::time::Duration::from_secs(1) {
            let elapsed_us = (now - last_report).total_micros();
            let polls_per_sec = poll_count * 1_000_000 / elapsed_us as u64;
            log::info!(
            "poll() rate: {} calls/sec (avg {} ns/call)",
            polls_per_sec,
            elapsed_us * 1000 / poll_count
        );
            poll_count = 0;
            last_report = now;
            dp_master.statistics().log_summary();
        }
    }
}