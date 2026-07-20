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
use rp2040_hal::gpio::{FunctionUart, OutputSlewRate, OutputDriveStrength};

use heapless::spsc::{Producer, Consumer, Queue};
use core::sync::atomic::Ordering;

static mut LOG_QUEUE: Queue<u8, 32768> = Queue::new();
static mut CORE1_STACK: Stack<4096> = Stack::new();

// mod logger;
mod panic_handler;
mod time;
mod overclock;
mod logger_atomic;

const IO_ADDRESS: u8 = 3;
const SLAVE_IDENT: u16 = 0x0008;
const MASTER_ADDRESS: u8 = 2;
const BAUDRATE: Baudrate = Baudrate::B3000000;

#[bsp::entry]
fn main() -> ! {
    let mut log_consumer = logger_atomic::init();
    log::info!("Booting...");

    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    let clocks = overclock::init_clocks(
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
    let mut uart_tx = pins.gpio0.into_function::<FunctionUart>();
    uart_tx.set_slew_rate(OutputSlewRate::Fast);
    uart_tx.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);

    let uart_pins = (
        uart_tx,
        pins.gpio1.into_function(),
    );

    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS);

    let mut dir_pin = pins.gpio2.into_push_pull_output();
    dir_pin.set_slew_rate(OutputSlewRate::Fast);
    dir_pin.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);

    {
        let _framing  = pins.gpio6.into_push_pull_output();
        let _protocol = pins.gpio7.into_push_pull_output();
        // let _timeout  = pins.gpio9.into_push_pull_output();
        let _retry    = pins.gpio8.into_push_pull_output();

        let sio_ptr = unsafe { &pac::Peripherals::steal().SIO as *const _ as *const () };
        unsafe {
            logger_atomic::set_error_pin(logger_atomic::ErrorCategory::Framing,  6, sio_ptr);
            logger_atomic::set_error_pin(logger_atomic::ErrorCategory::Protocol, 7, sio_ptr);
            // logger_atomic::set_error_pin(logger_atomic::ErrorCategory::Timeout,  9, sio_ptr);
            logger_atomic::set_error_pin(logger_atomic::ErrorCategory::Retry,    8, sio_ptr);
        }
    }


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

        let mut last_drain = time::now().unwrap();
        loop {
            if logger_atomic::PANICKING.load(Ordering::Relaxed) {
                // Stop touching USB/the queue; let the panic handler take over.
                loop { cortex_m::asm::wfe(); }
            }
            let now = time::now().unwrap();
            if now - last_drain >= profirust::time::Duration::from_millis(1) {
                logger_atomic::drain(&mut log_consumer, |buf| serial.write(buf).unwrap_or(0));
                last_drain = now;
            }
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
            .watchdog_timeout(profirust::time::Duration::from_secs(1))
            .slot_bits(1000)
            .highest_station_address(3)
            .max_retry_limit(1)
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

    let mut max_poll_ns: u64 = 0;

    loop {
        let now = time::now().unwrap();

        if !init && now.secs() > 1 {
            fdl_master.set_online();
            dp_master.enter_operate();
            init = true;
        }

        for _ in 0..32 {
            let t0 = time::now().unwrap();
            fdl_master.poll(now, &mut phy, &mut dp_master);
            let dt = (time::now().unwrap() - t0).total_micros();
            if dt as u64 > max_poll_ns {
                max_poll_ns = dt as u64;
                log::warn!("New max poll latency: {} us", dt);
            }
        }
        poll_count += 1;

        if now - last_report >= profirust::time::Duration::from_millis(100) {
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