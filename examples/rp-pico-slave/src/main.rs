//! DP-V0 slave simulator for the RP2040 (`profirust` example).
//!
//! This firmware implements step 3 of `docs/PLAN-dp-slave-simulator-final.md`: a second Pico on
//! the same PROFIBUS bus as the DP master (`examples/rp-pico`) emulates one DP-V0 slave station.
//! It answers the master's `Slave_Diag`/`Set_Prm`/`Chk_Cfg` requests, takes part in cyclic data
//! exchange and does its own watchdog handling - all through a single [`profirust::dp::DpSlave`]
//! that is driven by [`profirust::dp::dispatch()`] from the main loop.
//!
//! The physical layer is the same as for the master: UART0 (GPIO0 = TX, GPIO1 = RX) with an
//! RS-485 transceiver and GPIO2 as the direction pin.
//!
//! Log output goes through a single-producer/single-consumer queue to core1, which forwards it
//! over USB CDC.  Nothing is printed from the PROFIBUS hot path on core0.
//!
//! Not implemented yet (later steps of the plan): LED/button on the simulated process image
//! (step 4), several slaves per device (step 5) and speeds above 500 kbit/s (steps 6/7).

#![no_std]
#![no_main]

use bsp::hal::{self, pac, sio::Sio, watchdog::Watchdog};
use rp_pico as bsp;

use core::sync::atomic::Ordering;
use embedded_hal::digital::v2::OutputPin;
use profirust::phy::ProfibusPhy;
use profirust::{dp, phy, time::Duration, Baudrate};

use hal::multicore::{Multicore, Stack};
use rp2040_hal::gpio::{FunctionUart, OutputDriveStrength, OutputSlewRate};
use rp2040_hal::Clock;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

mod logger_atomic;
mod overclock;
mod panic_handler;
mod time;

static mut CORE1_STACK: Stack<4096> = Stack::new();

// ===========================================================================
// Test-bench profile (see section 7 of the plan)
//
// The station profile must match what the DP master is configured with.  When
// testing against the Unigateway master, the gateway must be configured with
// this slave (GSD `PXC_00F0.gsd`, ident 0x00F0) and exactly the modules
// "IB IL 24 DI 8" (first) and "IB IL 24 DO 8" (second); the gateway derives
// `user_parameters`/`config` from the GSD, which yields exactly the constants
// below.
// ===========================================================================

/// Station address of the simulated slave.
const SLAVE_ADDRESS: u8 = 3;
/// Ident number from the GSD (`PXC_00F0.gsd`).
const SLAVE_IDENT: u16 = 0x00F0;
/// `User_Prm_Data`: one coupler parameter byte from the GSD.
const USER_PARAMETERS: &[u8] = &[0x20];
/// `Chk_Cfg` data: "IB IL 24 DI 8" (1 byte inputs) + "IB IL 24 DO 8" (1 byte outputs).
const CONFIG: &[u8] = &[0x42, 0x00, 0xBE, 0x81, 0x82, 0x00, 0xBD, 0x81];

/// Bus speed.  Start with 9.6 kbit/s (the speed of the current master setup), then raise it
/// together with the master in steps 6/7 of the plan.
const BAUDRATE: Baudrate = Baudrate::B9600;

/// Length of the input process image (`pi_i`, slave -> master): 1 byte from the DI8 module.
const PI_I_LEN: usize = 1;
/// Length of the output process image (`pi_q`, master -> slave): 1 byte from the DO8 module.
const PI_Q_LEN: usize = 1;

#[bsp::entry]
fn main() -> ! {
    let mut log_consumer = logger_atomic::init();
    log::info!("Booting PROFIBUS DP-V0 slave simulator...");

    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    // ===== Clocks (240 MHz) and system timer =====
    let clocks = overclock::init_clocks(
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.VREG_AND_CHIP_RESET,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    unsafe {
        time::init(timer);
    }

    log::info!("System clock: {} Hz", clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // On-board LED: blinks together with the simulated input byte (life sign, no bus traffic).
    let mut led = pins.led.into_push_pull_output();

    // ===== UART0 / RS-485 =====
    let mut uart_tx = pins.gpio0.into_function::<FunctionUart>();
    uart_tx.set_slew_rate(OutputSlewRate::Fast);
    uart_tx.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);

    let uart_pins = (
        uart_tx,
        // UART RX (characters received by the RP2040) on GPIO1
        pins.gpio1.into_function(),
    );

    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS);

    // Pin to toggle the RS485 direction (transmit vs. receive)
    let mut dir_pin = pins.gpio2.into_push_pull_output();
    dir_pin.set_slew_rate(OutputSlewRate::Fast);
    dir_pin.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);

    let mut phy_buffer = [0u8; 512];
    let mut phy = phy::Rp2040Phy::new(
        uart,
        dir_pin,
        &clocks.peripheral_clock,
        timer,
        &mut phy_buffer[..],
        BAUDRATE,
    )
    .unwrap();

    // ===== USB CDC on core1: logging only =====
    let usbctrl_regs = pac.USBCTRL_REGS;
    let usbctrl_dpram = pac.USBCTRL_DPRAM;
    let usb_clock = clocks.usb_clock;
    let resets = pac.RESETS;

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let _ = core1.spawn(
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK.mem) },
        move || {
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
                    .product("PROFIRUST PICO SLAVE")
                    .serial_number("PICO-SLAVE-01")])
                .unwrap()
                .device_class(2)
                .build();

            let mut last_drain = time::now().unwrap();
            loop {
                if logger_atomic::PANICKING.load(Ordering::Relaxed) {
                    // Stop touching USB/the queue; let the panic handler take over.
                    loop {
                        cortex_m::asm::wfe();
                    }
                }
                let now = time::now().unwrap();
                if now - last_drain >= Duration::from_millis(1) {
                    logger_atomic::drain(&mut log_consumer, |buf| serial.write(buf).unwrap_or(0));
                    last_drain = now;
                }
                usb_dev.poll(&mut [&mut serial]);
            }
        },
    );

    // ===== DP-V0 slave =====
    //
    // `pi_i`/`pi_q` are the process images: the master's outputs end up in `pi_q`, our `pi_i`
    // is what the master reads as inputs.  Both are plain slices owned by this function; the
    // slave accesses them through its accessors.
    let mut pi_i = [0u8; PI_I_LEN];
    let mut pi_q = [0u8; PI_Q_LEN];

    let slave = dp::DpSlave::new(
        dp::SlaveConfig {
            address: SLAVE_ADDRESS,
            ident_number: SLAVE_IDENT,
            user_parameters: Some(USER_PARAMETERS),
            config: Some(CONFIG),
            fail_safe: true,
        },
        &mut pi_i[..],
        &mut pi_q[..],
    );
    let mut slaves = [slave];
    let mut tx_buffer = [0u8; dp::MAX_REPLY_LEN];

    log::info!(
        "DP-V0 slave #{}: ident {:#06X}, {:?}, pi_i {} B, pi_q {} B",
        SLAVE_ADDRESS,
        SLAVE_IDENT,
        BAUDRATE,
        PI_I_LEN,
        PI_Q_LEN,
    );

    // ===== Main loop (core0): bus hot path =====
    let mut last_state = slaves[0].state();
    let mut last_counter = time::now().unwrap();

    loop {
        let now = time::now().unwrap();

        // While a transmission is in progress the PHY must not be touched.  Once the last byte
        // of a request has arrived, `receive_telegram()` calls the closure exactly once.
        if !phy.poll_transmission(now) {
            let reply = phy
                .receive_telegram(now, |res| match res {
                    // `dispatch()` must run inside the closure: the telegram borrows the PHY
                    // receive buffer.  The reply is copied into `tx_buffer` and transmitted
                    // after the closure returned.
                    Ok(telegram) => dp::dispatch(&mut slaves, now, &telegram, &mut tx_buffer),
                    // Broken telegram: stay silent and let the master retry.
                    Err(_) => 0,
                })
                .unwrap_or(0);

            if reply > 0 {
                // The first byte of the reply must not leave before the minimum TSDR has
                // passed.  `transmit_data()` delays the actual transmission by Tset itself,
                // so only the difference has to be waited out here.
                wait_min_tsdr(&slaves);

                let tx_now = time::now().unwrap();
                phy.transmit_data(tx_now, |buf| {
                    buf[..reply].copy_from_slice(&tx_buffer[..reply]);
                    (reply, ())
                });
            }
        }

        // Housekeeping: slave watchdog (also covers the case that the master went silent).
        for slave in slaves.iter_mut() {
            slave.tick(now);
        }

        // Step 3 stand-in for the button (step 4): change the input byte once per second so
        // that the master visibly sees new data.
        if now - last_counter >= Duration::from_secs(1) {
            last_counter = now;
            if let Some(byte) = slaves[0].pi_i_mut().first_mut() {
                *byte = byte.wrapping_add(1);
                led.set_state(((*byte & 1) != 0).into()).ok();
            }
        }

        // Report lifecycle transitions.  This is not in the per-telegram hot path.
        let state = slaves[0].state();
        if state != last_state {
            log::info!("DP slave #{} is now {:?}", SLAVE_ADDRESS, state);
            last_state = state;
        }
    }
}

/// Wait for the minimum TSDR of all slaves (at least 11 bit times) after the end of the request.
///
/// [`Rp2040Phy::transmit_data()`] starts the transmission `Tset` bit times after the `now` that
/// is passed to it, so waiting for `need - Tset` here results in a total response delay of
/// `need`, which is what the plan (section 6) asks for.
fn wait_min_tsdr(slaves: &[dp::DpSlave<'_>]) {
    let min_tsdr_bits = slaves
        .iter()
        .map(|slave| u32::from(slave.min_tsdr_bits()))
        .max()
        .unwrap_or(11)
        .max(11);

    let rate = BAUDRATE.to_rate();
    let need_us = (u64::from(min_tsdr_bits) * 1_000_000).div_ceil(rate);
    let tset_us = (u64::from(BAUDRATE.tset_bits()) * 1_000_000).div_ceil(rate);

    let Some(wait_us) = need_us.checked_sub(tset_us) else {
        // Tset is already longer than the minimum TSDR (e.g. at 12 Mbit/s): nothing to do.
        return;
    };

    let start = time::now().unwrap();
    while (time::now().unwrap() - start).total_micros() < wait_us {
        core::hint::spin_loop();
    }
}
