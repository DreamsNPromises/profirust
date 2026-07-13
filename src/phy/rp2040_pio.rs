//! PHY implementation for RP2040 using PIO for hardware-accurate DE (RS-485 direction) control.
//!
//! Unlike the standard [`Rp2040Phy`], this implementation offloads the DE pin timing
//! to a PIO state machine.  The PIO watches the UART TX line, raises DE at the start bit,
//! and lowers DE exactly after the stop bit — with single-cycle (≈8 ns) precision,
//! independent of CPU load.
//!
//! This is critical for baudrates ≥ 6 Mbit/s where CPU-mediated GPIO toggling
//! introduces too much jitter.

use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    pac::{self, PIO0},
    pio::{
        Buffers, InstalledProgram, PIOBuilder, PinDir, PinState, PIOExt, Running, SM0, Stopped,
        StateMachine, UninitStateMachine, ValidStateMachine,
    },
    uart,
};
use arrayvec::ArrayVec;

use fugit::RateExtU32;
use rp2040_hal::Clock;

// ---------------------------------------------------------------------------
// PIO program (raw machine code — no assembler dependency needed)
// ---------------------------------------------------------------------------
//
// Assembly source (for reference):
//
//   .wrap_target
//       wait 0 pin 0       ; wait for TX falling edge (start bit)
//       set pins, 1        ; raise DE
//       set x, 8           ; loop counter = 8
//   delay_loop:
//       jmp x-- delay_loop  ; 8 iterations (bits 1-8: data + parity)
//       set pins, 0        ; lower DE (after stop bit)
//       irq nowait 0       ; signal CPU that transmission is done
//   .wrap
//
// Timing (in PIO clocks, where 1 PIO clock = 1 UART bit time):
//   t=0  : wait completes           → start bit begins
//   t=1  : set pins, 1              → DE high, still within start bit
//   t=2  : set x, 8
//   t=3-10: 8 × jmp x--             → data bits + parity bit
//   t=10 : jmp with X=0 falls through
//   t=11 : set pins, 0              → DE low exactly after stop bit
//   t=12 : irq nowait 0
//   t=13 : wrap → back to wait
//
// Encoded instructions (see RP2040 datasheet §3.4.2):
const PIO_INSTRUCTIONS: [u16; 6] = [
    0x0500, //  0: wait 0 pin 0     (op=WAIT, src=PIN, pol=0, idx=0)
    0x1C20, //  1: set pins, 1      (op=SET, dst=PINS, data=1)
    0x1D01, //  2: set x, 8         (op=SET, dst=X, data=8)
    0x0043, //  3: jmp x-- 3        (op=JMP, cond=X--, addr=3)
    0x1C00, //  4: set pins, 0      (op=SET, dst=PINS, data=0)
    0x1800, //  5: irq nowait 0     (op=IRQ, set, nowait, num=0)
];

// wrap_target = instruction 0, wrap = after instruction 5
const WRAP_TARGET: u8 = 0;
const WRAP: u8 = 5;

fn make_program() -> pio::Program<{pio::RP2040_MAX_PROGRAM_SIZE}> {
    let mut code: ArrayVec<u16, {pio::RP2040_MAX_PROGRAM_SIZE}> = ArrayVec::new();
    code.try_push(0x2000).unwrap(); //  0: wait 0 pin 0
    code.try_push(0xA001).unwrap(); //  1: set pins, 1
    code.try_push(0xA102).unwrap(); //  2: set x, 2
    code.try_push(0x1B05).unwrap(); //  3: jmp pin  5
    code.try_push(0x0002).unwrap(); //  4: jmp      2
    code.try_push(0x0803).unwrap(); //  5: jmp x--  3
    code.try_push(0xA000).unwrap(); //  6: set pins, 0
    code.try_push(0x8000).unwrap(); //  7: irq nowait 0

    pio::Program {
        code,
        origin: None,
        wrap: pio::Wrap {
            source: 7,       // wrap — после последней инструкции
            target: 0,       // wrap target — первая инструкция
        },
        side_set: pio::SideSet::default(),
    }
}


// ---------------------------------------------------------------------------
// PhyData (same as in rp2040.rs)
// ---------------------------------------------------------------------------

#[derive(Debug)]
enum PhyData<'a> {
    Rx {
        buffer: crate::phy::BufferHandle<'a>,
        length: usize,
    },
    Tx {
        buffer: crate::phy::BufferHandle<'a>,
        length: usize,
        cursor: usize,
        start_tx: crate::time::Instant,
    },
}

impl PhyData<'_> {
    fn make_rx(&mut self) {
        if let PhyData::Tx { buffer, .. } = self {
            let buffer = core::mem::replace(buffer, (&mut [][..]).into());
            *self = PhyData::Rx { buffer, length: 0 };
        }
    }
}

// ---------------------------------------------------------------------------
// Rp2040PioPhy
// ---------------------------------------------------------------------------

pub struct Rp2040PioPhy<'a, D, P>
where
    D: uart::UartDevice,
    P: uart::ValidUartPinout<D>,
{
    uart: uart::UartPeripheral<uart::Enabled, D, P>,
    pio: rp2040_hal::pio::PIO<PIO0>,
    _sm: StateMachine<(PIO0, SM0), Running>,
    data: PhyData<'a>,
    baudrate: crate::Baudrate,
}

impl<'a, D, P> Rp2040PioPhy<'a, D, P>
where
    D: uart::UartDevice,
    P: uart::ValidUartPinout<D>,
{
    /// Create a new `Rp2040PioPhy`.
    ///
    /// * `uart`       — disabled UART peripheral (will be enabled inside).
    /// * `dir_pin`    — the RS-485 direction pin.  It is **not** used directly; instead
    ///                  the PIO state machine will drive this GPIO.  The pin is consumed
    ///                  here only for backwards-compatibility and to ensure it is reserved.
    /// * `pio_sm`     — a **stopped** PIO state machine, pre-configured with
    ///   - `sm.set_in_pins(&[&tx_pio_pin])`   (GPIO0 = UART TX)
    ///   - `sm.set_set_pins(&[&de_pio_pin])`  (GPIO2 = DE)
    ///   - correct pin directions
    /// * `per_clock`  — peripheral clock (used for UART baudrate divider).
    /// * `buffer`     — backing buffer for TX/RX data.
    /// * `baudrate`   — PROFIBUS baudrate.
    pub fn new(
        uart: uart::UartPeripheral<uart::Disabled, D, P>,
        mut pio: rp2040_hal::pio::PIO<PIO0>,
        uninit_sm: UninitStateMachine<(PIO0, SM0)>,
        per_clock: &rp2040_hal::clocks::PeripheralClock,
        buffer: impl Into<crate::phy::BufferHandle<'a>>,
        baudrate: crate::Baudrate,
    ) -> Result<Self, uart::Error> {
        // ---- Enable UART (same as Rp2040Phy) ----
        let uart = uart.enable(
            uart::UartConfig::new(
                u32::try_from(baudrate.to_rate()).unwrap().Hz(),
                uart::DataBits::Eight,
                Some(uart::Parity::Even),
                uart::StopBits::One,
            ),
            per_clock.freq(),
        )?;

        // // ---- Configure PIO clock divider ----
        // // We want 1 PIO clock = 1 UART bit time.
        // // clk_div = per_clock_freq / baudrate
        // let sys_hz = per_clock.freq().raw();
        // let baud_hz = baudrate.to_rate() as u32;
        // // Fixed-point 16.8: int = sys_hz / baud_hz, frac = remainder * 256 / baud_hz
        // let int = (sys_hz / baud_hz) as u16;
        // let frac = (((sys_hz % baud_hz) as u32) * 256 / baud_hz) as u8;
        // sm.set_clkdiv_int_frac(int, frac);

        // // ---- Load PIO program and start ----
        // let program = pio_program();
        // let installed = sm.load_program(&program);
        // let running_sm = installed.start();
        //
        // Ok(Self {
        //     uart,
        //     sm: running_sm,
        //     data: PhyData::Rx {
        //         buffer: buffer.into(),
        //         length: 0,
        //     },
        //     baudrate,
        // })

        // ---- Собрать PIO-программу ----
        // `pio` crate: Program — кортежная структура (code: [u16; 32], wrap_target, wrap)
        // let mut code = [0u16; pio::RP2040_MAX_PROGRAM_SIZE as usize];
        // code[..PIO_INSTRUCTIONS.len()].copy_from_slice(&PIO_INSTRUCTIONS);
        let program = make_program();

        // ---- Установить программу в PIO ----
        let installed = pio.install(&program).unwrap();

        let sys_hz = per_clock.freq().raw();
        let baud_hz = baudrate.to_rate() as u32;
        let clk_int = (sys_hz / baud_hz) as u16;
        let clk_frac = (((sys_hz % baud_hz) as u32) * 256 / baud_hz) as u8;

        // ---- Построить конфигурацию SM через PIOBuilder ----
        // GPIO0 = TX → вход PIO (in_base = 0)
        // GPIO2 = DE → set-выход PIO (set_base = 2, set_count = 1)
        let builder = PIOBuilder::from_installed_program(installed)
            .in_pin_base(0)        // GPIO0 → IN
            .set_pins(2, 1)        // GPIO2 → SET, 1 пин
            .clock_divisor_fixed_point(clk_int, clk_frac)
            .buffers(Buffers::RxTx);

        // ---- Собрать SM (получаем SM, Rx, Tx) ----
        let (sm_stopped, _rx, _tx) = builder.build(uninit_sm);

        // ---- Запустить SM ----
        let sm_running = sm_stopped.start();

        Ok(Self {
            uart,
            pio,
            _sm: sm_running,
            data: PhyData::Rx {
                buffer: buffer.into(),
                length: 0,
            },
            baudrate,
        })
    }
}

// ---------------------------------------------------------------------------
// ProfibusPhy implementation
// ---------------------------------------------------------------------------

impl<'a, D, P> crate::phy::ProfibusPhy for Rp2040PioPhy<'a, D, P>
where
    D: uart::UartDevice,
    P: uart::ValidUartPinout<D>,
{
    fn poll_transmission(&mut self, now: crate::time::Instant) -> bool {
        if let PhyData::Tx {
            buffer,
            length,
            cursor,
            start_tx,
        } = &mut self.data
        {
            // 1. Wait for Tset (bus settling time) if needed
            if now < *start_tx {
                return true;
            }

            // 2. Feed bytes into UART FIFO
            if length != cursor {
                let pending = &buffer[*cursor..*length];
                let written = match self.uart.write_raw(pending) {
                    Ok(b) => pending.len() - b.len(),
                    Err(nb::Error::WouldBlock) => 0,
                    Err(nb::Error::Other(_)) => unreachable!(),
                };
                *cursor += written;
                return true;
            }

            // 3. All bytes written — check if PIO has finished
            //    PIO raises IRQ0 after the stop bit, signalling DE is low again.
            // let irq_pending = self.pio.get_irq_raw() & (1 << 0) != 0;

            if self.pio.get_irq_raw() & 0x01 != 0 {
                // Clear PIO IRQ, transition to RX mode
                self.pio.clear_irq(0x01);
                self.data.make_rx();
                log::trace!("PHY PIO: switched to RX");
                false
            } else {
                // Still transmitting (PIO hasn't finished yet)
                true
            }
        } else {
            false
        }
    }

    fn transmit_data<F, R>(&mut self, now: crate::time::Instant, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> (usize, R),
    {
        match &mut self.data {
            PhyData::Tx { .. } => panic!("transmit_data() while already transmitting!"),
            PhyData::Rx {
                buffer,
                length: receive_length,
            } => {
                if *receive_length != 0 {
                    log::warn!(
                        "{} bytes in the receive buffer and we go into transmission?",
                        receive_length
                    );
                }
                let (length, res) = f(&mut buffer[..]);
                if length == 0 {
                    return res;
                }

                // PIO is already watching the TX pin via `wait 0 pin 0`.
                // It will automatically raise DE when the start bit appears.
                // Nothing to do here — the PIO is free-running.

                // Tset: wait 1 bit time before actually feeding UART (spec requirement)
                let t_set = self.baudrate.bits_to_time(1);

                let buffer = core::mem::replace(buffer, (&mut [][..]).into());
                self.data = PhyData::Tx {
                    buffer,
                    length,
                    cursor: 0,
                    start_tx: now + t_set,
                };
                res
            }
        }
    }

    fn receive_data<F, R>(&mut self, _now: crate::time::Instant, f: F) -> R
    where
        F: FnOnce(&[u8]) -> (usize, R),
    {
        match &mut self.data {
            PhyData::Tx { .. } => panic!("receive_data() while transmitting!"),
            PhyData::Rx { buffer, length } => {
                *length += match self.uart.read_raw(&mut buffer[*length..]) {
                    Ok(l) => l,
                    Err(nb::Error::WouldBlock) => 0,
                    Err(nb::Error::Other(_)) => 0,
                };

                let log_len = core::cmp::min(*length, 20);
                if log_len > 0 {
                    log::trace!("PHY RX raw bytes: {:02X?}", &buffer[..log_len]);
                }

                debug_assert!(*length <= buffer.len());
                let (drop, res) = f(&buffer[..*length]);
                match drop {
                    0 => (),
                    d if d == *length => *length = 0,
                    d => {
                        log::warn!(
                            "ignoring partial drop of receive buffer ({} of {})",
                            d,
                            *length
                        );
                        *length = 0;
                    }
                }
                res
            }
        }
    }
}