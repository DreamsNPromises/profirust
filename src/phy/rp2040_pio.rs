//! PHY implementation for RP2040 using PIO for hardware-accurate DE (RS-485 direction) control.
//!
//! The PIO state machine watches the UART TX line, raises DE at the start bit,
//! and lowers DE exactly after the stop bit — independent of CPU load.

use rp2040_hal::{
    clocks::PeripheralClock,
    // pio::{self, PIOBuilder, PinDir, Running, SM0, StateMachine, UninitStateMachine},
    pio::{self as rp_pio},
    uart::{self, UartDevice, ValidUartPinout},
    Clock,
};
use rp2040_hal::fugit::RateExtU32;

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

pub struct Rp2040PioPhy<'a, D: UartDevice, P: ValidUartPinout<D>> {
    uart: uart::UartPeripheral<uart::Enabled, D, P>,
    _sm: rp_pio::StateMachine<(rp2040_hal::pac::PIO0, rp_pio::SM0), rp_pio::Running>,
    tx_fifo: *mut u32,
    data: PhyData<'a>,
    baudrate: crate::Baudrate,
}

impl<'a, D: UartDevice, P: ValidUartPinout<D>> Rp2040PioPhy<'a, D, P> {
    pub fn new(
        uart: uart::UartPeripheral<uart::Disabled, D, P>,
        mut pio: rp2040_hal::pio::PIO<rp2040_hal::pac::PIO0>,
        uninit_sm: rp_pio::UninitStateMachine<(rp2040_hal::pac::PIO0, rp_pio::SM0)>,
        per_clock: &PeripheralClock,
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

        // ---- PIO program (assembler) ----
        let mut a = pio::Assembler::<32>::new();
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut de_low = a.label();

        a.bind(&mut wrap_target);

        a.pull(false, true);
        a.mov(::pio::MovDestination::X, ::pio::MovOperation::None, ::pio::MovSource::OSR);
        a.jmp(::pio::JmpCondition::XIsZero, &mut de_low);
        // a.wait(0, ::pio::WaitSource::PIN, 0, false);      // wait 0 pin 0

        a.set_with_delay(::pio::SetDestination::PINS, 1, 0);       // set pins, 1

        a.jmp(::pio::JmpCondition::Always, &mut wrap_target);
        a.bind(&mut de_low);
        // a.set(::pio::SetDestination::X, 8);                        // set x, 8
        // let mut delay_loop = a.label();
        // a.bind(&mut delay_loop);

        // a.jmp(::pio::JmpCondition::XDecNonZero, &mut delay_loop);  // jmp x-- delay_loop
        a.wait(1, ::pio::WaitSource::PIN, 0, false);

        a.set_with_delay(::pio::SetDestination::PINS, 0, 0);       // set pins, 0
        a.bind(&mut wrap_source);
        let program = a.assemble_with_wrap(wrap_source, wrap_target);

        let installed = pio.install(&program).unwrap();

        // ---- Clock divider = per_clock / baudrate ----
        let sys_hz = per_clock.freq().raw();
        let baud_hz = baudrate.to_rate() as u32;
        let clk_int = (sys_hz / baud_hz) as u16;
        let clk_frac = (((sys_hz % baud_hz) as u32) * 256 / baud_hz) as u8;

        // ---- Build SM ----
        let (mut sm_stopped, _, _) = rp_pio::PIOBuilder::from_installed_program(installed)
            .set_pins(2, 1)     // GPIO2 = SET pin 0
            .in_pin_base(0)           // GPIO0 = IN pin 0
            .clock_divisor_fixed_point(clk_int, clk_frac)
            .build(uninit_sm);

        // Set pin directions (IN pin = input, SET pin = output)
        sm_stopped.set_pindirs([
            (0, rp_pio::PinDir::Input),    // GPIO0 (TX) -> input
            (2, rp_pio::PinDir::Output),   // GPIO2 (DE) -> output
        ]);

        let sm_running = sm_stopped.start();

        // TX FIFO SM0: base adress PIO0 + 0x010 + 0 * 0x20 = 0x50200010
        let tx_fifo = 0x5020_0010 as *mut u32;

        Ok(Self {
            uart,
            _sm: sm_running,
            tx_fifo,
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

impl<'a, D: UartDevice, P: ValidUartPinout<D>> crate::phy::ProfibusPhy
for Rp2040PioPhy<'a, D, P>
{
    fn poll_transmission(&mut self, now: crate::time::Instant) -> bool {
        if let PhyData::Tx {
            buffer,
            length,
            cursor,
            start_tx,
        } = &mut self.data
        {
            // 1. Wait for Tset (bus settling time)
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

            // 3. All bytes written — wait until UART finishes transmitting
            let busy = self.uart.uart_is_busy();
            if !busy {
                unsafe { core::ptr::write_volatile(self.tx_fifo, 0); }
                self.data.make_rx();
                log::trace!("PHY PIO: switched to RX");
            }
            busy
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

                unsafe { core::ptr::write_volatile(self.tx_fifo, 1); }
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