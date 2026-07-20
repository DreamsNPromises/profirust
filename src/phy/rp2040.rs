use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::uart;

use fugit::RateExtU32;
use rp2040_hal::Clock;

use embedded_hal::blocking::delay::DelayUs;

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
    pub fn is_rx(&self) -> bool {
        match self {
            PhyData::Rx { .. } => true,
            _ => false,
        }
    }

    pub fn is_tx(&self) -> bool {
        match self {
            PhyData::Tx { .. } => true,
            _ => false,
        }
    }

    pub fn make_rx(&mut self) {
        if let PhyData::Tx { buffer, .. } = self {
            let buffer = core::mem::replace(buffer, (&mut [][..]).into());
            *self = PhyData::Rx { buffer, length: 0 };
        }
    }
}

/// PHY implementation for the [RP2040] microcontroller's UART peripheral
///
/// Available with the `phy-rp2040` feature.
///
/// [RP2040]: https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html
///
/// # Example
/// ```no_run
/// # use rp2040_hal::gpio::{Pin, bank0, FunctionNull, PullNone};
/// # let clocks: rp2040_hal::clocks::ClocksManager = todo!();
/// # let pac: rp2040_hal::pac::Peripherals = todo!();
/// # struct FakePins {
/// #    pub gpio15: Pin<bank0::Gpio15, FunctionNull, PullNone>,
/// #    pub gpio16: Pin<bank0::Gpio16, FunctionNull, PullNone>,
/// #    pub gpio17: Pin<bank0::Gpio17, FunctionNull, PullNone>,
/// # }
/// # let pins: FakePins = todo!();
/// use profirust::{Baudrate, fdl, dp, phy};
/// const BAUDRATE: Baudrate = Baudrate::B19200;
///
/// let uart_pins = (
///     // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
///     pins.gpio16.into_function(),
///     // UART RX (characters received by RP2040) on pin 2 (GPIO1)
///     pins.gpio17.into_function(),
/// );
/// let uart = rp2040_hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS);
///
/// // Pin to toggle the RS485 direction (transmit vs. receive)
/// let dir_pin = pins.gpio15.into_push_pull_output();
///
/// let mut phy_buffer = [0u8; 256];
/// let mut phy = phy::Rp2040Phy::new(
///     uart,
///     dir_pin,
///     &clocks.peripheral_clock,
///     &mut phy_buffer[..],
///     BAUDRATE,
/// )
/// .unwrap();
/// ```

pub struct Rp2040Phy<'a, UART, DIR> {
    uart: UART,
    dir_pin: DIR,
    data: PhyData<'a>,
    baudrate: crate::Baudrate,
    timer: rp2040_hal::Timer,
    tset_us: u64,
    tqui_us: u64,
}

impl<'a, UART, DIR> core::fmt::Debug for Rp2040Phy<'a, UART, DIR>
where
    UART: core::fmt::Debug,
    DIR: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Rp2040Phy")
            .field("uart", &self.uart)
            .field("dir_pin", &self.dir_pin)
            .field("data", &self.data)
            .field("baudrate", &self.baudrate)
            .field("tset_us", &self.tset_us)
            .field("tqui_us", &self.tqui_us)
            .finish()
    }
}

impl<'a, D, P, DIR> Rp2040Phy<'a, uart::UartPeripheral<uart::Enabled, D, P>, DIR>
where
    D: uart::UartDevice,
    P: uart::ValidUartPinout<D>,
    DIR: OutputPin,
{
    pub fn new(
        uart: uart::UartPeripheral<uart::Disabled, D, P>,
        mut dir_pin: DIR,
        per_clock: &rp2040_hal::clocks::PeripheralClock,
        timer: rp2040_hal::Timer,
        buffer: impl Into<crate::phy::BufferHandle<'a>>,
        baudrate: crate::Baudrate,
    ) -> Result<Self, uart::Error> {
        let uart = uart.enable(
            uart::UartConfig::new(
                u32::try_from(baudrate.to_rate()).unwrap().Hz(),
                uart::DataBits::Eight,
                Some(uart::Parity::Even),
                uart::StopBits::One,
            ),
            per_clock.freq(),
        )?;

        let tset_us = (baudrate.tset_bits() as u64 * 1_000_000 + baudrate.to_rate() - 1) / baudrate.to_rate();
        let tqui_us = (baudrate.tqui_bits() as u64 * 1_000_000 + baudrate.to_rate() - 1) / baudrate.to_rate();

        // Go into RX mode.
        dir_pin.set_low().ok().unwrap();

        Ok(Self {
            uart,
            dir_pin,
            data: PhyData::Rx {
                buffer: buffer.into(),
                length: 0,
            },
            baudrate,
            timer,
            tset_us,
            tqui_us,
        })
    }

    // fn busy_wait_us(timer: &mut rp2040_hal::Timer, us: u64) {
    //     if us == 0 { return; }
    //     let start = timer.get_counter().ticks();
    //     while timer.get_counter().ticks().wrapping_sub(start) < us {}
    // }
}

impl<'a, D, P, DIR> crate::phy::ProfibusPhy
    for Rp2040Phy<'a, uart::UartPeripheral<uart::Enabled, D, P>, DIR>
where
    D: uart::UartDevice,
    P: uart::ValidUartPinout<D>,
    DIR: OutputPin,
{
    fn poll_transmission(&mut self, now: crate::time::Instant) -> bool {
        if let PhyData::Tx {
            buffer,
            length,
            cursor,
            start_tx,
        } = &mut self.data
        {
            if now < *start_tx {
                // We must still wait before beginning transmission (Tset).
                true
            } else if length != cursor {
                let pending = &buffer[*cursor..*length];
                let written = match self.uart.write_raw(pending) {
                    Ok(b) => pending.len() - b.len(),
                    Err(nb::Error::WouldBlock) => 0,
                    Err(nb::Error::Other(_)) => unreachable!(),
                };
                debug_assert!(written <= *length - *cursor);
                *cursor += written;
                true
            } else {
                let busy = self.uart.uart_is_busy();
                if !busy {
                    // Self::busy_wait_us(&self.timer, self.tqui_us);
                    // self.timer.delay_us(self.tqui_us as u32);

                    self.data.make_rx();
                    self.dir_pin.set_low().ok().unwrap();
                    log::trace!("PHY: switched to RX");
                }
                busy
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
                    // Don't transmit anything.
                    return res;
                }

                // We enable the transmitter here and then wait for Tset before poll_transmission()
                // will start scheduling bytes for transmission.
                self.dir_pin.set_high().ok().unwrap();

                // self.timer.delay_us(self.tset_us as u32);
                // Self::busy_wait_us(&self.timer, self.tset_us);

                let buffer = core::mem::replace(buffer, (&mut [][..]).into());
                self.data = PhyData::Tx {
                    buffer,
                    length,
                    cursor: 0,
                    start_tx: now + crate::time::Duration::from_micros(self.tset_us),
                };
                res
            }
        }
    }

    fn receive_data<F, R>(&mut self, now: crate::time::Instant, f: F) -> R
    where
        F: FnOnce(&[u8]) -> (usize, R),
    {
        match &mut self.data {
            PhyData::Tx { .. } => panic!("receive_data() while transmitting!"),
            PhyData::Rx { buffer, length } => {
                *length += match self.uart.read_raw(&mut buffer[*length..]) {
                    Ok(l) => l,
                    Err(nb::Error::WouldBlock) => 0,
                    Err(nb::Error::Other(_)) => {
                        // TODO: handle uart errors
                        0
                    }
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
                        // TODO: Properly implement partial buffer drops here as well. It isn't
                        // that important because this shouldn't really ever happen on a
                        // microcontroller, but having it may be needed somewhere someday anyway...
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
