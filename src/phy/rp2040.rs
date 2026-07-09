use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::uart;

use fugit::RateExtU32;
use rp2040_hal::Clock;

use rp2040_hal::pac;
use rp2040_hal::pac::interrupt;
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

pub static IRQ_HITS: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
pub static BYTES_SEEN: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

const RING_CAP: usize = 512;

struct RxRing {
    buffer: [u8; RING_CAP],
    head: usize,
    tail: usize,
    len: usize,
}

impl RxRing {
    const fn new() -> Self {
        RxRing { buffer: [0; RING_CAP], head: 0, tail: 0, len: 0 }
    }

    fn push(&mut self, byte: u8) {
        if self.len == RING_CAP {
            // переполнение: либо дропаем новый байт, либо сдвигаем tail —
            // на первом шаге просто считаем событие и дропаем новый байт
            return;
        }
        self.buffer[self.head] = byte;
        self.head = (self.head + 1) % RING_CAP;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            None
        } else {
            let byte = self.buffer[self.tail];
            self.tail = (self.tail + 1) % RING_CAP;
            self.len -= 1;
            Some(byte)
        }
    }

    fn available(&self) -> usize {
        if self.head >= self.tail {
            self.head - self.tail
        } else {
            self.buffer.len() - (self.tail - self.head)
        }
    }
}

static RX_RING: Mutex<RefCell<RxRing>> = Mutex::new(RefCell::new(RxRing::new()));

#[interrupt]
fn UART0_IRQ() {
    let uart = unsafe { &*pac::UART0::ptr() };

    if !uart.uartfr().read().rxfe().bit_is_set() {
        cortex_m::interrupt::free(|cs| {
            *IRQ_HITS.borrow(cs).borrow_mut() += 1;
            let mut ring = RX_RING.borrow(cs).borrow_mut();

            while uart.uartfr().read().rxfe().bit_is_clear() {
                let byte = uart.uartdr().read().data().bits();
                ring.push(byte);
                *BYTES_SEEN.borrow(cs).borrow_mut() += 1;
            }
        });
    }

    uart.uarticr().write(|w| unsafe {
        // 0b01111110 (Clear all mask bits at once)
        w.bits(0x7E)
    });

    // uart.uarticr().write(|w| {
    //     w.rtic().clear_bit_by_one();   // Receive interrupt
    //     w.oeic().clear_bit_by_one();   // Overrun error
    //     w.beic().clear_bit_by_one();   // Break error
    //     w.peic().clear_bit_by_one();   // Parity error
    //     w.feic().clear_bit_by_one();   // Framing error
    //     w
    // });

    // uart.uarticr().write(|w| w.rtic().clear_bit_by_one());
}

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
#[derive(Debug)]
pub struct Rp2040Phy<'a, UART, DIR> {
    uart: UART,
    dir_pin: DIR,
    data: PhyData<'a>,
    baudrate: crate::Baudrate,
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
        buffer: impl Into<crate::phy::BufferHandle<'a>>,
        baudrate: crate::Baudrate,
    ) -> Result<Self, uart::Error> {
        let mut uart = uart.enable(
            uart::UartConfig::new(
                u32::try_from(baudrate.to_rate()).unwrap().Hz(),
                uart::DataBits::Eight,
                Some(uart::Parity::Even),
                uart::StopBits::One,
            ),
            per_clock.freq(),
        )?;

        // Go into RX mode.
        dir_pin.set_low().ok().unwrap();

        uart.enable_rx_interrupt();
        // unsafe { pac::NVIC::unmask(pac::Interrupt::UART0_IRQ); }
        let imsc = unsafe { (*pac::UART0::ptr()).uartimsc().read().bits() };
        log::info!("UARTIMSC = {:#012b}", imsc);
        unsafe { pac::NVIC::unmask(pac::Interrupt::UART0_IRQ); }

        Ok(Self {
            uart,
            dir_pin,
            data: PhyData::Rx {
                buffer: buffer.into(),
                length: 0,
            },
            baudrate,
        })
    }
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
                    self.data.make_rx();
                    self.dir_pin.set_low().ok().unwrap();

                    for _ in 0..1000 {
                        cortex_m::asm::nop();
                    }

                    let uart = unsafe { &*pac::UART0::ptr() };
                    uart.uarticr().write(|w| unsafe {
                        w.bits(0x7E)
                    });

                    // while self.uart.uart_is_readable() {
                    //     let _ = self.uart.read_raw(&mut [0u8; 1]);
                    // }

                    // cortex_m::interrupt::free(|cs| {
                    //     let mut ring = RX_RING.borrow(cs).borrow_mut();
                    //     while ring.pop().is_some() {}
                    // });

                    self.uart.enable_rx_interrupt();
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
                self.uart.disable_rx_interrupt();
                self.dir_pin.set_high().ok().unwrap();
                // TODO: Tset is not always 1 bit time
                // let t_set = self.baudrate.bits_to_time(1);
                let t_set = self.baudrate.bits_to_time(
                    if self.baudrate.to_rate() >= 6_000_000 {
                        15
                    } else if self.baudrate.to_rate() > 1_500_000 {
                        5
                    } else {
                        1
                    }
                );

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
                cortex_m::interrupt::free(|cs| {
                    let mut ring = RX_RING.borrow(cs).borrow_mut();
                    while *length < buffer.len() {
                        match ring.pop() {
                            Some(b) => {
                                buffer[*length] = b;
                                *length += 1;
                            }
                            None => break,
                        }
                    }
                });

                debug_assert!(*length <= buffer.len());
                let (drop, res) = f(&buffer[..*length]);
                match drop {
                    0 => (),
                    d if d == *length => *length = 0,
                    d => {
                        // TODO: Properly implement partial buffer drops here as well. It isn't
                        // that important because this shouldn't really ever happen on a
                        // microcontroller, but having it may be needed somewhere someday anyway...
                        buffer.copy_within(d..*length, 0);
                        *length -= d;
                    }
                }
                res
            }
        }
    }
}
