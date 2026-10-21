//! Simulated DP-V0 stations of this device.
//!
//! One RP2040 emulates several DP-V0 slaves at the same time (step 5 of
//! `docs/PLAN-dp-slave-simulator-final.md`).  Everything that differs between the stations is
//! described by the [`SLAVES`] table: the station address, the LED which shows bit 0 of the
//! master's output byte and the increment of the test counter which stands in for a real input.
//!
//! All stations share one profile (see [`gsd-files/PRF_0D0F.gsd`]): identical ident number, user
//! parameters and configuration, so a master configures each of them the same way - only the bus
//! address is different.
//!
//! The process images (`pi_i`/`pi_q`) live in `static` memory so that the [`DpSlave`] objects can
//! borrow them for `'static`.  [`Slaves`] owns the station array together with the LED pins and
//! does the housekeeping: watchdog, test counters, LED update and lifecycle logging.
//!
//! [`gsd-files/PRF_0D0F.gsd`]: ../../../gsd-files/PRF_0D0F.gsd

use core::array;

use embedded_hal::digital::v2::OutputPin;
use profirust::dp::{self, DpSlave, SlaveConfig, SlaveState};
use profirust::fdl::Telegram;
use profirust::time::{Duration, Instant};
use rp2040_hal::gpio::{DynBankId, DynPinId, FunctionSio, Pin, PullDown, SioOutput};

// ===========================================================================
// Station profile (`gsd-files/PRF_0D0F.gsd`)
//
// The GSD file describes this station type to a DP master's configuration tool; the constants
// below must stay identical to it, otherwise the slaves reject `Set_Prm`/`Chk_Cfg`.
// ===========================================================================

/// Ident number, arbitrary (see the GSD file).
pub const IDENT_NUMBER: u16 = 0x0D0F;
/// `User_Prm_Data` of the GSD.
pub const USER_PARAMETERS: &[u8] = &[0x20];
/// `Chk_Cfg` of the GSD: 1 byte of inputs + 1 byte of outputs.
pub const CONFIG: &[u8] = &[0x42, 0x00, 0xBE, 0x81, 0x82, 0x00, 0xBD, 0x81];

/// Length of the input process image (`pi_i`, slave -> master).
pub const PI_I_LEN: usize = 1;
/// Length of the output process image (`pi_q`, master -> slave).
pub const PI_Q_LEN: usize = 1;

/// Number of simulated stations on this device.
pub const N_SLAVES: usize = 2;

// ===========================================================================
// Station table
// ===========================================================================

/// One simulated DP-V0 station.
pub struct SlaveDef {
    /// Station address on the bus (unique within this device).
    pub address: u8,
    /// External LED which mirrors bit 0 of the station's output process image (`pi_q`).
    pub led_pin: DynPinId,
    /// Increment of the test counter in `pi_i[0]` per second.
    ///
    /// The counter replaces a real input signal (a button in the original plan): different
    /// increments make the stations distinguishable in the master's input log.
    pub counter_step: u8,
}

/// GPIO pin of bank 0.
const fn gpio(num: u8) -> DynPinId {
    DynPinId {
        bank: DynBankId::Bank0,
        num,
    }
}

/// All simulated stations.
///
/// The LEDs must be passed to [`Slaves::new()`] in exactly this order.
pub const SLAVES: [SlaveDef; N_SLAVES] = [
    SlaveDef {
        address: 3,
        led_pin: gpio(15),
        counter_step: 1,
    },
    SlaveDef {
        address: 4,
        led_pin: gpio(14),
        counter_step: 2,
    },
];

// ===========================================================================
// Process image storage
// ===========================================================================

/// Input process images of all stations (`pi_i`, slave -> master).
static mut PI_I: [[u8; PI_I_LEN]; N_SLAVES] = [[0; PI_I_LEN]; N_SLAVES];
/// Output process images of all stations (`pi_q`, master -> slave).
static mut PI_Q: [[u8; PI_Q_LEN]; N_SLAVES] = [[0; PI_Q_LEN]; N_SLAVES];

/// Output pin driving the LED of one station.
pub type LedPin = Pin<DynPinId, FunctionSio<SioOutput>, PullDown>;

/// The simulated slave stations of this device.
pub struct Slaves {
    /// Station state machines, driven by [`dp::dispatch()`].
    stations: [DpSlave<'static>; N_SLAVES],
    /// LED of each station, in [`SLAVES`] order.
    leds: [LedPin; N_SLAVES],
    /// Lifecycle state which was last logged for each station.
    states: [SlaveState; N_SLAVES],
    /// `pi_q` byte which was last written to the LED of each station.
    outputs: [u8; N_SLAVES],
    /// Timestamp of the last advance of the test counters.
    last_counter: Instant,
}

impl Slaves {
    /// Set up all simulated stations.
    ///
    /// `leds` must be the LED pins of the stations in [`SLAVES`] order; a mismatch is caught by a
    /// debug assertion.  The LEDs are switched off initially.
    ///
    /// Must be called at most once: it hands out `'static` borrows of the module's static process
    /// images, so a second [`Slaves`] would alias the first one's `pi_i`/`pi_q`.
    pub fn new(mut leds: [LedPin; N_SLAVES]) -> Self {
        for (def, led) in SLAVES.iter().zip(&leds) {
            debug_assert_eq!(
                led.id(),
                def.led_pin,
                "LED of station #{} is wired to the wrong pin",
                def.address
            );
        }
        for led in &mut leds {
            let _ = led.set_low();
        }

        // SAFETY: `Slaves::new()` is called exactly once, before the main loop starts.  Every
        // station gets its own row of the two process image arrays (`addr_of_mut!` on distinct
        // indices), so the resulting `&'static mut` slices never alias.  The referenced statics
        // are only ever accessed through these slices after this point.
        let stations = array::from_fn(|i| {
            let pi_i: &'static mut [u8; PI_I_LEN] =
                unsafe { &mut *core::ptr::addr_of_mut!(PI_I[i]) };
            let pi_q: &'static mut [u8; PI_Q_LEN] =
                unsafe { &mut *core::ptr::addr_of_mut!(PI_Q[i]) };

            DpSlave::new(
                SlaveConfig {
                    address: SLAVES[i].address,
                    ident_number: IDENT_NUMBER,
                    user_parameters: Some(USER_PARAMETERS),
                    config: Some(CONFIG),
                    fail_safe: true,
                },
                &mut pi_i[..],
                &mut pi_q[..],
            )
        });

        Self {
            stations,
            leds,
            states: [SlaveState::default(); N_SLAVES],
            outputs: [0; N_SLAVES],
            last_counter: Instant::ZERO,
        }
    }

    /// Pass a received telegram to all stations and return the length of the reply, if any.
    ///
    /// Must be called from inside `ProfibusPhy::receive_telegram()`; `out` is the caller's reply
    /// buffer and must be at least [`dp::MAX_REPLY_LEN`] bytes long.
    pub fn dispatch(&mut self, now: Instant, telegram: &Telegram, out: &mut [u8]) -> usize {
        dp::dispatch(&mut self.stations[..], now, telegram, out)
    }

    /// Largest minimum TSDR (in bits, at least 11) requested by any station.
    ///
    /// The reply delay has to satisfy every station, so it uses the maximum: the responding
    /// station is not known before the telegram was parsed.
    pub fn min_tsdr_bits(&self) -> u32 {
        self.stations
            .iter()
            .map(|station| u32::from(station.min_tsdr_bits()))
            .max()
            .unwrap_or(11)
            .max(11)
    }

    /// Periodic housekeeping: station watchdogs, test counters, LEDs and lifecycle logging.
    ///
    /// Returns `true` when the 1 Hz test counters advanced (the caller uses this as the heartbeat
    /// for the on-board LED).
    pub fn tick(&mut self, now: Instant) -> bool {
        let counters_advanced = now - self.last_counter >= Duration::from_secs(1);
        if counters_advanced {
            self.last_counter = now;
            for (station, def) in self.stations.iter_mut().zip(&SLAVES) {
                if let Some(byte) = station.pi_i_mut().first_mut() {
                    *byte = byte.wrapping_add(def.counter_step);
                }
            }
        }

        for (i, station) in self.stations.iter_mut().enumerate() {
            // Watchdog: brings the station back to `Wprm` when the master goes silent.
            station.tick(now);

            let state = station.state();
            if state != self.states[i] {
                self.states[i] = state;
                log::info!("DP slave #{} is now {:?}", SLAVES[i].address, state);
            }

            // Output process image: bit 0 of `pi_q` drives the external LED of the station.
            // The watchdog (`fail_safe`) clears `pi_q`, which turns the LED off when the master
            // stops exchanging data.
            let outputs = station.pi_q().first().copied().unwrap_or(0);
            if outputs != self.outputs[i] {
                self.outputs[i] = outputs;
                let _ = self.leds[i].set_state(((outputs & 1) != 0).into());
            }
        }

        counters_advanced
    }
}
