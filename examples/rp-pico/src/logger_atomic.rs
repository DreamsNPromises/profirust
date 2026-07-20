use core::cell::UnsafeCell;
use core::fmt::Write;
use heapless::spsc::{Consumer, Producer, Queue};
use core::sync::atomic::{AtomicBool, Ordering};

/// Set by the panic handler on core0. core1 must check this every loop
/// iteration and, once true, stop touching USB/the log queue entirely and
/// park itself — ceding exclusive hardware+queue access to the panic handler
pub static PANICKING: AtomicBool = AtomicBool::new(false);
const CAP: usize = 32768;

static mut LOG_QUEUE: Queue<u8, CAP> = Queue::new();

// ---------- GPIO error signaling ----------

#[derive(Copy, Clone)]
pub enum ErrorCategory {
    Framing,
    Protocol,
    Timeout,
    Retry,
}

static mut ERROR_PINS: [u8; 4] = [0xFF; 4];
const PULSE_TICKS: u8 = 10;
static mut SIO_BASE: *const () = core::ptr::null();

pub unsafe fn set_error_pin(cat: ErrorCategory, pin: u8, sio: *const ()) {
    if pin <= 29 {
        ERROR_PINS[cat as usize] = pin;
    }
    SIO_BASE = sio;
}

/// Toggle the error pin quickly to create a visible pulse.
unsafe fn pulse_pin(pin: u8) {
    if pin > 29 || SIO_BASE.is_null() {
        return;
    }
    let sio = &*(SIO_BASE as *const rp_pico::hal::pac::SIO);
    let mask = 1u32 << pin;
    for _ in 0..PULSE_TICKS {
        sio.gpio_out_xor().write(|w| w.bits(mask));
    }
    // Ensure the pin ends low (assumes idle low).
    sio.gpio_out_clr().write(|w| w.bits(mask));
}

// ---------- Logger ----------

// SAFETY: `log()` is only ever called from core0 (the FDL polling loop never migrates cores
// in this firmware), so there is exactly one writer. This wrapper only exists to satisfy
// `Sync` for the static; it does not itself provide any synchronization.
struct ProducerCell(UnsafeCell<Option<Producer<'static, u8, CAP>>>);
unsafe impl Sync for ProducerCell {}

static PRODUCER: ProducerCell = ProducerCell(UnsafeCell::new(None));

struct ProducerWriter<'a>(&'a mut Producer<'static, u8, CAP>);

impl core::fmt::Write for ProducerWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.as_bytes() {
            // Queue full -> drop the byte. Cheaper than the old truncation-indicator logic,
            // and full events should now be rare since core1 drains frequently.
            let _ = self.0.enqueue(*b);
        }
        Ok(())
    }
}

struct SpscLogger;

impl log::Log for SpscLogger {
    fn enabled(&self, _: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        // --- fast keyword classification (done before queue formatting) ---
        let mut buf = [0u8; 128];
        let mut written = 0;
        {
            struct SliceWriter<'a>(&'a mut [u8], &'a mut usize);
            impl core::fmt::Write for SliceWriter<'_> {
                fn write_str(&mut self, s: &str) -> core::fmt::Result {
                    let avail = &mut self.0[*self.1..];
                    let len = s.len().min(avail.len());
                    avail[..len].copy_from_slice(&s.as_bytes()[..len]);
                    *self.1 += len;
                    Ok(())
                }
            }
            let mut w = SliceWriter(&mut buf, &mut written);
            let _ = write!(w, "{}", record.args());
        }
        // We parse as UTF-8 lossily – only used for substring checks.
        let msg = core::str::from_utf8(&buf[..written]).unwrap_or("");

        unsafe {
            if msg.contains("framing error") {
                pulse_pin(ERROR_PINS[ErrorCategory::Framing as usize]);
            }
            if msg.contains("Token lost")
            {
                pulse_pin(ERROR_PINS[ErrorCategory::Protocol as usize]);
            }
            if msg.contains("timeout") {
                pulse_pin(ERROR_PINS[ErrorCategory::Timeout as usize]);
            }
            if msg.contains("Resending") {
                pulse_pin(ERROR_PINS[ErrorCategory::Retry as usize]);
            }
        }

        let timestamp = crate::time::now().unwrap_or(profirust::time::Instant::ZERO);
        let color = match record.level() {
            log::Level::Error => "\x1B[31m",
            log::Level::Warn => "\x1B[1m",
            log::Level::Info => "",
            log::Level::Debug | log::Level::Trace => "\x1B[2m",
        };

        // SAFETY: only core0 ever calls into `log`, so this is not actually concurrent
        // with itself. No other synchronization needed for a single writer.
        let producer = unsafe { &mut *PRODUCER.0.get() };
        let Some(producer) = producer else { return };
        let mut writer = ProducerWriter(producer);

        if let Some(module_path) = record.module_path() {
            let _ = write!(
                writer,
                "\x1B[32m[{:5}.{:06}] \x1B[33m{}\x1B[0m: {}{}\x1B[0m\r\n",
                timestamp.secs(),
                timestamp.micros(),
                module_path.trim_start_matches("vlab_ethernet_bridge_firmware::"),
                color,
                record.args()
            );
        } else {
            let _ = write!(
                writer,
                "\x1B[32m[{:12}] {}{}\r\n",
                timestamp,
                color,
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

static LOGGER: SpscLogger = SpscLogger;

/// Call once, on core0, before core1 is spawned. Returns the `Consumer` half which must be
/// moved into core1's closure.
pub fn init() -> Consumer<'static, u8, CAP> {
    #[allow(static_mut_refs)]
    let (producer, consumer) = unsafe { LOG_QUEUE.split() };
    unsafe {
        *PRODUCER.0.get() = Some(producer);
    }
    unsafe {
        log::set_logger_racy(&LOGGER)
            .map(|()| log::set_max_level_racy(log::LevelFilter::Info))
            .unwrap();
    }
    consumer
}

/// Call from core1 only. Pulls up to `tmp`'s length of bytes out of the queue and hands them
/// to `f` (e.g. a USB serial write). Bytes are removed from the queue as soon as they're
/// dequeued; if `f` doesn't accept all of them (e.g. USB buffer momentarily full), those bytes
/// are lost rather than requeued. Given `f` is called every 1ms this should be rare in
/// practice.
pub fn drain<F: FnMut(&[u8]) -> usize>(consumer: &mut Consumer<'static, u8, CAP>, mut f: F) {
    let mut tmp = [0u8; 64];
    let mut n = 0;
    while n < tmp.len() {
        match consumer.dequeue() {
            Some(b) => {
                tmp[n] = b;
                n += 1;
            }
            None => break,
        }
    }
    if n > 0 {
        f(&tmp[..n]);
    }
}

/// Emergency drain used only by the panic handler, after it has set
/// `PANICKING` and waited long enough for core1 to observe it and park.
///
/// # Safety
/// Caller must guarantee core1 is no longer dequeuing from `LOG_QUEUE`
/// concurrently (i.e. it has already parked after observing `PANICKING`).
/// This bypasses the SPSC `Consumer` typestate, which is only sound because
/// at this point there is, in practice, a single reader left.
pub unsafe fn drain_from_panic<F: FnMut(&[u8]) -> usize>(mut f: F) {
    let queue = &mut *core::ptr::addr_of_mut!(LOG_QUEUE);
    let mut tmp = [0u8; 64];
    loop {
        let mut n = 0;
        while n < tmp.len() {
            match queue.dequeue() {
                Some(b) => { tmp[n] = b; n += 1; }
                None => break,
            }
        }
        if n == 0 { break; }
        let mut written = 0;
        while written < n {
            written += f(&tmp[written..n]);
        }
    }
}