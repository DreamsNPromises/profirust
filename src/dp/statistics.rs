/// Statistical counters for the DP master.
///
/// These counters track the number of successful and failed cycles, retransmissions,
/// timeouts, and other events that help assess link quality.
/// Enable via feature `"statistics"` in `profirust`.
#[derive(Debug, Clone)]
pub struct DpStatistics {
    /// Number of completed polling cycles (all peripherals processed).
    pub cycles_completed: core::cell::Cell<u64>,
    /// Number of successful data exchange cycles (received valid PDU).
    pub data_exchanges: core::cell::Cell<u64>,
    /// Set to true when first successful data exchange occurs.
    first_data_exchanged: core::cell::Cell<bool>,
    /// Number of correct data exchange cycles (received expected PDU).
    pub correct_data_received: core::cell::Cell<u64>,
    /// When set, each successful data exchange telegram that matches this
    /// pattern will increment `correct_data_received`.
    pub expected_data: core::cell::Cell<Option<&'static [u8]>>,
    /// Number of request retries (first retry for any reason).
    pub retries: core::cell::Cell<u64>,
    /// Number of timeouts waiting for a reply.
    pub timeouts: core::cell::Cell<u64>,
    /// Number of CRC errors detected in received telegrams.
    pub crc_errors: core::cell::Cell<u64>,
    /// Number of times the receive buffer had to be dropped (overrun).
    pub rx_buffer_overruns: core::cell::Cell<u64>,
    /// Number of times a peripheral went offline (max retries exceeded).
    pub offline_events: core::cell::Cell<u64>,
    /// Number of length info mismatches (LE != LEr).
    pub length_mismatches: core::cell::Cell<u64>,
    /// Number of times a diagnostics request was made (separate from data exchange).,
    pub diagnostics_events: core::cell::Cell<u64>,
}

impl Default for DpStatistics {
    fn default() -> Self {
        Self {
            cycles_completed: core::cell::Cell::new(0),
            data_exchanges: core::cell::Cell::new(0),
            first_data_exchanged: core::cell::Cell::new(false),
            correct_data_received: core::cell::Cell::new(0),
            // TODO: maybe specify elsewhere
            // like dp_master.statistics().expected_data = Some(&[0x03, 0x0C]);
            expected_data: core::cell::Cell::new(Some(&[0x03, 0x0C])),
            retries: core::cell::Cell::new(0),
            timeouts: core::cell::Cell::new(0),
            crc_errors: core::cell::Cell::new(0),
            rx_buffer_overruns: core::cell::Cell::new(0),
            offline_events: core::cell::Cell::new(0),
            length_mismatches: core::cell::Cell::new(0),
            diagnostics_events: core::cell::Cell::new(0),
        }
    }
}

impl DpStatistics {
    /// Log the current statistics at `info` level.
    pub fn log_summary(&self) {
        let data = self.data_exchanges.get();
        let correct = self.correct_data_received.get();
        let ok_pct = if data > 0 {
            (correct as f64 / data as f64) * 100.0
        } else {
            0.0
        };
        log::info!(
            "DP stats: cycles={} ok={:.1}% (data={} correct={}) retries={} timeouts={} diag={} offline={} crc_err={} len_err={}",
            self.cycles_completed.get(),
            ok_pct,
            data,
            correct,
            self.retries.get(),
            self.timeouts.get(),
            self.diagnostics_events.get(),
            self.offline_events.get(),
            // CRC check is performed inside `DataTelegram::deserialize`
            // but there is currently no way to
            // report failures to the statistics. To enable this counter,
            // either plumb a `&DpStatistics` reference down to
            // `deserialize`, or use a global/thread‑local counter.
            self.crc_errors.get(),
            self.length_mismatches.get(),
        );
    }

    // --- Increment helpers (with conditional logic where needed) ---

    /// Increment `cycles_completed`, but only after the first data exchange.
    pub fn inc_cycles(&self) {
        if self.first_data_exchanged.get() {
            self.cycles_completed.set(self.cycles_completed.get() + 1);
        }
    }

    pub fn inc_data_exchanges(&self) {
        self.data_exchanges.set(self.data_exchanges.get() + 1);
    }

    pub fn inc_correct(&self) {
        self.correct_data_received.set(self.correct_data_received.get() + 1);
    }

    pub fn inc_retries(&self) {
        self.retries.set(self.retries.get() + 1);
    }

    pub fn inc_timeouts(&self) {
        self.timeouts.set(self.timeouts.get() + 1);
    }

    pub fn inc_crc_errors(&self) {
        self.crc_errors.set(self.crc_errors.get() + 1);
    }

    pub fn inc_rx_overruns(&self) {
        self.rx_buffer_overruns.set(self.rx_buffer_overruns.get() + 1);
    }

    pub fn inc_offline(&self) {
        self.offline_events.set(self.offline_events.get() + 1);
    }

    pub fn inc_length_mismatch(&self) {
        self.length_mismatches.set(self.length_mismatches.get() + 1);
    }

    pub fn inc_diagnostics(&self) {
        self.diagnostics_events.set(self.diagnostics_events.get() + 1);
    }

    /// Mark that the first data exchange has occurred.
    pub fn mark_first_data_exchanged(&self) {
        self.first_data_exchanged.set(true);
    }
}