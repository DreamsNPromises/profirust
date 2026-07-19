/// Statistical counters for the DP master.
///
/// These counters track the number of successful and failed cycles, retransmissions,
/// timeouts, and other events that help assess link quality.
/// Enable via feature `"statistics"` in `profirust`.
#[derive(Debug, Clone)]
pub struct DpStatistics {
    // --- DP-level ---
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
    /// Number of times a peripheral went offline (max retries exceeded).
    pub offline_events: core::cell::Cell<u64>,
    /// Number of times a diagnostics request was made (separate from data exchange).,
    pub diagnostics_events: core::cell::Cell<u64>,

    // --- Link quality ---
    /// Number of CRC errors detected in received telegrams.
    pub crc_errors: core::cell::Cell<u64>,
    /// Number of length/structure errors (LE/LEr, too short, DSAP/SSAP missing, etc.).
    pub malformed_telegrams: core::cell::Cell<u64>,
    /// Number of telegrams with bad start/end delimiters (corrupted framing).
    pub framing_errors: core::cell::Cell<u64>,
    /// Number of telegrams with invalid function codes.
    pub invalid_fc_errors: core::cell::Cell<u64>,
    /// Number of unexpected telegrams received (reply from wrong address).
    pub unexpected_telegrams: core::cell::Cell<u64>,

    // --- FDL health ---
    /// Number of times the token was lost.
    pub token_losses: core::cell::Cell<u64>,
    /// Number of address collisions detected.
    pub collisions: core::cell::Cell<u64>,

    // --- PHY ---
    /// Number of times the receive buffer had to be dropped (overrun).
    pub rx_buffer_overruns: core::cell::Cell<u64>,
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
            offline_events: core::cell::Cell::new(0),
            diagnostics_events: core::cell::Cell::new(0),

            crc_errors: core::cell::Cell::new(0),
            malformed_telegrams: core::cell::Cell::new(0),
            framing_errors: core::cell::Cell::new(0),
            invalid_fc_errors: core::cell::Cell::new(0),
            unexpected_telegrams: core::cell::Cell::new(0),

            token_losses: core::cell::Cell::new(0),
            collisions: core::cell::Cell::new(0),

            rx_buffer_overruns: core::cell::Cell::new(0),
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
            "DP: cycles={} data_ok={:.1}% ({}/{}) retries={} timeouts={} offline={} diag={}",
            self.cycles_completed.get(),
            ok_pct,
            correct,
            data,
            self.retries.get(),
            self.timeouts.get(),
            self.offline_events.get(),
            self.diagnostics_events.get(),
            );
        log::info!(
            "Link: crc={} malformed={} framing={} bad_fc={} unexpected={} token_loss={} collisions={} rx_overrun={}",
            self.crc_errors.get(),
            self.malformed_telegrams.get(),
            self.framing_errors.get(),
            self.invalid_fc_errors.get(),
            self.unexpected_telegrams.get(),
            self.token_losses.get(),
            self.collisions.get(),
            self.rx_buffer_overruns.get(),
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

    pub fn inc_rx_overruns(&self) {
        self.rx_buffer_overruns.set(self.rx_buffer_overruns.get() + 1);
    }

    pub fn inc_offline(&self) {
        self.offline_events.set(self.offline_events.get() + 1);
    }

    pub fn inc_diagnostics(&self) {
        self.diagnostics_events.set(self.diagnostics_events.get() + 1);
    }

    /// Mark that the first data exchange has occurred.
    pub fn mark_first_data_exchanged(&self) {
        self.first_data_exchanged.set(true);
    }

    pub fn inc_crc_errors(&self) {
        self.crc_errors.set(self.crc_errors.get() + 1);
    }
    pub fn inc_malformed(&self) {
        self.malformed_telegrams.set(self.malformed_telegrams.get() + 1);
    }

    pub fn inc_framing_error(&self) {
        self.framing_errors.set(self.framing_errors.get() + 1);
    }

    pub fn inc_invalid_fc(&self) {
        self.invalid_fc_errors.set(self.invalid_fc_errors.get() + 1);
    }

    pub fn inc_unexpected(&self) {
        self.unexpected_telegrams.set(self.unexpected_telegrams.get() + 1);
    }

    pub fn inc_token_loss(&self) {
        self.token_losses.set(self.token_losses.get() + 1);
    }

    pub fn inc_collision(&self) {
        self.collisions.set(self.collisions.get() + 1);
    }
}