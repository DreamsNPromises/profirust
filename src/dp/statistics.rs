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
        let cycles = self.cycles_completed.get();
        let data = self.data_exchanges.get();
        let ok_pct = if cycles > 0 {
            (data as f64 / cycles as f64) * 100.0
        } else {
            0.0
        };
        log::info!(
            "DP stats: ok={:.1}% cycles={} data={} retries={} timeouts={} diag={} offline={} crc_err={} len_err={}",
            ok_pct,
            cycles,
            data,
            self.retries.get(),
            self.timeouts.get(),
            self.diagnostics_events.get(),
            self.offline_events.get(),
            self.crc_errors.get(),
            self.length_mismatches.get(),
        );
    }
}