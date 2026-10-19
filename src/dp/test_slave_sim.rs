//! Host integration tests: `DpMaster` <-> N x `DpSlave` on a simulated bus.
//!
//! This module implements step 2 of the DP-slave-simulator plan
//! (`docs/PLAN-dp-slave-simulator-final.md`): the master side of profirust talks to a number of
//! virtual DP-V0 slaves ([`DpSlave`]) over a single simulated bus ([`SimulatorPhy`]).  The
//! scenarios are the ones from appendix B of the plan (`b1_*` ... `b8_*`).
//!
//! The harness mirrors the firmware main-loop from section 3.2 of the plan:
//!
//! - Three duplicates of one [`SimulatorPhy`]: `phy_ctrl` (owns the bus time), `phy_master`
//!   (driven by [`FdlActiveStation`] + [`DpMaster`]) and a single `phy_slave` which drives *all*
//!   [`DpSlave`]s through [`dispatch()`] - just like the firmware drives all of its virtual
//!   slaves through one UART.
//! - The slave side only ever transmits a reply (after the minimum T<sub>SDR</sub>) and never
//!   initiates anything on its own.
//! - One simulation timestep is one byte-time: telegrams appear on the bus byte by byte and every
//!   byte is processed exactly once, so this is equivalent to a finer timestep while keeping the
//!   long-running soak scenario (b8) fast.
//!
//! The bus protocol itself is exercised end-to-end: token-ring management and gap (live-list)
//! polling of the FDL layer, parameterization/configuration of the DP layer, FCB de-duplication,
//! diagnostics and the watchdog.

use crate::dp::{
    self, DpSlave, ExtDiagBlock, Peripheral, PeripheralEvent, PeripheralHandle, SlaveConfig,
    SlaveState,
};
use crate::fdl;
use crate::phy::simulator::SimulatorPhy;
use crate::phy::ProfibusPhy;
use crate::time::{Duration, Instant};
use crate::Baudrate;

// ---------------------------------------------------------------------------
// Testbench profile (see section 7 of the plan)
// ---------------------------------------------------------------------------
const MASTER_ADDRESS: u8 = 2;
const SLAVE_ADDRESSES: [u8; 4] = [3, 4, 5, 6];
const IDENT_NUMBER: u16 = 0x00F0;
const USER_PRM: &[u8] = &[0x20];
const CONFIG: &[u8] = &[0x42, 0x00, 0xBE, 0x81, 0x82, 0x00, 0xBD, 0x81];

/// Watchdog period which the master requests from the slaves (50 ms -> WD_Fact_1/2 = 1/5).
const WD_TIMEOUT: Duration = Duration::from_millis(50);

/// `max_tsdr` for the peripherals (must fit below the default slot time).
const MAX_TSDR: u16 = 15;

fn slave_config(address: u8) -> SlaveConfig<'static> {
    SlaveConfig {
        address,
        ident_number: IDENT_NUMBER,
        user_parameters: Some(USER_PRM),
        config: Some(CONFIG),
        fail_safe: true,
    }
}

fn make_slave(address: u8) -> DpSlave<'static> {
    // Owned buffers so that slaves can be (re-)created at any time (see `Sim::reboot_slave()`).
    DpSlave::new(slave_config(address), vec![0u8; 1], vec![0u8; 1])
}

/// Configuration of one simulated testbench.
///
/// The master-side [`PeripheralOptions`] can be deviated from the slave-side [`SlaveConfig`] to
/// provoke parameter/configuration faults (scenario b5).
#[derive(Debug, Clone, Copy)]
struct SimParams {
    baudrate: Baudrate,
    n_slaves: usize,
    /// Ident number sent by the master during `Set_Prm` (mismatch -> `prm_fault`).
    ident_number: u16,
    /// User parameters sent by the master during `Set_Prm` (mismatch -> `prm_fault`).
    user_parameters: &'static [u8],
    /// Configuration sent by the master during `Chk_Cfg` (mismatch -> `cfg_fault`).
    config: &'static [u8],
}

impl SimParams {
    fn with_slaves(n_slaves: usize) -> Self {
        assert!(n_slaves >= 1 && n_slaves <= SLAVE_ADDRESSES.len());
        Self {
            n_slaves,
            ..Default::default()
        }
    }
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            baudrate: Baudrate::B500000,
            n_slaves: 2,
            ident_number: IDENT_NUMBER,
            user_parameters: USER_PRM,
            config: CONFIG,
        }
    }
}

/// The simulated testbench: one DP master and `n` DP slaves on a shared bus.
struct Sim {
    baudrate: Baudrate,
    timestep: Duration,

    phy_ctrl: SimulatorPhy,
    phy_master: SimulatorPhy,
    phy_slave: SimulatorPhy,

    fdl_master: fdl::FdlActiveStation,
    dp_master: dp::DpMaster<'static>,
    /// Address/handle mapping for the peripherals on the master side.
    handles: Vec<(u8, PeripheralHandle)>,

    slaves: Vec<DpSlave<'static>>,
    tx_buf: [u8; dp::MAX_REPLY_LEN],

    /// All peripheral events seen so far, in order.
    events: Vec<(u8, PeripheralEvent)>,
    /// Number of completed DP cycles (all peripherals polled).
    cycles: u64,
}

impl Sim {
    fn new(params: SimParams) -> Self {
        let phy_ctrl = SimulatorPhy::new(params.baudrate, "ctrl");
        let phy_master = phy_ctrl.duplicate("master");
        let phy_slave = phy_ctrl.duplicate("slaves");

        let mut dp_master = dp::DpMaster::new(vec![]);
        let mut handles = Vec::new();
        for &address in &SLAVE_ADDRESSES[..params.n_slaves] {
            let options = dp::PeripheralOptions {
                ident_number: params.ident_number,
                user_parameters: Some(params.user_parameters),
                config: Some(params.config),
                max_tsdr: MAX_TSDR,
                ..Default::default()
            };
            let handle = dp_master.add(
                dp::Peripheral::new(address, options, vec![0u8; 1], vec![0u8; 1])
                    .with_diag_buffer(vec![0u8; 32]),
            );
            handles.push((address, handle));
        }

        // Section 3.2/appendix B: default `slot_bits`, master watchdog of 50-100 ms.
        let fdl_master = fdl::FdlActiveStation::new(
            fdl::ParametersBuilder::new(MASTER_ADDRESS, params.baudrate)
                .highest_station_address(SLAVE_ADDRESSES[params.n_slaves - 1] + 1)
                .watchdog_timeout(WD_TIMEOUT)
                .build_verified(&dp_master),
        );

        let slaves = SLAVE_ADDRESSES[..params.n_slaves]
            .iter()
            .map(|&address| make_slave(address))
            .collect();

        Self {
            // One byte time - telegrams become visible on the bus byte by byte.
            timestep: params.baudrate.bits_to_time(11),
            baudrate: params.baudrate,
            phy_ctrl,
            phy_master,
            phy_slave,
            fdl_master,
            dp_master,
            handles,
            slaves,
            tx_buf: [0; dp::MAX_REPLY_LEN],
            events: Vec::new(),
            cycles: 0,
        }
    }

    /// Bring the master online and start cyclic data exchange.
    fn start(&mut self) {
        self.fdl_master.set_online();
        self.dp_master.enter_operate();
    }

    fn now(&self) -> Instant {
        self.phy_ctrl.bus_time()
    }

    /// Advance bus time by one timestep and run one iteration of the firmware main-loop (section
    /// 3.2 of the plan).
    fn step(&mut self) {
        let now = self.phy_ctrl.advance_bus_time(self.timestep);
        crate::test_utils::set_log_timestamp(now);

        self.poll_master(now);
        self.poll_slaves(now);
    }

    fn poll_master(&mut self, now: Instant) {
        // While the master PHY is still transmitting, the FDL layer must be left alone.
        if self.phy_master.poll_transmission(now) {
            return;
        }

        self.fdl_master
            .poll(now, &mut self.phy_master, &mut self.dp_master);

        let events = self.dp_master.take_last_events();
        if events.cycle_completed {
            self.cycles += 1;
        }
        if let Some((handle, event)) = events.peripheral {
            let address = self
                .handles
                .iter()
                .find(|(_, h)| *h == handle)
                .map(|(address, _)| *address)
                .expect("event for unknown peripheral handle");
            self.events.push((address, event));
        }
    }

    fn poll_slaves(&mut self, now: Instant) {
        // The telegram appears on the bus byte by byte - only once it is complete does
        // `receive_telegram()` call the closure and only then may the slave answer.
        if !self.phy_slave.poll_transmission(now) {
            let reply = self
                .phy_slave
                .receive_telegram(now, |res| match res {
                    Ok(t) => dp::dispatch(&mut self.slaves, now, &t, &mut self.tx_buf),
                    Err(_) => 0,
                })
                .unwrap_or(0);

            if reply > 0 {
                // Wait for the minimum TSDR before answering (>= 11 bit times).
                let reply_time = self.phy_ctrl.advance_bus_time(self.reply_delay());
                self.phy_slave.transmit_data(reply_time, |buf| {
                    buf[..reply].copy_from_slice(&self.tx_buf[..reply]);
                    (reply, ())
                });
            }
        }

        for slave in self.slaves.iter_mut() {
            slave.tick(now);
        }
    }

    /// Minimum response delay (T<sub>SDR</sub>) of the slowest slave, at least 11 bit times.
    fn reply_delay(&self) -> Duration {
        let bits = self
            .slaves
            .iter()
            .map(|s| s.min_tsdr_bits())
            .max()
            .unwrap_or(11)
            .max(11);
        self.baudrate.bits_to_time(u32::from(bits))
    }

    fn run_for(&mut self, duration: Duration) {
        let end = self.now() + duration;
        while self.now() < end {
            self.step();
        }
    }

    /// Run the simulation (with the master) until `f` returns `true`.
    ///
    /// Panics when the condition is not met within `timeout` of simulated time.
    fn run_until<F: FnMut(&mut Self) -> bool>(&mut self, timeout: Duration, mut f: F) {
        let start = self.now();
        while !f(self) {
            self.step();
            if self.now() - start > timeout {
                panic!(
                    "Simulation timed out after {} us!",
                    (self.now() - start).total_micros()
                );
            }
        }
    }

    /// Keep the bus running (slaves still tick and would still answer) but stop polling the
    /// master - this is how "the master went silent" is simulated (scenario b7).
    fn run_for_master_paused(&mut self, duration: Duration) {
        let end = self.now() + duration;
        while self.now() < end {
            let now = self.phy_ctrl.advance_bus_time(self.timestep);
            crate::test_utils::set_log_timestamp(now);
            self.poll_slaves(now);
        }
    }

    /// Simulate a power-cycle of one slave: its state resets to `Wprm` (scenario b6).
    fn reboot_slave(&mut self, index: usize) {
        let address = self.slaves[index].address();
        self.slaves[index] = make_slave(address);
    }

    // --- accessors -------------------------------------------------------

    fn slave_index(&self, address: u8) -> usize {
        self.slaves
            .iter()
            .position(|s| s.address() == address)
            .expect("no slave with this address")
    }

    fn slave(&self, address: u8) -> &DpSlave<'static> {
        &self.slaves[self.slave_index(address)]
    }

    fn slave_mut(&mut self, address: u8) -> &mut DpSlave<'static> {
        let index = self.slave_index(address);
        &mut self.slaves[index]
    }

    fn peripheral(&self, address: u8) -> &Peripheral<'static> {
        let handle = self.handle_of(address);
        self.dp_master
            .iter()
            .find(|(h, _)| *h == handle)
            .map(|(_, p)| p)
            .expect("no peripheral with this address")
    }

    fn peripheral_mut(&mut self, address: u8) -> &mut Peripheral<'static> {
        let handle = self.handle_of(address);
        self.dp_master.get_mut(handle)
    }

    fn handle_of(&self, address: u8) -> PeripheralHandle {
        self.handles
            .iter()
            .find(|(a, _)| *a == address)
            .map(|(_, h)| *h)
            .expect("no peripheral with this address")
    }

    /// Set the output process image of a peripheral (master -> slave).
    fn set_outputs(&mut self, address: u8, value: u8) {
        self.peripheral_mut(address).pi_q_mut()[0] = value;
    }

    /// Set the input process image of a slave (slave -> master).
    fn set_inputs(&mut self, address: u8, value: u8) {
        self.slave_mut(address).pi_i_mut()[0] = value;
    }

    fn slave_outputs(&self, address: u8) -> u8 {
        self.slave(address).pi_q()[0]
    }

    fn master_inputs(&self, address: u8) -> u8 {
        self.peripheral(address).pi_i()[0]
    }

    fn events_for(&self, address: u8) -> Vec<PeripheralEvent> {
        self.events
            .iter()
            .filter(|(a, _)| *a == address)
            .map(|(_, e)| *e)
            .collect()
    }

    fn event_count(&self, address: u8, event: PeripheralEvent) -> usize {
        self.events_for(address)
            .iter()
            .filter(|e| **e == event)
            .count()
    }
}

/// Wait for the full startup sequence: `Online` -> `Configured` -> cyclic `DataExchanged`.
fn wait_until_running(sim: &mut Sim, address: u8) {
    sim.run_until(Duration::from_secs(60), |s| {
        s.peripheral(address).is_running() && s.cycles >= 3
    });

    let events = sim.events_for(address);
    assert_eq!(events[0], PeripheralEvent::Online, "startup event #1");
    assert_eq!(events[1], PeripheralEvent::Configured, "startup event #2");
    assert!(events.len() > 2, "no data exchange events yet");
    assert!(
        events[2..]
            .iter()
            .all(|e| *e == PeripheralEvent::DataExchanged),
        "unexpected events after startup: {events:?}"
    );
}

// ---------------------------------------------------------------------------
// Scenario B.1: one slave, full startup
// ---------------------------------------------------------------------------
#[test]
fn b1_single_slave_startup() {
    crate::test_utils::prepare_test_logger();

    // Exercise both a slow and a fast baudrate (the timing math in the harness depends on it).
    for baudrate in [Baudrate::B9600, Baudrate::B500000] {
        let mut sim = Sim::new(SimParams {
            baudrate,
            n_slaves: 1,
            ..Default::default()
        });
        sim.start();

        wait_until_running(&mut sim, SLAVE_ADDRESSES[0]);
        assert_eq!(sim.slave(SLAVE_ADDRESSES[0]).state(), SlaveState::Dxchg);
    }
}

// ---------------------------------------------------------------------------
// Scenario B.2: process-image echo
// ---------------------------------------------------------------------------
#[test]
fn b2_process_image_echo() {
    crate::test_utils::prepare_test_logger();

    let address = SLAVE_ADDRESSES[0];
    let mut sim = Sim::new(SimParams::with_slaves(1));
    sim.start();
    wait_until_running(&mut sim, address);

    for value in [0xA5u8, 0x5A, 0x01, 0x00, 0xFF] {
        // Outputs: master -> slave, inputs: slave -> master.
        sim.set_outputs(address, value);
        sim.set_inputs(address, !value);

        sim.run_until(Duration::from_secs(60), |s| {
            s.slave_outputs(address) == value && s.master_inputs(address) == !value
        });
    }
}

// ---------------------------------------------------------------------------
// Scenario B.3: four slaves with different addresses
// ---------------------------------------------------------------------------
#[test]
fn b3_four_slaves() {
    crate::test_utils::prepare_test_logger();

    let mut sim = Sim::new(SimParams::with_slaves(4));
    sim.start();

    // All four slaves must reach data exchange ("tokons/foreign telegrams don't disturb us" -
    // the master also polls the gap (live-list) and sends global-control broadcasts in between).
    for &address in &SLAVE_ADDRESSES {
        wait_until_running(&mut sim, address);
    }

    // Per-slave process images are independent of each other.
    for (i, &address) in SLAVE_ADDRESSES.iter().enumerate() {
        sim.set_outputs(address, 0x10 + i as u8);
        sim.set_inputs(address, 0x80 + i as u8);
    }
    sim.run_until(Duration::from_secs(60), |s| {
        SLAVE_ADDRESSES.iter().enumerate().all(|(i, &address)| {
            s.slave_outputs(address) == 0x10 + i as u8 && s.master_inputs(address) == 0x80 + i as u8
        })
    });

    assert!(sim.cycles >= 3);
}

// ---------------------------------------------------------------------------
// Scenario B.4: extended diagnostics
// ---------------------------------------------------------------------------
#[test]
fn b4_extended_diagnostics() {
    crate::test_utils::prepare_test_logger();

    let address = SLAVE_ADDRESSES[0];
    let mut sim = Sim::new(SimParams::with_slaves(1));
    sim.start();
    wait_until_running(&mut sim, address);

    // Identifier-based diagnostics block: "problem with module 0".
    sim.slave_mut(address).set_ext_diag(&[0x42, 0x01]);

    // The next data-exchange reply is "DH" which makes the master fetch the diagnostics.
    sim.run_until(Duration::from_secs(60), |s| {
        s.event_count(address, PeripheralEvent::Diagnostics) >= 1
    });

    let diag = sim
        .peripheral(address)
        .last_diagnostics()
        .expect("no diagnostics recorded");
    assert!(diag.flags.contains(dp::DiagnosticFlags::EXT_DIAG));
    assert!(diag.flags.contains(dp::DiagnosticFlags::STATUS_DIAGNOSTICS));

    let blocks: Vec<ExtDiagBlock> = diag.extended_diagnostics.iter_diag_blocks().collect();
    assert_eq!(blocks.len(), 1);
    match &blocks[0] {
        ExtDiagBlock::Identifier(bits) => {
            assert_eq!(bits.count_ones(), 1);
            assert!(bits[0], "expected a problem for module 0");
        }
        b => panic!("unexpected diagnostics block: {b:?}"),
    }
}

// ---------------------------------------------------------------------------
// Scenario B.5: parameter fault and configuration fault
// ---------------------------------------------------------------------------
#[test]
fn b5_parameter_fault() {
    crate::test_utils::prepare_test_logger_with_warnings(vec![
        "Peripheral #3 reports a parameter fault!",
    ]);

    // The master sends a different ident number than the slave expects.
    let address = SLAVE_ADDRESSES[0];
    let mut sim = Sim::new(SimParams {
        n_slaves: 1,
        ident_number: IDENT_NUMBER ^ 1,
        ..Default::default()
    });
    sim.start();

    sim.run_until(Duration::from_secs(60), |s| {
        s.event_count(address, PeripheralEvent::ParameterError) >= 1
    });

    // The slave never accepted the parameterization.
    assert_eq!(sim.slave(address).state(), SlaveState::Wprm);
    assert_eq!(
        sim.events_for(address)[..2],
        [PeripheralEvent::Online, PeripheralEvent::ParameterError,]
    );
}

#[test]
fn b5_configuration_fault() {
    crate::test_utils::prepare_test_logger_with_warnings(vec![
        "Peripheral #3 reports a configuration fault!",
    ]);

    // The master sends a different configuration than the slave expects.
    let address = SLAVE_ADDRESSES[0];
    let mut sim = Sim::new(SimParams {
        n_slaves: 1,
        config: &[0x42, 0x00, 0xBE, 0x41, 0x82, 0x00, 0xBD, 0x41],
        ..Default::default()
    });
    sim.start();

    sim.run_until(Duration::from_secs(60), |s| {
        s.event_count(address, PeripheralEvent::ConfigError) >= 1
    });

    assert_eq!(sim.slave(address).state(), SlaveState::Wcfg);
    assert_eq!(
        sim.events_for(address)[..2],
        [PeripheralEvent::Online, PeripheralEvent::ConfigError,]
    );
}

// ---------------------------------------------------------------------------
// Scenario B.6: "RS" on data-exchange (slave was reset to Wprm) -> recovery
// ---------------------------------------------------------------------------
#[test]
fn b6_rs_on_data_exchange_recovery() {
    crate::test_utils::prepare_test_logger_with_warnings(vec![
        "Got \"SAP not enabled\" response from #3, revalidating config...",
        "Peripheral #3 wants parameters after completing setup?! Retrying...",
    ]);

    let address = SLAVE_ADDRESSES[0];
    let mut sim = Sim::new(SimParams::with_slaves(1));
    sim.start();
    wait_until_running(&mut sim, address);

    // Power-cycle the slave: it is back in `Wprm` while the master is still in data exchange.
    // The slave answers "RS" (SAP not enabled) which makes the master revalidate the config and
    // re-parameterize the slave.
    sim.reboot_slave(0);
    assert_eq!(sim.slave(address).state(), SlaveState::Wprm);

    sim.run_until(Duration::from_secs(60), |s| {
        s.peripheral(address).is_running()
            && s.event_count(address, PeripheralEvent::Configured) >= 2
    });
    assert_eq!(sim.slave(address).state(), SlaveState::Dxchg);

    // Data exchange works again after the recovery.
    sim.set_outputs(address, 0xC3);
    sim.set_inputs(address, 0x3C);
    sim.run_until(Duration::from_secs(60), |s| {
        s.slave_outputs(address) == 0xC3 && s.master_inputs(address) == 0x3C
    });
}

// ---------------------------------------------------------------------------
// Scenario B.7: watchdog (master goes silent for longer than the watchdog period)
// ---------------------------------------------------------------------------
#[test]
fn b7_watchdog_recovery() {
    crate::test_utils::prepare_test_logger_with_warnings(vec![
        "Resending a telegram to #3...",
        "Got \"SAP not enabled\" response from #3, revalidating config...",
        "Peripheral #3 wants parameters after completing setup?! Retrying...",
    ]);

    let address = SLAVE_ADDRESSES[0];
    let mut sim = Sim::new(SimParams::with_slaves(1));
    sim.start();
    wait_until_running(&mut sim, address);

    sim.set_outputs(address, 0xA5);
    sim.run_until(Duration::from_secs(60), |s| {
        s.slave_outputs(address) == 0xA5
    });

    // The master goes silent for more than the watchdog period (50 ms).
    sim.run_for_master_paused(WD_TIMEOUT * 2);
    assert_eq!(sim.slave(address).state(), SlaveState::Wprm);
    // Fail-safe: the outputs were cleared.
    assert_eq!(sim.slave_outputs(address), 0);

    // Once the master is back, the slave is re-parameterized and reaches data exchange again.
    sim.run_until(Duration::from_secs(60), |s| {
        s.peripheral(address).is_running()
            && s.event_count(address, PeripheralEvent::Configured) >= 2
    });
    assert_eq!(sim.slave(address).state(), SlaveState::Dxchg);

    sim.set_outputs(address, 0x5A);
    sim.run_until(Duration::from_secs(60), |s| {
        s.slave_outputs(address) == 0x5A
    });
}

// ---------------------------------------------------------------------------
// Scenario B.8: soak - many cycles without retries/timeouts/CRC errors
// ---------------------------------------------------------------------------
#[test]
fn b8_soak() {
    crate::test_utils::prepare_test_logger_with_warnings(vec![]);
    // Long-running: suppress trace/debug output but keep the warnings check active.
    crate::test_utils::set_log_quiet(true);

    const SOAK_CYCLES: u64 = 10_000;

    let mut sim = Sim::new(SimParams::with_slaves(4));
    sim.start();
    for &address in &SLAVE_ADDRESSES {
        wait_until_running(&mut sim, address);
    }
    let startup_events = sim.events.len();

    let target = sim.cycles + SOAK_CYCLES;
    sim.run_until(Duration::from_secs(3600), |s| s.cycles >= target);

    // All slaves are still in data exchange ...
    for &address in &SLAVE_ADDRESSES {
        assert_eq!(sim.slave(address).state(), SlaveState::Dxchg);
        assert!(sim.peripheral(address).is_running());
    }

    // ... and nothing unexpected happened on the way here.
    assert!(
        sim.events[startup_events..]
            .iter()
            .all(|(_, e)| *e == PeripheralEvent::DataExchanged),
        "unexpected events during soak run"
    );

    #[cfg(feature = "statistics")]
    {
        let stats = sim.dp_master.statistics();
        assert!(stats.cycles_completed.get() >= SOAK_CYCLES);
        assert!(stats.data_exchanges.get() >= 4 * SOAK_CYCLES);
        assert_eq!(stats.retries.get(), 0, "retries");
        assert_eq!(stats.timeouts.get(), 0, "timeouts");
        assert_eq!(stats.offline_events.get(), 0, "offline events");
        assert_eq!(stats.crc_errors.get(), 0, "crc errors");
        assert_eq!(stats.malformed_telegrams.get(), 0, "malformed telegrams");
        assert_eq!(stats.framing_errors.get(), 0, "framing errors");
        assert_eq!(stats.invalid_fc_errors.get(), 0, "invalid function codes");
        assert_eq!(stats.unexpected_telegrams.get(), 0, "unexpected telegrams");
    }
}
