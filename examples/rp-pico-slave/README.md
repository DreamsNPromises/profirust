# profirust example: DP-V0 slave simulator on RP2040

This firmware turns a Raspberry Pi Pico into a **DP-V0 slave station**: together with the
master example ([`examples/rp-pico`](../rp-pico/)) it forms the smallest possible test bench -
one master, one simulated slave, one LED and (later) one button.

It implements step 3 of [`docs/PLAN-dp-slave-simulator-final.md`](../../docs/PLAN-dp-slave-simulator-final.md):
the firmware drives a single `profirust::dp::DpSlave` from the main loop and answers the
master's `Slave_Diag`, `Set_Prm`, `Chk_Cfg` and cyclic data exchange requests.  LED/button on
the process image (step 4), several slaves per device (step 5) and speeds above 500 kbit/s
(steps 6/7) are not implemented yet.

## Wiring

| Pico pin | Function |
|----------|----------|
| GPIO0 | UART0 TX -> RS-485 transceiver DI |
| GPIO1 | UART0 RX <- RS-485 transceiver RO |
| GPIO2 | RS-485 direction (DE/RE), high while transmitting |
| GND | common ground with the master |
| GPIO25 (on-board LED) | heartbeat: blinks with the simulated input bit |

As with the master example, the RS-485 driver enable is handled by the firmware; the same
transceiver circuit can be used on both ends.  Don't forget bus termination (and a pull-up on
the RX line if your transceiver needs one).

## Test-bench profile

| Parameter | Value |
|-----------|-------|
| Station address | 3 |
| Ident number | `0x00F0` (`PXC_00F0.gsd`, Phoenix Contact IL PB BK DP/V1) |
| `User_Prm_Data` | `[0x20]` |
| `Chk_Cfg` | `[0x42, 0x00, 0xBE, 0x81, 0x82, 0x00, 0xBD, 0x81]` |
| Inputs (`pi_i`, slave -> master) | 1 byte (module "IB IL 24 DI 8") |
| Outputs (`pi_q`, master -> slave) | 1 byte (module "IB IL 24 DO 8") |
| Baudrate | `B9600` (compile-time constant) |
| Watchdog | armed by the master's `Set_Prm`, fail-safe clears the outputs |

The profile is defined by the constants at the top of [`src/main.rs`](src/main.rs).  It must
match the master's `PeripheralOptions` - otherwise the slave rejects `Set_Prm`/`Chk_Cfg` and the
master reports `ParameterError`/`ConfigError`.

### Matching master configuration

Any DP master can talk to this slave as long as its `PeripheralOptions` match the profile
above.  When the master's options are generated from `PXC_00F0.gsd`, this means selecting
exactly the two modules that make up the profile - **DI8 first, DO8 second** (the order
determines the byte order in `Chk_Cfg`):

```text
"IB IL 24 DI 8"   -- 1 byte inputs  -> [42 00 BE 81]
"IB IL 24 DO 8"   -- 1 byte outputs -> [82 00 BD 81]
```

Any other module selection produces different `Chk_Cfg` bytes and/or different process image
lengths, so the slave rejects the configuration and the master reports
`ParameterError`/`ConfigError`.

## Build & flash

```bash
./build.sh              # build + convert to dp-slave-pico.uf2 + copy to the bootloader volume
```

or manually:

```bash
cargo build --release
elf2uf2-rs target/thumbv6m-none-eabi/release/dp-slave-pico dp-slave-pico.uf2
```

The log goes over USB CDC (core1), so the device shows up as `/dev/ttyACM*`:

```bash
picocom -q /dev/ttyACM1
```

## What to expect

Log of the slave:

```terminal
[    0.000000] dp_slave_pico::main: Booting PROFIBUS DP-V0 slave simulator...
[    0.000000] dp_slave_pico::main: System clock: 240000000 Hz
[    0.000000] dp_slave_pico::main: DP-V0 slave #3: ident 0x00F0, B9600, pi_i 1 B, pi_q 1 B
[    1.234567] dp_slave_pico::main: DP slave #3 is now Wcfg
[    1.241234] dp_slave_pico::main: DP slave #3 is now Dxchg
```

Log of the master:

```terminal
profirust::dp::peripheral: Peripheral #3 becomes ready for data exchange.
dp_master_pico: Inputs RAW: 0x01
dp_master_pico: Inputs RAW: 0x02
```

While the master is exchanging data, the on-board LED blinks at 1 Hz and the input byte
increments once per second.  The output byte written by the master is stored in `pi_q` but not
used yet (that is step 4).

## Timing

Before answering, the slave waits for the minimum T<sub>SDR</sub> (at least 11 bit times) that
the master requested in `Set_Prm`.  `Rp2040Phy::transmit_data()` additionally delays the first
byte by T<sub>SET</sub>, so the busy-wait in `wait_min_tsdr()` only covers the difference (see
section 6 of the plan).  At 9.6 kbit/s this is about 1 ms per response.

## Notes

- Nothing is logged from the PROFIBUS hot path on core0; all log records go through an SPSC
  queue to core1 (USB CDC), same as in the master example.
- `profirust` is used with `default-features = false, features = ["phy-rp2040"]`; the slave
  implementation is `no_std`-clean and needs no allocator.
