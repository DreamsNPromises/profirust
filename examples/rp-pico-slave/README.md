# profirust example: DP-V0 slave simulator on RP2040

This firmware turns a Raspberry Pi Pico into a **DP-V0 slave station**: together with the
master example ([`examples/rp-pico`](../rp-pico/)) it forms the smallest possible test bench -
one master, one simulated slave, one LED and (later) one button.

It implements step 3 of [`docs/PLAN-dp-slave-simulator-final.md`](../../docs/PLAN-dp-slave-simulator-final.md)
and the LED part of step 4: the firmware drives a single `profirust::dp::DpSlave` from the main
loop and answers the master's `Slave_Diag`, `Set_Prm`, `Chk_Cfg` and cyclic data exchange
requests.  The button on the process image (rest of step 4), several slaves per device (step 5)
and speeds above 500 kbit/s (steps 6/7) are not implemented yet.

## Wiring

| Pico pin | Function |
|----------|----------|
| GPIO0 | UART0 TX -> RS-485 transceiver DI |
| GPIO1 | UART0 RX <- RS-485 transceiver RO |
| GPIO2 | RS-485 direction (DE/RE), high while transmitting |
| GPIO6 | external LED -> output process image, bit 0 of `pi_q` (active high) |
| GND | common ground with the master |
| GPIO25 (on-board LED) | heartbeat: blinks with the simulated input bit |

External LED wiring (any 3.3 V LED works):

```text
GPIO6 ---[ 220..330 Ohm ]---|>|--- GND
                            LED
```

A written output bit `0` turns the LED off, `1` turns it on.  If the master stops exchanging
data, the slave watchdog clears `pi_q` (fail-safe), so the LED goes dark on its own.

As with the master example, the RS-485 driver enable is handled by the firmware; the same
transceiver circuit can be used on both ends.  Don't forget bus termination (and a pull-up on
the RX line if your transceiver needs one).

## Test-bench profile

| Parameter | Value |
|-----------|-------|
| Station address | 3 |
| Ident number | `0x0D0F` (GSD [`gsd-files/PRF_0D0F.gsd`](../../gsd-files/PRF_0D0F.gsd)) |
| `User_Prm_Data` | `[0x20]` |
| `Chk_Cfg` | `[0x42, 0x00, 0xBE, 0x81, 0x82, 0x00, 0xBD, 0x81]` |
| Inputs (`pi_i`, slave -> master) | 1 byte |
| Outputs (`pi_q`, master -> slave) | 1 byte |
| Baudrate | `B9600` (compile-time constant) |
| Watchdog | armed by the master's `Set_Prm`, fail-safe clears the outputs |

The profile is defined by the constants at the top of [`src/main.rs`](src/main.rs) and is
described in machine-readable form by the GSD file
[`gsd-files/PRF_0D0F.gsd`](../../gsd-files/PRF_0D0F.gsd).  Both must stay in sync: if the slave
and the master disagree, the slave rejects `Set_Prm`/`Chk_Cfg` and the master reports
`ParameterError`/`ConfigError`.

### Matching master configuration

Any DP master can talk to this slave as long as its configuration matches the profile above.
There are two ways to set up the master:

1. **Import the GSD** (`gsd-files/PRF_0D0F.gsd`) into the master's configuration tool and add
   the station "DP-V0 Slave Simulator (RP2040)".  The file declares a single compact module
   with exactly the `Chk_Cfg` bytes and the process image above, so there is nothing else to
   choose.
2. **Let `gsdtool` generate the `PeripheralOptions`** for a profirust master
   (`examples/rp-pico`):

   ```bash
   cargo run -p gsdtool -- config-wizard gsd-files/PRF_0D0F.gsd
   ```

   The wizard auto-selects the single module and prints `ident_number: 0x0d0f`,
   `user_parameters: Some(&[0x20])` and the 8 `config` bytes.  Because the profile uses the
   PROFIBUS special identifier format, `gsdtool` cannot derive the process image sizes and
   warns about it - add them by hand: `buffer_inputs = [0u8; 1]`, `buffer_outputs = [0u8; 1]`.

The GSD describes only the *slave*.  The `config`/`user_parameters`/`ident_number` on the
master side are always explicit values, so the ident number is arbitrary as long as both sides
agree (`0x0D0F` is a placeholder, not a registered PNO ident).

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
[    0.000000] dp_slave_pico::main: DP-V0 slave #3: ident 0x0D0F, B9600, pi_i 1 B, pi_q 1 B
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
increments once per second.  The output byte written by the master lands in `pi_q` and drives
the external LED on GPIO6, so a master-side write of `0x01`/`0x00` visibly switches the LED
on and off - that is the end-to-end check of the write path (master -> slave).

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
