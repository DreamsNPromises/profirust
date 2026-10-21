# profirust example: DP-V0 slave simulator on RP2040

This firmware turns a Raspberry Pi Pico into **several DP-V0 slave stations** at once: together
with a DP master it forms the smallest possible test bench - one master and two simulated slaves
with one LED each.

It implements steps 3 to 5 of
[`docs/PLAN-dp-slave-simulator-final.md`](../../docs/PLAN-dp-slave-simulator-final.md): the
firmware drives a table of `profirust::dp::DpSlave` objects from the main loop and answers the
master's `Slave_Diag`, `Set_Prm`, `Chk_Cfg` and cyclic data exchange requests for every station.
The stations are described by the table in [`src/slaves.rs`](src/slaves.rs) - by default two
stations on addresses 3 and 4.  Speeds above 500 kbit/s (steps 6/7) are not implemented yet.

Instead of real input signals every station runs a **test counter** in its input process image:
the counter starts at 0 and is incremented once per second, station #3 by 1 and station #4 by 2,
so the two stations are easy to tell apart in the master's input log.  The master's output byte
of each station drives that station's LED.

## Wiring

| Pico pin | Function |
|----------|----------|
| GPIO0 | UART0 TX -> RS-485 transceiver DI |
| GPIO1 | UART0 RX <- RS-485 transceiver RO |
| GPIO2 | RS-485 direction (DE/RE), high while transmitting |
| GPIO15 | external LED of station #3 -> bit 0 of its `pi_q` (active high) |
| GPIO14 | external LED of station #4 -> bit 0 of its `pi_q` (active high) |
| GND | common ground with the master |
| GPIO25 (on-board LED) | heartbeat: toggles once per second |

External LED wiring (any 3.3 V LED works):

```text
GPIO15 ---[ 220..330 Ohm ]---|>|--- GND    station #3
GPIO14 ---[ 220..330 Ohm ]---|>|--- GND    station #4
                             LED
```

A written output bit `0` turns the LED off, `1` turns it on.  If the master stops exchanging
data with a station, that station's watchdog clears its `pi_q` (fail-safe), so its LED goes dark
on its own - independently of the other station.

As with the master example, the RS-485 driver enable is handled by the firmware; the same
transceiver circuit can be used on both ends.  Don't forget bus termination (and a pull-up on
the RX line if your transceiver needs one).

## Test-bench profile

All stations share one profile; only address and test counter differ.

| Parameter | Value |
|-----------|-------|
| Station addresses | 3 and 4 |
| Ident number | `0x0D0F` (GSD [`gsd-files/PRF_0D0F.gsd`](../../gsd-files/PRF_0D0F.gsd)) |
| `User_Prm_Data` | `[0x20]` |
| `Chk_Cfg` | `[0x42, 0x00, 0xBE, 0x81, 0x82, 0x00, 0xBD, 0x81]` |
| Inputs (`pi_i`, slave -> master) | 1 byte (test counter, `+1`/s on #3, `+2`/s on #4) |
| Outputs (`pi_q`, master -> slave) | 1 byte (bit 0 -> LED) |
| Baudrate | `B9600` (compile-time constant) |
| Watchdog | armed by the master's `Set_Prm`, fail-safe clears the outputs |

The profile is defined by the constants at the top of [`src/slaves.rs`](src/slaves.rs) and is
described in machine-readable form by the GSD file
[`gsd-files/PRF_0D0F.gsd`](../../gsd-files/PRF_0D0F.gsd).  Both must stay in sync: if the slave
and the master disagree, the stations reject `Set_Prm`/`Chk_Cfg` and the master reports
`ParameterError`/`ConfigError`.

### Matching master configuration

Any DP master can talk to these stations as long as its configuration matches the profile above.
There are two ways to set up the master:

1. **Import the GSD** (`gsd-files/PRF_0D0F.gsd`) into the master's configuration tool and add
   the station "DP-V0 Slave Simulator (RP2040)" **twice**, once per bus address.  A GSD describes
   a *station type*, not a bus address: the same file is used for every instance, the address is
   part of the master's project.  The file declares a single compact module with exactly the
   `Chk_Cfg` bytes and the process image above, so there is nothing else to choose.
2. **Let `gsdtool` generate the `PeripheralOptions`** for a profirust master:

   ```bash
   cargo run -p gsdtool -- config-wizard gsd-files/PRF_0D0F.gsd
   ```

   The wizard auto-selects the single module and prints `ident_number: 0x0d0f`,
   `user_parameters: Some(&[0x20])` and the 8 `config` bytes.  Because the profile uses the
   PROFIBUS special identifier format, `gsdtool` cannot derive the process image sizes and
   warns about it - add them by hand: `buffer_inputs = [0u8; 1]`, `buffer_outputs = [0u8; 1]`,
   once per peripheral.

The GSD describes only the *slave*.  The `config`/`user_parameters`/`ident_number` on the
master side are always explicit values, so the ident number is arbitrary as long as both sides
agree (`0x0D0F` is a placeholder, not a registered PNO ident).

A master which derives its station list from a config file needs one entry per simulated
station, for example (UniGateway syntax, `config.lua`):

```lua
PROFIBUS_pico = PROFIBUS_master {
    port                    = "/dev/ttyACM0",
    master_address          = 2,
    baudrate                = BAUDRATE.B9600,
    highest_station_address = 4,     -- HSA: highest station address of the token ring
    max_retry_limit         = 3,
    watchdog_timeout_ms     = 1000,
    gap_wait_rotations      = 10,

    slaves = {
        PROFIBUS_slave { address = 3, gsd = "PRF_0D0F.gsd", modules = {}, global_parameters = {}, module_parameters = {}, push_interval_ms = 0 },
        PROFIBUS_slave { address = 4, gsd = "PRF_0D0F.gsd", modules = {}, global_parameters = {}, module_parameters = {}, push_interval_ms = 0 },
    },
},
```

Two things are easy to forget:

- `highest_station_address` is the HSA of the token ring: it bounds the GAP scan for other
  *active* stations.  Passive stations may have higher addresses and are still addressed
  directly by the DP master, but the conventional value is "highest simulated address"
  (here `4`), so use that if in doubt.
- Sources/subscriptions refer to station addresses (`S3.B0`, `S4.B0`), so add mappings for both
  stations.

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

Log of the slave device:

```terminal
[    0.000000] dp_slave_pico::main: Booting PROFIBUS DP-V0 slave simulator...
[    0.000000] dp_slave_pico::main: System clock: 240000000 Hz
[    0.000000] dp_slave_pico::main: DP-V0 simulator: 2 slaves, ident 0x0D0F, B9600, pi_i 1 B, pi_q 1 B
[    0.000000] dp_slave_pico::main:   slave #3: LED GPIO15, test counter +1/s
[    0.000000] dp_slave_pico::main:   slave #4: LED GPIO14, test counter +2/s
[    1.234567] dp_slave_pico::main: DP slave #3 is now Wcfg
[    1.235012] dp_slave_pico::main: DP slave #4 is now Wcfg
[    1.241234] dp_slave_pico::main: DP slave #3 is now Dxchg
[    1.241890] dp_slave_pico::main: DP slave #4 is now Dxchg
```

Log of a profirust master:

```terminal
profirust::dp::peripheral: Peripheral #3 becomes ready for data exchange.
profirust::dp::peripheral: Peripheral #4 becomes ready for data exchange.
```

Inputs of the two stations increment independently (station #4 at twice the rate, shown with the
UniGateway tag names `S3.B0`/`S4.B0`):

```terminal
S3.B0 = 0x01   S4.B0 = 0x02
S3.B0 = 0x02   S4.B0 = 0x04
S3.B0 = 0x03   S4.B0 = 0x06
```

The output byte written by the master lands in the `pi_q` of the addressed station and drives
its LED (GPIO15 for #3, GPIO14 for #4), so a master-side write of `0x01`/`0x00` visibly switches
that LED on and off - the end-to-end check of the write path (master -> slave) for every
station separately.

## Timing

Before answering, a station waits for the minimum T<sub>SDR</sub> (at least 11 bit times) that
the master requested in `Set_Prm`.  All stations are configured with the same value; the reply
delay uses the maximum, so it is valid regardless of which station answers.
`Rp2040Phy::transmit_data()` additionally delays the first byte by T<sub>SET</sub>, so the
busy-wait in `wait_min_tsdr()` only covers the difference (see section 6 of the plan).  At
9.6 kbit/s this is about 1 ms per response.

## Notes

- Nothing is logged from the PROFIBUS hot path on core0; all log records go through an SPSC
  queue to core1 (USB CDC), same as in the master example.
- `profirust` is used with `default-features = false, features = ["phy-rp2040"]`; the slave
  implementation is `no_std`-clean and needs no allocator.
- Process images live in `static` memory (`src/slaves.rs`) because `DpSlave` borrows them for
  `'static`; adding a station means adding a row to [`SLAVES`](src/slaves.rs) and one LED to the
  array in `main()`.
