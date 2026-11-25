# Field_Instrument_Core Hardware Utilities (`extras/tools/`)

Reusable hardware configuration, RF spectrum diagnostic, link-testing, and bench characterization tools shared across ESP32-S3 field instruments built on `Field_Instrument_Core`. Because these live under Arduino's `extras/` directory, they are excluded from ESP32-S3 sketch compilation while remaining immediately accessible from a standard library install.

## Subdirectories

### `e220/` -- Ebyte E220-900T30S / T22D LoRa Utilities
Supports both Raspberry Pi BCM GPIO lines and USB-FTDI serial adapters (`DTR=M0`, `RTS=M1`, `CTS#=AUX`):

| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`e220/setup_e220.py`](e220/setup_e220.py) | `v1.3.0` | Mode 3 register programmer (`--show`, `--configure`, TX power, air rate, RSSI byte, WOR/LBT/FEC). |
| [`e220/scan_e220_channels.py`](e220/scan_e220_channels.py) | `v1.1.0` | Passive `902-928 MHz` ambient RF noise scanner using temporary RAM writes (`0xC2`, zero EEPROM wear) + CSV export. |
| [`e220/bench_e220_multirate.py`](e220/bench_e220_multirate.py) | `v1.0.3` | Stage 1 hardware-in-the-loop multi-rate (`4.8k` / `9.6k` / `19.2k`) ToA, guard sweep, PER, thermal stability, and 4-ACK rate-switch characterization suite (supports `--self-test`). |
| [`e220/test_e220_link.py`](e220/test_e220_link.py) | `v1.1.0` | Bidirectional RF link and burst throughput tester between two E220 transceivers. |

*(Companion `RTK_Rover` Base Station daemon `e220_pacer.py`, `e220-pacer.service`, `set_e220_power.py`, `update_e220_pacer.sh`, and `stage2_bench_verify.py` remain in [`RTK_Rover/scripts/`](https://github.com/rguilmet/RTK-Rover/tree/master/scripts).)*

### `um98x/` -- Unicore High-Precision GNSS Utilities

| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`um98x/setup_um98x.py`](um98x/setup_um98x.py) | `v1.3.0` | Unified configuration and diagnostic CLI for Unicore `UM980` (`--device um980`) and dual-antenna `UM982` (`--device um982`) receivers (`--freset`, `--query`, signal-group sync). |
