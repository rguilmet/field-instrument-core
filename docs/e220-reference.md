# Ebyte E220-900T30S Practical Engineering Reference

**Document Version:** 3.2  
**Last Updated:** November 16, 2026  
**Firmware Target:** `v0.8.13`  
**Companion Software:** `e220_pacer.py v3.7.1`  
**Target Hardware:** Ebyte `E220-900T30S` (`850.125 - 930.125 MHz`, `1 W` / `+30 dBm` SMD UART LoRa Module)  
**Applicability:** Directly applies to `E220-900T30S` / `E220-900T30D` (`1 W`) and `E220-900T22S` / `E220-900T22D` (`160 mW`, noting TX power bit differences in Section 4.4).  

**Vendor Manuals & Articles:** Archived in [`Field_Instrument_Reference` (`EByte E220` & `LoRa_E220`)](https://github.com/rguilmet/field-instrument-reference/tree/main/datasheets/EByte%20E220)

**Reference Implementations (`Field_Instrument_Core` & `RTK_Rover`):**
* **Shared E220 Hardware Utilities (`Field_Instrument_Core`):** [`extras/tools/e220/setup_e220.py`](../extras/tools/e220/setup_e220.py) (`v1.3.0`), [`extras/tools/e220/scan_e220_channels.py`](../extras/tools/e220/scan_e220_channels.py) (`v1.1.0`), [`extras/tools/e220/bench_e220_multirate.py`](../extras/tools/e220/bench_e220_multirate.py) (`v1.0.3`), [`extras/tools/e220/test_e220_link.py`](../extras/tools/e220/test_e220_link.py) (`v1.1.0`)
* **ESP32-S3 Rover Driver & RTCM3 Gatekeeper:** [`src/e220_driver/e220_driver.cpp`](https://github.com/rguilmet/RTK-Rover/blob/master/src/e220_driver/e220_driver.cpp), [`src/e220_driver/rover_telemetry.h`](https://github.com/rguilmet/RTK-Rover/blob/master/src/e220_driver/rover_telemetry.h), [`user_config.h`](https://github.com/rguilmet/RTK-Rover/blob/master/user_config.h)
* **Raspberry Pi Base Pacer, Power & Verifier (`RTK_Rover`):** [`scripts/e220_pacer.py`](https://github.com/rguilmet/RTK-Rover/blob/master/scripts/e220_pacer.py) (`v3.7.1`), [`scripts/e220-pacer.service`](https://github.com/rguilmet/RTK-Rover/blob/master/scripts/e220-pacer.service), [`scripts/update_e220_pacer.sh`](https://github.com/rguilmet/RTK-Rover/blob/master/scripts/update_e220_pacer.sh), [`scripts/set_e220_power.py`](https://github.com/rguilmet/RTK-Rover/blob/master/scripts/set_e220_power.py) (`v1.1.0`), [`scripts/stage2_bench_verify.py`](https://github.com/rguilmet/RTK-Rover/blob/master/scripts/stage2_bench_verify.py) (`v1.0.3`), [`docs/benchmarks/e220_stage1_20261116/`](https://github.com/rguilmet/RTK-Rover/blob/master/docs/benchmarks/e220_stage1_20261116/README.md)

---

## Table of Contents
1. [Why This Document Exists](#1-why-this-document-exists)
2. [Internal Hardware Architecture](#2-internal-hardware-architecture)
3. [Pinout, Power Supply & Electrical Traps](#3-pinout-power-supply--electrical-traps)
4. [Complete Register Map & Mode 3 Programming Protocol](#4-complete-register-map--mode-3-programming-protocol)
5. [The AUX Pin Truth & Two-Regime Pacing](#5-the-aux-pin-truth--two-regime-pacing)
6. [Air Rates, Link Capacity & Multi-Constellation Interleaving](#6-air-rates-link-capacity--multi-constellation-interleaving)
7. [Live Mode 0 RSSI & Ambient Noise Floor Queries](#7-live-mode-0-rssi--ambient-noise-floor-queries)
8. [RF Spectrum Selection & Near-Field Desense](#8-rf-spectrum-selection--near-field-desense)
9. [Bi-Directional Half-Duplex LoRa Protocol & Telemetry Uplink](#9-bi-directional-half-duplex-lora-protocol--telemetry-uplink)
10. [Touchscreen Remote Air-Rate Switching Protocol](#10-touchscreen-remote-air-rate-switching-protocol)
11. [Base Station Driver Architecture (e220_pacer.py)](#11-base-station-driver-architecture-e220_pacerpy)
12. [Store-and-Verify Sliding-Window Gatekeeper Pattern](#12-store-and-verify-sliding-window-gatekeeper-pattern)
13. [Rapid Field Troubleshooting Matrix](#13-rapid-field-troubleshooting-matrix)

---

## 1. Why This Document Exists

The official Ebyte manuals (`E220-900T30S_UserManual_EN_v1.0` and `E220-xxxTxxx_UserManual_EN_v1.7`) suffer from severe translation ambiguities, missing timing diagrams, and several outright factual errors regarding `AUX` pin behavior, EEPROM write acknowledgments, register reply framing, and half-duplex arbitration.

This guide documents the **actual silicon behavior** verified empirically across dozens of bench and field test campaigns (oscilloscope/ISR pulse timing, raw byte-level UART wire-tapping, RF spectrum scans, and cadastral survey field trials) while operating a `100.00%` lossless `1 Hz` multi-constellation GNSS `RTCM3` radio link with bi-directional remote telemetry.

---

## 2. Internal Hardware Architecture

Understanding the `E220-900T30S` requires recognizing that **your host MCU does not talk to a raw LoRa chip**. It communicates with an internal proprietary Ebyte protocol microcontroller that manages a Semtech `LLCC68` LoRa transceiver:

```text
+--------------------+        UART         +-------------------------------------------------------+
|                    |  TX  --------> RXD  |  Internal Ebyte MCU      SPI      Semtech LLCC68      |
|  Host Processor    |  RX  <-------- TXD  |  - 400-Byte RAM FIFO  <=======>  - 200B Max Payload   |
|  (ESP32 / RPi /    |  GPIO -------> M0   |  - Mode 0..3 State                - 32 MHz Passive    |
|   STM32 / RP2040)  |  GPIO -------> M1   |    Machine & AT/Hex                 XTAL (NO TCXO!)   |
|                    |  ISR <-------- AUX  |    Command Parser                         |           |
|                    |  GPIO -------> RST  |  - ~500ms RC Sleep Timer          +-------+-------+   |
+--------------------+                     |    (~320-340 ppm drift)           | 1W PA / +14dB |   |
                                           |                                   | RX LNA Switch |   |
                                           +-----------------------------------+-------+-------+---+
                                                                                       |
                                                                                   50-Ohm RF
```

### Critical Silicon Realities

1. **400-Byte MCU RAM Buffer vs. 200-Byte LoRa Packet Limit:**
   * Each single over-the-air `LLCC68` LoRa packet carries at most **200 bytes** (`REG1[7:6] = 00`), and hardware `AUX` chaining is guaranteed only up to **400 bytes** per atomic write. *(Bench testing in [`test/_045.._047`](benchmarks/e220_stage1_20261116/README.md) observed the internal UART FIFO absorbing isolated idle pairs up to `442 B` (`253 + 189 B`), but sustained continuous pacing must still obey full `ToA` floors so FIFO never overflows.)*
   * This 400-byte staging enables **zero-dead-air hardware packet chaining**: when you write a `201..400 B` message at `115200` baud with no inter-byte gaps, the MCU loads the first `200 B` into the `LLCC68` immediately and holds bytes `201..N` in RAM, chaining Subpacket 2 onto the air the instant Subpacket 1 completes.
2. **Passive `32 MHz` Crystal (`XTAL`), NOT a `TCXO`:**
   * Unlike industrial modules fitted with a temperature-compensated oscillator (`TCXO`), the `E220-900T30S` uses a **passive `32 MHz` quartz crystal** adjacent to a `1 W` power amplifier.
   * Under high duty cycles (`> 60%`), PA self-heating can induce slight carrier offset; however, full Stage 1 bench soak testing at **`30 dBm` (`1 W`) and `~85–90%` duty for `5.0 min/rate`** confirmed **`526/526` (`100.0%`) @ `4.8 kbps`** and **`777/777` (`100.0%`) @ `9.6 kbps`** with **`0` CRC errors** on stable `5 V` power. Wide bandwidth rates (`38.4`, `62.5 kbps`) tolerate thermal offset even more broadly, while `4.8` / `9.6` / `19.2 kbps` still require managed pacing and clean `>= 20 ms` turnaround guards.
3. **Uncalibrated Internal RC Sleep Timer (`~330 ppm`):**
   * In Wake-on-Radio (`WOR`) Modes 1 and 2, the sleep/wake period (`500..4000 ms`) runs off an internal low-power RC oscillator measured at **`+320..+340 ppm`** (`~115 ms` drift per 6 minutes relative to GPS UTC). Transparent Mode 0 (`M0=0, M1=0`) bypasses this timer entirely.

---

## 3. Pinout, Power Supply & Electrical Traps

### 3.1 Pin Definitions

| Pin Name | Direction (E220 View) | Internal Pull | Electrical / Functional Notes |
| :--- | :---: | :---: | :--- |
| **`M0`** | Input | **Weak Pull-Up (`3.3 V`)** | Mode select bit 0 (`3.3 V` logic; **never leave floating**). |
| **`M1`** | Input | **Weak Pull-Up (`3.3 V`)** | Mode select bit 1 (`3.3 V` logic; **never leave floating**). |
| **`RXD`** | Input | Weak Pull-Up | UART data into E220 (`3.3 V` CMOS; use series `1k` resistor if driven from `5 V` TTL). |
| **`TXD`** | Output | Push-Pull (`3.3 V`) | UART data out of E220 (idles `HIGH = 3.3 V` once powered and initialized). |
| **`AUX`** | Output | Push-Pull / Open-Drain | `HIGH` = Idle/Buffer ready; `LOW` = Busy / Mode switching / RX UART shifting. Enable `INPUT_PULLUP` on host MCU. |
| **`VCC`** | Power | -- | `3.3 V - 5.5 V` DC (`>= 5.0 V` required to reach full `+30.0 dBm`; `3.3 V` rail caps RF output around `+26..27 dBm`, while `+21 dBm` works cleanly at `3.3 V` or `5.0 V`). |
| **`GND`** | Ground | -- | Low-impedance RF + power ground plane. |
| **`NRST` / `RST`** | Input | Internal Pull-Up | Active-`LOW` hardware reset (`drive LOW >= 50 ms`, release `HIGH`, then wait for `AUX == HIGH`). |

### 3.2 Power Supply & Bulk Decoupling Requirements
* **Peak TX Current (`E220-900T30S`):**
  * `@ 30 dBm (1000 mW)`: **`650 - 850 mA`** instantaneous current spikes on every packet.
  * `@ 21 dBm (125 mW)`: **`200 - 300 mA`** instantaneous current spikes.
  * `@ RX Continuous (Mode 0)`: **`~14 - 18 mA`**.
* Place **`>= 100 uF` to `470 uF` low-ESR electrolytic/tantalum + `100 nF` ceramic** directly across `VCC` and `GND` at the module pads. Brownouts mid-burst corrupt the internal MCU state machine and lock `AUX` `LOW`.

### 3.3 Operating Modes Truth Table

| Mode | `M1` | `M0` | Name | UART Baud | RF TX Behavior | RF RX Behavior | `C0 C1 C2 C3` RSSI Cmd? | `9600` EEPROM Cfg? |
| :---: | :---: | :---: | :--- | :---: | :--- | :--- | :---: | :---: |
| **0** | **`0`** | **`0`** | **Transparent / Normal** | `REG0` (`115200`) | Immediate TX (no WOR preamble) | **100% Continuous RX** | **YES (`< 30 ms`)** | No |
| **1** | **`0`** | **`1`** | **WOR Transmitting** | `REG0` (`115200`) | Prepends `WOR_Period` (`500 ms+`) wake preamble | Periodic `WOR` Sleep/Wake RX | YES (`~500 ms` delay) | No |
| **2** | **`1`** | **`0`** | **WOR Receiving** | `REG0` (`115200`) | TX Disabled | Periodic `WOR` Sleep/Wake RX | NO | No |
| **3** | **`1`** | **`1`** | **Deep Sleep / Config** | **Fixed `9600 8N1`** | TX Disabled | RX Disabled | No | **YES (`C0/C1/C2`)** |

### 3.4 The Open-`M0` Connector Failure ("Fake Mode 0")
Because **both `M0` and `M1` have internal weak pull-ups to `3.3 V` (`1`)**, a broken wire or cold solder joint on `M0` creates an insidious failure:
1. **Boot Self-Test (`Mode 3: M1=1, M0=1`) PASSES 100%:** Even with `M0` disconnected, the internal pull-up holds `M0 = 1`. The module enters Mode 3 and responds to EEPROM commands.
2. **Runtime Transition (`Mode 0: M1=0, M0=0`) Enters WOR Mode 1 (`M1=0, M0=1`):** MCU drives `M0 = 0`, but the open pin floats `HIGH`. The module enters Mode 1 (500 ms duty-cycled sleep).
3. **Diagnostic Signature:** Packets arrive with strong SNR, but **never closer than 500 ms apart** (`<= 1` subpacket/s ceiling). Multi-subpacket frames (`> 200 B`) always truncate. RSSI commands arrive phase-locked to a 500 ms grid.

#### Software Continuity Self-Test at Boot
When switching between any two modes, the E220 pulls `AUX` **`LOW` for `9..11 ms`**:
1. Drive `(M1=1, M0=1)` and wait for `AUX == HIGH`.
2. Toggle only `M0` `LOW` `(1,1) -> (1,0)` while monitoring `AUX` for 25 ms. An `AUX` `LOW` pulse verifies `M0` continuity.
3. Restore `(1,1)`, wait `AUX == HIGH`, then toggle only `M1` `LOW` `(1,1) -> (0,1)`. An `AUX` `LOW` pulse verifies `M1` continuity.
4. Drive `(M1=0, M0=0)` and wait for `AUX == HIGH` + 60 ms guard before opening operational traffic.

---

## 4. Complete Register Map & Mode 3 Programming Protocol

### 4.1 Mode 3 Serial Rules
* **Mode 3 (`M1=1, M0=1`) ALWAYS uses `9600 bps, 8N1`**, regardless of the operational baud rate in `REG0`.
* Flush Mode 0 UART TX, assert **`M0=1, M1=1` BEFORE switching host UART to `9600 bps`**, and wait `>= 60 ms` settle time. Switching the ESP32 UART to `9600` while the E220 is still listening at `115200` in Mode 0 injects a baud-mismatch framing glitch into the transceiver RX shift register (`test/_040`).
* Issue a pre-write Mode 3 read probe (`C1 00 06`) to verify UART alignment before sending `C0 00 06`, insert **`>= 15–20 ms` settling delay between consecutive Mode 3 commands** (`C1 <-> C2` in either direction, preventing `verify_mismatch:none` or retry frames seen in `test/_044` and `test/_045`), and keep `M0=1, M1=1` steady between initial retries instead of toggling `Mode 0 -> Mode 3` at `9600` baud.
* Wait **`25 ms` after `M0=0, M1=0`** before restoring operational baud (`115200`), wait **`>= 55–80 ms` + `AUX == HIGH`**, flush Mode 3 reply (`test/_045` Cycle 3 `9.6k -> 4.8k` showed `45 ms` was marginal before transmitting a slower-SF `4.8k` RF frame), and run a **`5 ms` Mode 0 liveness probe (`C0 C1 C2 C3 00 02`)** when `REG1[5]=1` to detect and bounce rare FTDI/Mode-0 exit desyncs (observed once at `9.6k` in `test/_047` `21 dBm`; implemented across both `e220_write_default_config_registers()` and `e220_read_config_registers()` via `e220_exit_mode3_to_mode0()` / `e220_probe_mode0_liveness()` in firmware `v0.8.12`, plus `e220_pacer.py v3.7.0` & `bench_e220_multirate.py v1.0.3`).

### 4.2 Binary Command Frame Formats

| Operation | Host Command Bytes (Hex) | Expected E220 Reply (Hex) | Persistence |
| :--- | :--- | :--- | :--- |
| **Write EEPROM** | `C0 <START> <LEN> <DATA[0..LEN-1]>` | `C1` *(or `C0`!)* `<START> <LEN> <DATA...>` | Saved to non-volatile EEPROM |
| **Read Registers** | `C1 <START> <LEN>` | `C1 <START> <LEN> <DATA[0..LEN-1]>` | Read-only |
| **Write Temp RAM** | `C2 <START> <LEN> <DATA[0..LEN-1]>` | `C1` *(or `C2`)* `<START> <LEN> <DATA...>` | Lost on power-cycle or `RST` |

> **`0xC0` vs `0xC1` Acknowledgment & Mode 3 Retry Protocol (`v0.8.7` / `v0.8.9`):** Certain E220 firmware builds echo `0xC0` instead of `0xC1` upon successful EEPROM write. Always accept `reply[0] in (0xC0, 0xC1)`. Field testing (`test/_038`, `test/_040`) showed that entering Mode 3 while a Mode 0 RSSI command or RF transmission is still active — or switching ESP32 UART to `9600` baud *before* raising `M0/M1` / bouncing `M0/M1` while locked at `9600` baud — injects a framing glitch and provokes repeated `FF FF FF` format-error replies (`got 3 bytes` / `got 4 bytes`) even while a subsequent `e220_read_config_registers()` (`set_mode(1,1)` *then* `9600`) succeeds immediately. Firmware `v0.8.9` cancels active Mode 0 RSSI queries, waits for `s_tx_air_busy_until_ms` + `AUX == HIGH` + `Serial0.flush()`, asserts **`e220_set_mode(1, 1)` BEFORE `Serial0.updateBaudRate(9600)`**, runs `e220_mode3_sync_read()` (`0xC1 0x00 0x06`), retries Attempts `1..3` in Mode 3 without bouncing `M0/M1` (`280 ms` timeout with `35 ms` frame-end exit for `>= 3 B` error frames), reserves a clean baud-matched `Mode 0 (115200) -> Mode 3 (9600)` bounce + sync read for Attempt `4`, and falls back to `e220_read_config_registers()` to resync `s_active_air_rate` if all attempts fail.

### 4.3 Production Register Profiles (Profile Rev 3 Default)

| Parameter | Default Forest / Field (`4.8 kbps`) | Canopy (`9.6 kbps`) | Open-Sky / Bench (`19.2 kbps`) |
| :--- | :---: | :---: | :---: |
| **`ADDH` / `ADDL`** | `0x00` / `0x00` | `0x00` / `0x00` | `0x00` / `0x00` |
| **`REG0` (UART + Air Rate)** | **`0xE3`** (`115200 8N1`, `4.8k`) | **`0xE4`** (`115200 8N1`, `9.6k`) | **`0xE5`** (`115200 8N1`, `19.2k`) |
| **`REG1` (Pkt + RSSI + Power)**| **`0x20`** (`200B`, `RSSI=1`, `30dBm`; `0x23` for `21dBm`) | **`0x20`** (`200B`, `RSSI=1`, `30dBm`; `0x23` for `21dBm`) | **`0x20`** (`200B`, `RSSI=1`, `30dBm`; `0x23` for `21dBm` bench) |
| **`CHAN` (Frequency)** | **`0x4B`** (`CH 75` = `925.125 MHz`) | **`0x4B`** (`CH 75` = `925.125 MHz`) | **`0x4B`** (`CH 75` = `925.125 MHz`) |
| **`REG3` (Transparent Mode)** | **`0x00`** (Transparent, LBT Off) | **`0x00`** (Transparent, LBT Off) | **`0x00`** (Transparent, LBT Off) |
| **`CRYPT_H` / `CRYPT_L`** | `0x00` / `0x00` | `0x00` / `0x00` | `0x00` / `0x00` |

---

## 5. The AUX Pin Truth & Two-Regime Pacing

### 5.1 `AUX == HIGH` Does NOT Mean RF Transmission Finished
`AUX` indicates whether the **internal 400-byte UART RAM buffer can accept bytes without overflow** — NOT whether the `LLCC68` RF power amplifier has finished airing.

At `115200` baud, end-of-frame is detected after a 3-character idle gap (`~0.26 ms`). What `AUX` does depends strictly on frame size:

### 5.2 Regime 1: Single-Subpacket Writes (`1 .. 200 Bytes`)
Because all bytes transfer into the idle `LLCC68` FIFO in `< 0.5 ms`, the 400-byte RAM buffer is immediately empty — **`AUX` stays `HIGH` while the packet transmits on the air!**

```text
Host UART TX:  |=== 144B (12.5ms) ===|---(idle)-------------------------------|
E220 RAM Buf:  [ Filling 0..144B     ][Xfer->LLCC68 (<0.5ms): RAM NOW EMPTY! ]
E220 AUX Pin:  HIGH ---------------------------------------------------------- (NEVER STAYS LOW!)
LLCC68 RF PA:                         |======= On-Air TX: 114.0 ms (@19.2k) =======|
Rover UART RX:                                                                    |== 144B (12.5ms) ==|
                                      |<----------- Must Wait: ToA + 35ms Guard ---------->|
```

* **Pacing Rule (`1..200 B`, `e220_pacer.py v3.7.0`):**
  1. Verify `AUX == HIGH` before writing.
  2. Write complete frame (or `<= 200 B` coalesced batch) atomically and `flush()`.
  3. Sleep deterministically for **`ToA(N) + Guard_Delay`** (`Guard_Delay = max(guard_delay_s, 35 ms)` @ `<= 5.0 kbps`, `30 ms` default @ `>= 9.6 kbps` — validated down to `10 ms` in Stage 1 benchmarks) using the Stage-1-calibrated piecewise `ToA` curve. Never wait for an `AUX` falling edge.
* **Mode 0 `C0 C1 C2 C3` Command Interaction During Active `<= 200 B` TX (`test/_038` Finding):**
  * Because `AUX` remains `HIGH` while a single-subpacket (`<= 200 B`, e.g., `48 B` Rover telemetry @ `4.8 kbps`, `~166 ms` on air) is actively transmitting on the `LLCC68`, writing `C0 C1 C2 C3 00 02` to `RXD` immediately after `Serial.flush()` causes the E220 MCU to hold the command in its UART RAM buffer and **pull `AUX` `LOW` for the remaining RF TX duration (`100-130 ms` at `4.8 kbps`)**, only outputting the 5-byte `0xC1 0x00 0x02 <noise> <pkt>` reply (`AUX` low `~7.0 ms`) once RF TX completes (`~166 ms` total latency).
  * **Guard Rule (`v0.8.12`):** After transmitting any Mode 0 uplink packet (`<= 200 B`), clear `isr_aux_fall_us = 0` when restoring `e220_suppress_aux_isr`, hold Mode 0 RSSI queries and Mode 3 transitions until `s_tx_air_busy_until_ms` (`70+1.87*N / 45+1.19*N / 29+0.80*N + 25 ms`) expires, accept cold-boot `pkt_raw == 0x00` prior to the 1st RF packet, and enforce `E220_RSSI_REPLY_TIMEOUT_MS = 95 ms` bounded by `sf_ms >= 990 ms`.

### 5.3 Regime 2: Two-Subpacket Writes (`201 .. 400 Bytes`)
```text
Host UART TX:  |==== 343B (29.8ms) ====|
E220 RAM Buf:  [0..200B -> LLCC68][ 143B waiting in RAM... ][143B -> LLCC68: RAM EMPTY!]
E220 AUX Pin:  HIGH ---|_______________LOW (capped ~398ms @4.8k!)___|------ HIGH ----------------
LLCC68 RF PA:          |=== Subpkt 1: 200B (up to 545ms @4.8k) ===|=== Subpkt 2: tail ===|
                                                                   ^
                                       DANGER! At <=9.6 kbps, AUX can rise 100-147ms BEFORE
                                       Subpacket 1 finishes airing on LLCC68!
```

* **Critical `test/_038` Silicon Discovery (Early `AUX` Rise at `4.8 / 9.6 kbps`):**
  * At `19.2 kbps` (`Subpacket 1 ~ 148 ms`), `AUX_low` tracks Subpacket 1 closely (`~144.5 ms`).
  * At lower air rates (`4.8 kbps` where `200 B` takes `505-545 ms`, and `9.6 kbps` where `200 B` takes `265-281 ms`), the E220 MCU empties its 400-byte UART RAM buffer into internal SPI staging **before Subpacket 1 finishes transmitting on RF** (`AUX_low` measured at `398.5 ms` @ `4.8 kbps` and `~200 ms` @ `9.6 kbps`).
  * Waiting only `ToA(N - 200) + Guard_Delay` after `AUX` rises `HIGH` returns **`100-147 ms` too early** while Subpacket 2 is still airing, causing the next RTCM frame (`1094` / `1084`) to abort Subpacket 2 (`[RTCM DROP TRUNC]` on `1074` with `200 B` received) and lose Subpacket 1 of the following frame!
* **Pacing Rule Rev 2 (`201..400 B`, `e220_pacer.py v3.6.1`):**
  1. Verify `AUX == HIGH` before writing.
  2. Write all `N` bytes in one contiguous write (never pre-split in host software).
  3. Wait for `AUX` falling edge (`< 50 ms` timeout).
  4. Wait for `AUX` rising edge (`1.5 s` watchdog timeout).
  5. **Mandatory Tail + Full-ToA Floor Wait:** Sleep for **`max(ToA(N - 200), ToA(200) + ToA(N - 200) - elapsed_since_flush) + Guard_Delay`** so early `AUX` release at `<= 9.6 kbps` never allows the next frame to collide with Subpacket 2.

### 5.4 Hardware-Calibrated Time-on-Air (`ToA`) Formula

Full Stage 1 bench characterization ([`docs/benchmarks/e220_stage1_20261116/`](benchmarks/e220_stage1_20261116/README.md), `1 ms` USB-FTDI resolution across `18..400 B` at `21 dBm` and `30 dBm`) measured end-to-end TX-to-RX RF airtime (`<= 1.9 ms` fit residual across all tested points, proving `4.8 kbps` transfers **`35–37%` faster** than earlier `0.60`-efficiency extrapolations while `19.2 kbps` measures `186.7 ms @ 200 B`):

| Air Rate | Single-Subpacket (`1..200 B`) | `200 B` Measured | Two-Subpacket (`201..400 B`) | `316 B` / `400 B` Measured | Recommended Min Guard (`>= 10 ms` verified) |
| :---: | :--- | :---: | :--- | :---: | :---: |
| **`4.8 kbps`** | `0.070 + 0.00187 * N` s | `442.8 ms` | `0.128 + 0.00180 * N` s | `695.4` / `847.0 ms` | **`30–35 ms`** |
| **`9.6 kbps`** | `0.045 + 0.00119 * N` s | `282.8 ms` | `0.080 + 0.00112 * N` s | `433.2` / `527.2 ms` | **`30 ms`** |
| **`19.2 kbps`** | `0.029 + 0.00080 * N` s | `186.7 ms` | `0.055 + 0.00070 * N` s | `277.8` / `336.7 ms` | **`25–30 ms`** |

*(Note: Both [`e220_pacer.py`](../scripts/e220_pacer.py) `v3.7.0` and firmware `v0.8.12` run these Stage-1-calibrated piecewise `ToA` formulas live across both subpacket airtime summation and Mode 0 uplink turnaround guards.)*

---

## 6. Air Rates, Link Capacity & Multi-Constellation Interleaving

### 6.1 Link Budget & Air Rate Comparison (Stage 1 Calibrated)

| Air Rate | `REG0[2:0]` | Sensitivity | `200 B` Subpacket Measured ToA | Est. Net Throughput (`+30 ms` Guard) | RTCM Stream Viability (`~600–760 B/s`) |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **`2.4 kbps`** | `010` | `-129 dBm` | `~1,180 ms` *(est.)* | `~110–160 B/s` | Unusable for multi-constellation RTK (`+2-3 dB` over `4.8k` vs `3-8 s` age) |
| **`4.8 kbps`** | `011` | `-126 dBm` | **`442.8 ms`** | **`~390–420 B/s`** | **Async Ping-Pong Mailbox + `<=200 B` Coalescing** (`316 B` = `695 ms`; `+5 dB` vs `19.2k`) |
| **`9.6 kbps`** | `100` | **`-124 dBm`** | **`282.8 ms`** | **`~610–640 B/s`** | **Async Interleaving (`~1.24–1.35 s` refresh at Max-Sat; `+3 dB` vs `19.2k`)** |
| **`19.2 kbps`**| `101` | **`-121 dBm`** | **`186.7 ms`** | **`~920–990 B/s`** | **Standard Production Baseline** (`~1 Hz` full 3-constellation up to Max-Sat) |
| **`38.4 kbps`**| `110` | `-118 dBm` | `~82 ms` *(est.)* | `~1,800 B/s` | Bench / High-rate telemetry (`500 kHz` BW) |
| **`62.5 kbps`**| `111` | `-115 dBm` | `~57 ms` *(est.)* | `~2,800 B/s` | Short range / Line of sight only |

### 6.2 The 9.6 kbps & 4.8 kbps Channel Saturation Reality
A standard 4-constellation (`GPS + GLO + GAL`) RTCM3 stream contains:
* `1006` Base Antenna Position: Every 5s (`27 B`)
* `1074` GPS MSM4: 1.0 Hz (`~197 - 316 B`, 2 subpackets)
* `1084` GLONASS MSM4: 1.0 Hz (`~138 - 173 B`, 1 subpacket)
* `1094` Galileo MSM4: 1.0 Hz (`~209 - 275 B`, 2 subpackets)
* `1230` GLONASS Code-Phase Biases: Every 10s (`~18 B`)
* **Total Throughput:** **`~600 B/s` typical (`~764 B/s` at 14-SV Max Open-Sky)**.

At `19.2 kbps`, `1074 (316 B) + 1094 (275 B) + 1084 (173 B)` completes in `~780 ms` (`+30 ms` guards), fitting cleanly inside a `1.0 s` cadence.

At `9.6 kbps` and `4.8 kbps`, standalone `ToA` + `30 ms` guards still exceed `1,000 ms` (`~1,163 ms` @ `9.6k`, `~1,803 ms` @ `4.8k` at Max-Sat), so naive FIFO queuing falls behind linearly and drops frames. Combining **Asynchronous Latest-Value Ping-Pong Mailboxes**, **`<= 200 B` coalescing** (`1006+1230+1084`), and a **content-independent `5 s` / `10 s` macro-cycle** eliminates FIFO lag permanently while keeping age `< 1.0 s` at TX.

### 6.3 Architecture 2B Asynchronous Ping-Pong Mailboxes & `<= 200 B` Coalescing (`e220_pacer.py v3.7.0`)
[`e220_pacer.py`](../scripts/e220_pacer.py) (`v3.7.0`) replaces open-loop/FIFO epoch buffering with a dual-thread **Ping-Pong Latest-Value Mailbox** scheduler:
1. **Asynchronous Ingress & CRC24Q Filtering (`_ingress_worker()`):** Drains TCP RTCM3 bytes continuously in a background thread, validates `CRC24Q == 0`, drops empty MSMs (`<= 28 B`), and overwrites per-`msg_type` mailboxes in place (zero epoch queue backlog).
2. **TTL Expiration & Starvation-Fair Round Robin (`_pop_next_tx_batch()`):** Expires untransmitted dynamic MSMs older than `2.5 s` (`15.0 s` for static `1005/1006/1033/1230`), serves static metadata first, and rotates active constellations cleanly by starvation age (`now - last_tx` with GPS `1074` tie-breaker) using `_estimate_frame_toa_ms()` (`Stage-1 ToA + UART + 30/35 ms guard`) against the remaining macro-cycle window (`_get_remaining_downlink_ms()`).
3. **Smart `<= 200 B` Coalescing:** Packs consecutive fitting frames (e.g., `1006 + 1230 + 1084`) into a single Regime A (`<= packet_size`) UART write while refusing to push any coalesced batch across the `200 B` 2-subpacket threshold.

### 6.4 Rate-Adaptive Truncation Timeouts ([`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp))
On the receiving rover, multi-subpacket frames (`201..400 B`) exhibit an inter-subpacket gap while the base E220 finishes airing Subpacket 1 before RX receives Subpacket 2 (`186.7 / 282.8 / 442.8 ms` `200 B` ToA):

| Air Rate | Empirical `200 B` ToA | Max Subpkt-1 -> Subpkt-2 Gap (`201..400 B`) | Legacy `v0.8.11` | **`v0.8.12` Active `TRUNC_TIMEOUT_MS`** |
| :---: | :---: | :---: | :---: | :---: |
| **`19.2 kbps`** | **`186.7 ms`** | `~150 ms` | `280 ms` | **`280 ms`** |
| **`9.6 kbps`** | **`282.8 ms`** | `~245 ms` | `650 ms` | **`380 ms`** |
| **`4.8 kbps`** | **`442.8 ms`** | `~404 ms` | `1,200 ms` *(exceeded 1s epoch!)* | **`480 ms`** |

---

## 7. Live Mode 0 RSSI & Ambient Noise Floor Queries

### 7.1 Enabling Without Data Corruption
* Set **`REG1` Bit 5 = `1` (`0x20`)** and keep **`REG3` Bit 7 = `0`**.
* This enables on-demand Mode 0 register reads (`0x00` = ambient RF noise floor on `CHAN`; `0x01` = RSSI of last received packet) without appending extra bytes to payload data.

### 7.2 Undocumented 4-Byte vs. 5-Byte Firmware Reply
The datasheet claims `C0 C1 C2 C3 00 02` returns a 5-byte frame (`C1 00 02 <noise> <pkt>`). **Actual E220-900T30S firmware omits the length byte:**

| Query Sent in Mode 0 | Datasheet Reply | **Actual Silicon Reply** | Conversion |
| :--- | :--- | :--- | :--- |
| **`C0 C1 C2 C3 00 02`** | `C1 00 02 <noise> <pkt>` | **`C1 00 <noise> <pkt>` (4 B!)** | `dBm = int(u8) - 256` |
| **`C0 C1 C2 C3 00 01`** | `C1 00 01 <noise>` | **`C1 00 <noise>` (3 B!)** | `dBm = int(u8) - 256` |

Raw bytes lie in `0x80 .. 0xF5` (`>= 128`, representing `-128 dBm .. -11 dBm`), **except on cold boot before any RF packet is received**, when `pkt_raw == 0x00`.
* If `len == 4` and `buf[0..1] == C1 00` and `buf[2] >= 0x80`: 4-byte reply (`noise = buf[2] - 256`, `pkt = buf[3] - 256` when `buf[3] >= 0x80`).
* If `len == 5` and `buf[0..2] == C1 00 02`: 5-byte reply (`noise = buf[3] - 256`, `pkt = buf[4] - 256` when `buf[4] >= 0x80`).

### 7.3 Consolidated Macro-Cycle Synchronization (`v0.8.12` Architecture 2B)
Never issue `C0 C1 C2 C3 00 02` blindly during active reception, during single-subpacket RF TX (`< s_tx_air_busy_until_ms`), or when a Mode 3 rate-switch / rollback is due:
1. **Consolidated `Second 0` Post-Uplink Poll (`sf_sec == 0`):** Firmware `v0.8.12` removes `end_of_sec_gap` completely so `Seconds 1..(macro_cycle_s - 1)` stay **100% unbroken Base downlink**. One RSSI query fires per macro-cycle right after the Rover's `Second 0` uplink TX completes (`now >= s_tx_air_busy_until_ms`, `sf_ms` in `680..920 ms`), leaving **`80–320 ms`** margin before `Second 1` (plus `idle_link_probe` pre-cutoff `< 390 ms` when link is idle `> 3 s`, and `unsynced_gap` prior to GPS lock).
2. **Cold-Boot `pkt_raw == 0x00` Acceptance (`test/_048` Fix):** Prior to receiving the first RF packet after boot/reset, the E220 returns `0x00` in the `pkt_raw` byte of `C1 00 02 <noise> 00` (`5 B`). Both Stage `0b` (`prefix_mismatch`) and `allow_late_rssi` accept `(pkt_raw == 0x00 || (pkt_raw >= 0x80 && pkt_raw <= 0xF6))` while `e220_record_rssi_reply()` updates `noise_floor_dbm` immediately and preserves `rssi_valid` strictly for real packets (`rx_bytes_total > 0 && pkt_raw >= 0x80`).
3. **Bounded Intercept & Clock Discipline:** Stage 1 (`Test 4`) proved E220 Mode 0 queries answer in `4.7–6.0 ms` after `>= 20 ms` turnaround. Keep `E220_RSSI_REPLY_TIMEOUT_MS = 95 ms` and expire any active query at `sf_ms >= 990 ms`, clear `isr_aux_fall_us = 0` when unmasking `e220_suppress_aux_isr`, reject `0x80` (`-128 dBm`) noise glitches and carrier-contaminated noise (`cand_noise_dbm > -65` within `8 dB` of `cand_pkt_dbm`), and discipline ESP32 UTC clock `< 5 ms` via `$GNRMC` with `Serial1` RX backlog guarding.

---

## 8. RF Spectrum Selection & Near-Field Desense

1. **Antenna Relocation (15-20 dB Noise Reduction):**
   * The `E220-900T30S` has a sensitive `+14 dB` RX LNA. Mounting an antenna directly on a Raspberry Pi 4B exposes it to USB 3.0, HDMI, and switching regulator EMI (`-80 dBm` noise floor).
   * Relocating the antenna via a **4-foot (`1.2 m`) low-loss coaxial cable** away from digital electronics drops ambient noise to **`-96..-98 dBm`** at the Base and **`-114 dBm`** in field environments.
2. **Channel Selection (US 902-928 MHz):**
   * Consumer weather stations (e.g. Ecowitt `915.0 MHz`), power meters, and LoRaWAN crowd `902 - 916 MHz`.
   * **`CH 75` (`925.125 MHz`)** is ranked `#1` cleanest US ISM channel in empirical scans (`-96.2 dBm` average, `+10.125 MHz` clear of consumer devices).

---

## 9. Bi-Directional Half-Duplex LoRa Protocol & Telemetry Uplink

### 9.1 Consolidated Macro-Cycle TDMA Architecture (`v0.8.12` / `e220_pacer.py v3.7.0`)
Because both Base and Rover operate on the same frequency (`925.125 MHz`), downlink and uplink transmissions are deterministically scheduled against GPS UTC seconds (`mc_sec = utc_sec % macro_cycle_s`, default `5`):

```text
Consolidated Macro-Cycle (default UTC_Second % 5):
  Seconds 1..4: [=========== 100% Unbroken Base RTCM3 Downlink (Zero RSSI Stops) ===========]
  Second 0:     [= Base DL (450/620/680ms) =|50ms|= Rover Uplink =|= Post-UL RSSI (680..920ms) =]
```

* **Base Station (`e220_pacer.py v3.7.0`):** Consumes incoming RTCM3 bytes on a background ingress thread into per-type ping-pong mailboxes (`2.5 s` dynamic / `15.0 s` static TTL), prioritizes static metadata and starvation-fair round-robin MSM constellations, coalesces compatible small frames (`<= 200 B`), uses Stage-1-calibrated `ToA + 30/35 ms` guard pacing, drops empty MSMs (`<= 28 B`), keeps Seconds `1..(macro_cycle_s - 1)` `100%` unbroken downlink (`--macro-cycle-s {5,10}`, default `5`), and enforces Second 0 downlink cutoffs (`450 ms` @ `<= 5.0k`, `620 ms` @ `<= 10.0k`, `680 ms` @ `> 10.0k`) before listening strictly for Rover uplink frames (`48 B` V3 telemetry with firmware version, or legacy `45 B` / `39 B`).
* **Rover (`e220_driver.cpp` `v0.8.12`):** Transmits its 48-byte binary telemetry frame (or 10-byte rate switch request) exclusively inside the GPS-disciplined Second 0 uplink window (`500..660 ms` @ `<= 5.0k`, `670..760 ms` @ `<= 10.0k`, `730..810 ms` @ `> 10.0k`), waiting for E220 TX completion via calibrated `s_tx_air_busy_until_ms` before issuing its single post-uplink Mode 0 RSSI query (`680..920 ms`).

### 9.2 48-Byte Uplink Frame Specification (`RoverUplinkTelemetry_t`)

```text
Offset  Field                Type      Units / Encoding
-----------------------------------------------------------------------------
0..1    sync                 uint8[2]  0xAA, 0x55 (Preamble sync)
2       msg_id               uint8     0x01 (ROVER_MSG_ID_TELEMETRY)
3       payload_len          uint8     42 (Bytes following length field)
4..11   tip_lat              double    Ground-tip Latitude (WGS84 degrees)
12..19  tip_lon              double    Ground-tip Longitude (WGS84 degrees)
20..23  tip_height_m         float     Ground-tip Ellipsoidal Height (meters)
24..25  h_acc_mm             uint16    Horizontal Accuracy (millimeters)
26      fix_type             uint8     ANT1 Master Fix (0=None, 1=Single, 2=DGPS, 4=Fix, 5=Float)
27      ant2_fix_type        uint8     ANT2 Slave Moving-Baseline Fix (0=None, 4=Fix, 5=Float)
28..29  baseline_mm          uint16    Measured 3D Baseline (millimeters, 555mm nominal)
30..31  baseline_pitch_cdeg  int16     Crossbar Pitch (centidegrees, e.g. +20 = +0.20 deg)
32      heading_source       uint8     0=NONE, 1=DUAL_FIX, 2=DUAL_FLT, 3=WALK, 4=GYRO, 5=COG, 6=HOLD, 7=COMP
33      sats_master          uint8     Master Antenna satellites tracked
34      sats_slave           uint8     Slave satellites (bits [5:0]), b6=valid, b7=fixed
35..36  heading_cdeg         uint16    True Heading (centidegrees, 0..36000 -> 0.00..360.00 deg)
37      battery_pct          uint8     Battery State of Charge (0..100%, 255=USB Power)
38..39  battery_mv           uint16    Battery / VBUS Terminal Voltage (millivolts)
40      rover_rssi_dbm       int8      Last Base packet RSSI received at Rover (dBm)
41      rover_noise_dbm      int8      Ambient RF noise floor at Rover (dBm)
42      seq                  uint8     Rolling sequence counter (0..255)
43      fw_major             uint8     Rover Firmware Major Version (e.g. 0 for v0.8.6)
44      fw_minor             uint8     Rover Firmware Minor Version (e.g. 8 for v0.8.6)
45      fw_patch             uint8     Rover Firmware Patch Version (e.g. 6 for v0.8.6)
46..47  crc16                uint16    CRC16-CCITT across bytes 0..45
```

### 9.3 ANT2 Moving-Baseline Semantics
To prevent operator confusion between ANT1's base fix and ANT2's moving-baseline heading lock:
* **`SLV_FIX` (`4`):** Secondary antenna has integer carrier phase lock relative to ANT1 (`heading_source == HDG_SRC_DUAL_FIX`).
* **`SLV_FLT` (`5`):** Secondary antenna has float carrier phase lock relative to ANT1 (`heading_source == HDG_SRC_DUAL_FLT`).
* **`NONE` (`0`):** Secondary antenna baseline is rejected (e.g. sanity check failure `BL: 231.5cm`) or heading is operating in inertial/displacement fallback (`[HOLD]`, `[WALK]`, `[GYRO]`).
* Standalone `$GPGGAH` base fix quality is decoupled and does not override moving-baseline status.

### 9.4 FreeRTOS Dual-Core State Synchronization
* **Core 0 (`gnss_nav_task`):** Handles raw UART streaming, RTCM forwarding, and E220 half-duplex transmission.
* **Core 1 (`loopTask`):** Samples PMIC battery voltage, polls 6-DOF IMU, and executes `nav_heading_displacement_update()`.
* **Synchronization Rule:** `e220_feed_telemetry_state(&ui_state)` must be invoked on **Core 1** after sensor fusion, writing to a `portMUX_TYPE` spinlock-protected buffer snapshot. Feeding from Core 0 results in zero battery data and lost fallback heading tags.

---

## 10. Touchscreen Remote Air-Rate Switching Protocol

The system supports zero-credential, GPS-synchronized 3-rate switching (`19.2k -> 9.6k -> 4.8k -> 19.2k`) between Rover and Base:

```mermaid
sequenceDiagram
    autonumber
    participant Rover as Rover UI (ESP32-S3)
    participant Base as Base Pacer (Raspberry Pi)

    Note over Rover: User taps [LORA] to cycle 19.2k / 9.6k / 4.8k<br/>Pre-TX queue allows rapid cycling or cancel
    Rover->>Base: CMD_REQ_RADIO_SWITCH (10B, Second 0 Uplink, req_id)
    Note over Base: Schedules 4-ACK train@T0+1, Switch@T0+2<br/>Holds RTCM downlink during Second 1
    Base->>Rover: ACK_RADIO_SWITCH #1..#4 @ T0+1 80/260/440/620ms
    Note over Rover: Superframe-aligned switch at Second 2 (T0+2)
    Note over Base: At T0+2 0ms: Reconfigures E220 Mode 3 registers
    Note over Rover: At T0+2 0ms: Reconfigures E220 Mode 3 registers
    Note over Rover,Base: Both arm 60s rollback watchdog until first valid post-switch traffic
```

### 10.1 Binary Frame Formats
* **Request (`CMD_REQ_RADIO_SWITCH`, 10 Bytes):**
  `0xAA 0x55 | 0x02 | len=0x04 | target_rate_code(1) | target_channel(1) | req_id(2) | crc16(2)`
  * Rate codes: `0x02`=2.4k, `0x03`=4.8k, `0x04`=9.6k, `0x05`=19.2k, `0x06`=38.4k, `0x07`=62.5k.
* **Acknowledge (`ACK_RADIO_SWITCH`, 11 Bytes):**
  `0xAA 0x55 | 0x82 | len=0x05 | req_id(2) | status=0x00 | exec_delay_ms(2, 1000ms) | crc16(2)`

### 10.2 Dynamic Pre-Switch Fallback & Automatic Rollback Watchdog
Both Base and Rover arm a **60-second rollback watchdog** upon switching registers at Second 2 (`s_rate_switch_done_ms` on Rover). Because the UI supports 3 air rates (`19.2k / 9.6k / 4.8k`), both nodes record the **pre-switch verified working air rate** (`s_fallback_air_rate` on Rover, `self.fallback_air_rate` on Base) immediately before executing a switch (and refresh it once post-switch traffic is verified). The Rover watchdog disarms upon receiving a valid `CRC24Q` RTCM frame strictly *after* the switch timestamp; the Base disarms upon receiving valid rover telemetry after `watchdog_rate_switch_time`. Otherwise, both nodes revert cleanly to their recorded pre-switch fallback rate (e.g., reverting `4.8k` if `4.8k -> 19.2k` fails, instead of hardcoding `19.2k`).

---

## 11. Base Station Driver Architecture (`e220_pacer.py v3.7.0`)

### 11.1 Key Daemon Responsibilities
1. **Asynchronous Ping-Pong Mailboxes & Coalescing Engine (`e220_pacer.py v3.7.0`):** Ingests TCP stream (`127.0.0.1:5016`) on a background thread into latest-value mailboxes (`2.5 s` dynamic / `15.0 s` static TTL), packs compatible small frames (`<= 200 B`) together, and paces TX with Stage-1-calibrated `ToA + 30/35 ms` guards plus `AUX` edge supervision for `201..400 B` multi-subpacket frames.
2. **Content-Independent Consolidated Macro-Cycle & Starvation-Fair Round Robin:** Keeps Seconds `1..(macro_cycle_s - 1)` `100%` unbroken downlink (`--macro-cycle-s`, default `5`) and enforces Second 0 cutoffs (`450 / 620 / 680 ms`) while rotating constellations cleanly by starvation age (`now - last_tx` with `1074` tie-breaker).
3. **Telemetry Processor & Storage:**
   * **Terminal Heartbeat:** Outputs clean 7-bit ASCII lines (using `deg` units):
     ```text
     [ROVER TEL #042] FW:v0.8.12 | ANT1:RTK_FIX (28M) | ANT2:SLV_FIX (26S, BL:55.4cm, P:+0.2deg) | Hdg:142.6deg [DUAL_FIX] | Bat:85% (4.02V) | RSSI:-84dBm | Pos:43.7319105, -70.6237371, 12.45m
     ```
   * **RAM State (`/run/rtk_rover_status.json`):** Atomic tmpfs writes in RAM (zero MicroSD card wear) for local web dashboard integration.
   * **Rolling Daily CSV (`~/rover_logs/rover_YYYY-MM-DD.csv`):** Auto-detects interactive user home directories in `/home/` when running under systemd `User=root`, creates directories with `0777` permissions, compresses yesterday's logs to `.csv.gz`, and enforces 30-day retention with a 50 MB hard FIFO cap.

---

## 12. Store-and-Verify Sliding-Window Gatekeeper Pattern

Never pass raw E220 UART output directly to a downstream GNSS receiver or parser:
1. In fringe conditions, Subpacket 1 (`200 B`) may pass LoRa CRC while Subpacket 2 (`140 B`) drops. Raw passthrough causes downstream parsers to swallow valid bytes from subsequent frames, destroying intact data.
2. **Implementation ([`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp)):**
   * Stage incoming bytes in a 1,536-byte ring buffer.
   * Validate preambles (`0xD3` for RTCM3, `0xAA 0x55` for telemetry).
   * Enforce rate-adaptive truncation timeouts (`280 ms` @ 19.2k, `380 ms` @ 9.6k, `480 ms` @ 4.8k).
   * Verify CRC (`CRC24Q` for RTCM3, `CRC16` for telemetry).
   * Forward only 100% complete, CRC-passing frames atomically to the GNSS receiver. On failure, shift by 1 byte to rescue following frames.

---

## 13. Rapid Field Troubleshooting Matrix

| Observed Symptom | Root Cause | Verification & Fix |
| :--- | :--- | :--- |
| **`E220 READ OK` at boot, strong RSSI, but `0` frames `> 200 B` succeed; packets arrive `>= 950 ms` apart.** | **Open `M0` connection.** Internal pull-up holds `M0 = 1`. Radio enters Mode 1 (500 ms WOR sleep) instead of Mode 0. | Measure `M0` pin voltage during run: if `3.3 V` instead of `0.0 V`, repair connection. |
| **Pacer stalls 500 ms on every small (`<= 200 B`) frame; backlog explodes.** | Waiting for `AUX` to fall `LOW` on `<= 200 B` writes. Single packets load into `LLCC68` in `< 0.5 ms` without holding `AUX` `LOW`. | Pace `<= 200 B` frames by `ToA(N) + 35 ms` without blocking on `AUX` falling edge (Section 5.2). |
| **`201..400 B` frames lose Subpacket 2 when followed by another frame.** | Sending next frame immediately when `AUX` rises `HIGH` (`AUX` rises when Subpacket 2 *starts* airing, not when it ends). | After `AUX` rises `HIGH`, sleep `ToA(N - 200) + 35 ms` before writing the next frame (Section 5.3). |
| **RTCM `1074` (`201..253 B`) loses Subpacket 2 (`[RTCM DROP TRUNC] 200B`) at `4.8k` or `9.6k` followed by missing `1094` head.** | At `<= 9.6 kbps`, E220 raises `AUX` `HIGH` `100-147 ms` *before* Subpacket 1 (`200 B`) finishes airing, causing `send_packet()` to transmit the next frame while Subpacket 2 is still on air. | Upgrade to `e220_pacer.py v3.6.1+` enforcing `max(ToA(tail), ToA(200)+ToA(tail)-elapsed) + Guard` (Section 5.3). |
| **Mode 0 RSSI query at `4.8k` pulls `AUX` `LOW` for `100-130 ms` and replies after `166 ms` (`[E220 RX BURST] 5 B`).** | `C0 C1 C2 C3 00 02` was written right after `48 B` uplink `flush()` while the single-subpacket TX was still airing on `LLCC68`. | Track `s_tx_air_busy_until_ms` after uplink TX and use `180 ms` RSSI timeout (Section 5.2 & 7.3). |
| **`[E220 WRITE FAIL] got 3 bytes` (`FF FF FF`) or `got 4 bytes` on rate switch, followed immediately by `E220 READ OK`.** | UART switched to `9600` baud *before* raising `M0/M1` (or `M0/M1` bounced at `9600`), injecting a framing glitch into Mode 3 parser. | Upgrade to `v0.8.9+` + `e220_pacer.py v3.6.2+` (asserts `M0/M1=1` before `9600` baud switch, runs `0xC1` sync read, avoids bounce on retries `1..3`). |
| **`[E220 RX BURST] 5 B` every ~3 s + `Orph:+25 B` + `Noise:--dBm` on cold boot before 1st RF packet (`test/_048`).** | Prior to the 1st RF packet since boot, E220 replies `C1 00 02 <noise> 00` (`pkt_raw = 0x00`), which earlier parsers rejected (`pkt_raw < 0x80`). Also check Mode 3 exit ordering (`M0/M1=0` before restoring `115200`). | Upgrade to **`v0.8.12`** (accepts cold-boot `pkt_raw == 0x00` while requiring real packets for `rssi_valid`, exits Mode 3 via `e220_exit_mode3_to_mode0()` + `5 ms` Mode 0 liveness probe, and consolidates RSSI to `Second 0`). |
| **Stale epoch lag or `[WARN] Shed` during `Max-Sat` / `4.8k` operation.** | Synchronous single-thread FIFO queuing falls behind when `1 Hz` epoch airtime exceeds `1000 ms`, or per-second `850 ms` RSSI stops waste bandwidth. | Upgrade to **`e220_pacer.py v3.7.0` + `v0.8.12`** (Arch 2B async ping-pong mailboxes, `<= 200 B` coalescing, Stage-1 `ToA`, consolidated `Second 0` macro-cycle, `280 / 380 / 480 ms` trunc timeouts). |
| **`9.6k -> 4.8k` switch fails (`ACK #1 @ 0 ms` missed, single backup drops) or `[E220 RX BURST] 5 B` spills across `.000 s`.** | Base `0 ms` ACK hits NMEA-lagged Rover `Second 0` turnaround; `65 ms` RSSI timeout is shorter than `70-85 ms` E220 reply; `35 ms` `Regime A` guard under-paces at `<= 9.6k`. | Upgrade to `v0.8.11+` + `e220_pacer.py v3.6.5+` (`80/260/440/620 ms` 4-ACK `Second 1` train, `95 ms` / `.990 s` RSSI gate). |
| **Telemetry shows `Bat:0% (0.00V)` and `Hdg: 0.0deg [NONE]` while Rover screen is valid.** | Telemetry state fed from Core 0 before Core 1 attaches PMIC battery and woods fallback heading. | Move `e220_feed_telemetry_state(&ui_state)` to Core 1 post-sensor-fusion (Section 9.4). |
| **Telemetry shows `ANT2:SLV_FIX` while Rover header says `HDG HLD`.** | `ant2_fix_type` populated from standalone `$GPGGAH` instead of moving baseline. | Align `ant2_fix_type` strictly with `heading_source` (Section 9.3). |
| **`Â°` characters appear in console logs.** | Non-UTF-8 terminal interpreting 2-byte UTF-8 degree symbol (`0xC2 0xB0`). | Use 7-bit ASCII `deg` in scripts and firmware logging (Section 11.1). |
| **`ls ~/rover_logs` returns `No such file or directory` on Base Pi.** | Service runs as root; logs created in `/root/rover_logs`. | Update to `e220_pacer.py v3.3.1+` (auto-detects user `/home` and links directory). |
