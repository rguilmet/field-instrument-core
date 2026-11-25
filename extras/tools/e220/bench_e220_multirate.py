#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: bench_e220_multirate.py
Description: Stage 1 Standalone Hardware-in-the-Loop Multi-Rate E220 Silicon & Driver
             Bench Characterization Suite (4.8k / 9.6k / 19.2k kbps).

             Connects directly to:
               - Radio A (Base Pi GPIO): /dev/ttyAMA0 with BCM GPIO M0=18, M1=17, AUX=27, RST=22
               - Radio B (FTDI USB):     /dev/ttyUSB0 with DTR=M0, RTS=M1, CTS#=AUX (or --self-test sim)

             Safely checks/stops e220-pacer.service on startup (--stop-service) and restarts
             it automatically in finally: on exit or Ctrl+C.

             Runs up to 6 independent empirical characterization tests:
               1. toa         : Empirical ToA & AUX waveform sweep (18 B .. 400 B + 456 B split)
               2. guard       : Inter-frame guard sweep (60..10ms) & AUX-only vs ToA-floor vs Coalesce
               3. per         : Framing PER comparison (1-pkt vs 2-pkt vs <=200B coalesced vs 200B slice)
               4. uplink_rssi : Consolidated uplink + Mode 0 C0 C1 C2 C3 RSSI turnaround margin sweep
               5. switch      : Automated Mode 3 3-rate switching endurance loop (C2 RAM default)
               6. thermal     : Sustained high-duty (85-90%) soak for passive XTAL thermal drift tracking

Hardware Safety Warning:
  Power the FTDI-connected E220-900T30S/D from clean +5.0V VBUS with >=100-470uF + 100nF
  local decoupling capacitors (NEVER from an FTDI onboard 50mA 3.3V regulator). Keep TX/RX
  and DTR/RTS/CTS signals at 3.3V CMOS logic levels.

Author: Roland L. Guilmet & Antigravity
License: MIT
"""

from __future__ import annotations

import argparse
import csv
import errno
import os
import signal
import statistics
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    import termios
    _TERMIOS_ERRORS: tuple[type[BaseException], ...] = (termios.error,)
except ImportError:
    _TERMIOS_ERRORS = ()


__filename__ = os.path.basename(__file__)
__app_name__ = "bench_e220_multirate"
__version__ = "v1.0.3"
__date__ = "2026-11-16"


# E220-900T30S Register & Pin Maps (matching e220_pacer.py v3.6.5 & setup_e220.py)
DEFAULT_TX_PORT = "/dev/ttyAMA0"
DEFAULT_RX_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
CONFIG_BAUD = 9600
DEFAULT_CHANNEL = 75
DEFAULT_POWER_DBM = 21
DEFAULT_RATES: tuple[float, ...] = (4.8, 9.6, 19.2)

PIN_M0 = 18
PIN_M1 = 17
PIN_AUX = 27
PIN_RST = 22

BAUD_BITS = {1200: 0, 2400: 1, 4800: 2, 9600: 3, 19200: 4, 38400: 5, 57600: 6, 115200: 7}
AIR_RATE_BITS = {2.4: 2, 4.8: 3, 9.6: 4, 19.2: 5, 38.4: 6, 62.5: 7}
POWER_BITS = {30: 0, 27: 1, 24: 2, 21: 3}
PKT_SIZE_BITS = {200: 0, 128: 1, 64: 2, 32: 3}

INV_AIR_RATE = {0: 2.4, 1: 2.4, **{v: k for k, v in AIR_RATE_BITS.items()}}
INV_POWER = {v: k for k, v in POWER_BITS.items()}
INV_PKT_SIZE = {v: k for k, v in PKT_SIZE_BITS.items()}

ROVER_SYNC = b"\xAA\x55"
MSG_ID_TELEMETRY = 0x01
FMT_TELEMETRY = "<2sBBddfHBBHhBBBHBHbbBBBBH"  # 48B V3 format

ALL_TESTS = ("toa", "guard", "per", "uplink_rssi", "switch", "thermal")
g_abort_requested = False


def _signal_handler(signum: int, _frame: object) -> None:
    """Handle SIGINT/SIGTERM gracefully so hardware & systemd state always unwind cleanly."""
    global g_abort_requested
    g_abort_requested = True
    print(f"\n[SIGNAL] Caught signal {signum}; unwinding hardware & service state...", flush=True)


def rtcm_crc24q(data: bytes) -> int:
    """Compute RTCM3 24-bit CRC24Q across full frame (0 indicates intact frame)."""
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC16-CCITT (False: poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc


def estimate_subpacket_air_s(n_bytes: int, air_rate_kbps: float) -> float:
    """
    Return silicon-calibrated single-subpacket Time-on-Air (1..200 B, seconds)
    fitted to Pi FTDI-1ms empirical E220-900T30S measurements (test/_045.._047,
    max err <= 1.9 ms at 4.8k/9.6k/19.2k; conservative fallback at <= 3.0k or > 20k).
    """
    if n_bytes <= 0:
        return 0.0
    n = float(min(n_bytes, 200))
    if air_rate_kbps <= 3.0:
        rate_bps = max(air_rate_kbps, 2.4) * 1000.0
        return 0.022 + ((n + 16.0) * 8.0) / (rate_bps * 0.60)
    if air_rate_kbps <= 5.0:
        return 0.070 + (0.00187 * n)
    if air_rate_kbps <= 10.0:
        return 0.045 + (0.00119 * n)
    if air_rate_kbps <= 20.0:
        return 0.029 + (0.00080 * n)
    return 0.016 + ((n + 16.0) * 8.0) / (max(air_rate_kbps, 2.4) * 1000.0 * 0.68)


def estimate_frame_air_s(frame_len: int, air_rate_kbps: float) -> float:
    """
    Return silicon-calibrated end-to-end RF air + E220 framing duration (seconds)
    for 1..400 B single-burst frames (test/_045.._047 fit, max err <= 1.9 ms), or
    summed <=200 B chunk durations for >400 B frames or <=3.0 kbps.
    """
    if frame_len <= 0:
        return 0.0
    if frame_len <= 200:
        return estimate_subpacket_air_s(frame_len, air_rate_kbps)
    if frame_len <= 400 and air_rate_kbps > 3.0:
        n = float(frame_len)
        if air_rate_kbps <= 5.0:
            return 0.128 + (0.00180 * n)
        if air_rate_kbps <= 10.0:
            return 0.080 + (0.00112 * n)
        if air_rate_kbps <= 20.0:
            return 0.055 + (0.00070 * n)
    rem = frame_len
    tot = 0.0
    while rem > 0:
        chunk = min(rem, 200)
        tot += estimate_subpacket_air_s(chunk, air_rate_kbps)
        rem -= chunk
    return tot


def estimate_frame_toa_ms(frame_len: int, air_rate_kbps: float, guard_s: Optional[float] = None, baud: int = DEFAULT_BAUD) -> float:
    """Estimate complete UART + RF air time + guard delay for an arbitrary frame length (ms)."""
    if frame_len <= 0:
        return 0.0
    subpkts = (frame_len + 199) // 200
    air_s = estimate_frame_air_s(frame_len, air_rate_kbps)
    uart_s = (float(frame_len) * 10.0) / max(float(baud), 9600.0)
    if guard_s is None:
        if frame_len > 400:
            per_chunk_g = 0.052 if air_rate_kbps <= 9.6 else 0.040
            eff_guard = per_chunk_g * float(subpkts)
        elif subpkts >= 2:
            eff_guard = 0.050 if air_rate_kbps <= 5.0 else 0.048
            eff_guard += 0.008 * float(subpkts - 1)
        else:
            eff_guard = 0.052 if air_rate_kbps <= 9.6 else 0.040
    else:
        eff_guard = guard_s * float(subpkts if frame_len > 400 else 1)
    return (air_s + uart_s + eff_guard) * 1000.0


def build_synthetic_rtcm(msg_type: int, total_len: int, seq: int) -> bytes:
    """
    Build a CRC24Q-valid RTCM3-compliant bench frame of exact length total_len (12..1029 B).
    Embeds msg_type (12-bit) and seq (32-bit BE) inside payload bytes so receiver tracks loss.
    """
    if total_len < 12 or total_len > 1029:
        raise ValueError(f"Synthetic RTCM frame total_len must be 12..1029 B (got {total_len})")
    plen = total_len - 6
    hdr = bytes([0xD3, (plen >> 8) & 0x03, plen & 0xFF])
    payload = bytearray(plen)
    payload[0] = (msg_type >> 4) & 0xFF
    payload[1] = (msg_type & 0x0F) << 4
    # Embed 32-bit sequence ID safely inside payload[2..5] when plen >= 6
    struct.pack_into(">I", payload, 2, seq & 0xFFFFFFFF)
    for i in range(6, plen):
        payload[i] = ((seq * 37) + (i * 13) + (msg_type & 0xFF)) & 0xFF
    body = hdr + bytes(payload)
    crc = rtcm_crc24q(body)
    return body + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


def parse_synthetic_rtcm(buf: bytearray) -> tuple[list[tuple[int, int, int]], int, int]:
    """
    Sliding-window RTCM3 extractor matching e220_driver.cpp Gatekeeper rules.
    Mutates buf in place for consumed/discarded bytes; preserves trailing incomplete candidate.
    Returns ([(msg_type, total_len, seq), ...], crc_errors, orphan_bytes).
    """
    valid: list[tuple[int, int, int]] = []
    crc_errors = 0
    orphan_bytes = 0

    while len(buf) > 0:
        if buf[0] != 0xD3:
            sync_idx = buf.find(b"\xD3", 1)
            if sync_idx == -1:
                orphan_bytes += len(buf)
                buf.clear()
                break
            orphan_bytes += sync_idx
            del buf[:sync_idx]
            continue

        if len(buf) < 3:
            break
        if (buf[1] & 0xFC) != 0:
            orphan_bytes += 1
            del buf[:1]
            continue
        plen = ((buf[1] & 0x03) << 8) | buf[2]
        if plen < 6 or plen > 1023:
            orphan_bytes += 1
            del buf[:1]
            continue
        tot_len = plen + 6
        if len(buf) < tot_len:
            break

        cand = bytes(buf[:tot_len])
        if rtcm_crc24q(cand) == 0:
            msg_type = (cand[3] << 4) | (cand[4] >> 4)
            seq = struct.unpack_from(">I", cand, 5)[0]
            valid.append((msg_type, tot_len, seq))
            del buf[:tot_len]
        else:
            crc_errors += 1
            orphan_bytes += 1
            del buf[:1]

    return (valid, crc_errors, orphan_bytes)


def build_uplink_48b(seq: int) -> bytes:
    """Build a valid 48-byte RoverUplinkTelemetry_t V3 frame with CRC16-CCITT."""
    body = struct.pack(
        FMT_TELEMETRY,
        ROVER_SYNC,
        MSG_ID_TELEMETRY,
        42,
        43.73247750,
        -70.62401580,
        91.90,
        14,
        4,
        4,
        555,
        20,
        1,
        18,
        0xC0 | 14,
        14260,
        88,
        4020,
        -65,
        -99,
        seq & 0xFF,
        0,
        8,
        11,
        0,
    )
    crc = crc16_ccitt(body[:-2])
    return body[:-2] + struct.pack("<H", crc)


def verify_uplink_48b(buf: bytearray) -> tuple[list[int], int]:
    """Extract and CRC16-verify 48B V3 uplink packets from buffer; returns (seqs, crc_errors)."""
    seqs: list[int] = []
    crc_errors = 0
    while len(buf) > 0:
        idx = buf.find(ROVER_SYNC)
        if idx == -1:
            if len(buf) > 1 and buf[-1] == 0xAA:
                del buf[:-1]
            else:
                buf.clear()
            break
        if idx > 0:
            del buf[:idx]
        if len(buf) < 48:
            break
        if buf[2] != MSG_ID_TELEMETRY or buf[3] != 42:
            del buf[:2]
            continue
        frame = bytes(buf[:48])
        rx_crc = struct.unpack_from("<H", frame, 46)[0]
        if crc16_ccitt(frame[:46]) == rx_crc:
            seqs.append(frame[42])
            del buf[:48]
        else:
            crc_errors += 1
            del buf[:2]
    return (seqs, crc_errors)


def parse_mode0_rssi_reply(raw: bytes) -> tuple[bool, int, int, str]:
    """
    Parse E220 Mode 0 RSSI query reply (C0 C1 C2 C3 00 02):
      - 4-byte E220-900T30S reply: C1 00 <noise> <pkt>
      - 5-byte datasheet reply:    C1 00 02 <noise> <pkt>
    Returns (ok, noise_dbm, pkt_dbm, note).
    """
    n = len(raw)
    for i in range(max(0, n - 3)):
        if raw[i] == 0xC1 and raw[i + 1] == 0x00:
            if i + 5 <= n and raw[i + 2] == 0x02:
                n_raw, p_raw = raw[i + 3], raw[i + 4]
                fmt = "5B"
            elif i + 4 <= n and 0x80 <= raw[i + 2] <= 0xE4:
                n_raw, p_raw = raw[i + 2], raw[i + 3]
                fmt = "4B"
            else:
                continue
            if n_raw == 0x80:
                return (False, -128, int(p_raw) - 256, "0x80_sentinel")
            if 0x81 <= n_raw <= 0xE4 and 0x80 <= p_raw <= 0xF6:
                return (True, int(n_raw) - 256, int(p_raw) - 256, fmt)
    return (False, 0, 0, f"no_hdr(len={n})")


def build_reg_bytes(
    air_rate_kbps: float = 19.2,
    power_dbm: int = 21,
    channel: int = 75,
    baud: int = DEFAULT_BAUD,
    packet_size: int = 200,
    rssi_reg: bool = True,
) -> bytes:
    """Build 6 core E220 registers (ADDH..REG3) with REG1 bit 5 Mode 0 RSSI enabled."""
    reg0 = (BAUD_BITS.get(baud, 7) << 5) | AIR_RATE_BITS.get(air_rate_kbps, 5)
    reg1 = (PKT_SIZE_BITS.get(packet_size, 0) << 6) | (0x20 if rssi_reg else 0x00) | POWER_BITS.get(power_dbm, 3)
    chan = max(0, min(83, int(channel)))
    reg3 = 0x00
    return bytes([0x00, 0x00, reg0, reg1, chan, reg3])


def decode_reg_str(regs: bytes) -> str:
    """Return compact ASCII summary of 6+ E220 config registers."""
    if len(regs) < 6:
        return f"INVALID(len={len(regs)})"
    addh, addl, r0, r1, ch, r3 = regs[:6]
    air = INV_AIR_RATE.get(r0 & 0x07, 0.0)
    pkt = INV_PKT_SIZE.get((r1 >> 6) & 0x03, 0)
    pwr = INV_POWER.get(r1 & 0x03, 0)
    rssi_reg = "ON" if (r1 & 0x20) else "OFF"
    return (
        f"ADD={addh:02X}{addl:02X} REG={r0:02X}/{r1:02X}/{ch:02X}/{r3:02X} "
        f"(Air={air}k Pkt={pkt}B Pwr={pwr}dBm CH={ch}({850.125+ch:.3f}MHz) RSSI_Reg={rssi_reg})"
    )


class ServiceGuard:
    """Pause e220-pacer.service if active during hardware testing and always restore on exit."""

    def __init__(self, stop_service: bool, leave_stopped: bool, sim_mode: bool = False):
        self.stop_service = stop_service
        self.leave_stopped = leave_stopped
        self.sim_mode = sim_mode
        self.was_active = False

    @staticmethod
    def is_active() -> bool:
        if os.name == "nt":
            return False
        res = subprocess.run(["systemctl", "is-active", "e220-pacer"], capture_output=True, text=True, check=False)
        return res.returncode == 0 and res.stdout.strip() == "active"

    @staticmethod
    def _systemctl(action: str) -> subprocess.CompletedProcess[str]:
        cmd = ["systemctl", action, "e220-pacer"] if os.geteuid() == 0 else ["sudo", "systemctl", action, "e220-pacer"]
        return subprocess.run(cmd, capture_output=True, text=True, check=False)

    def acquire(self) -> None:
        if self.sim_mode:
            print("[SERVICE] Simulation mode (--self-test): systemd check skipped.", flush=True)
            return
        if os.name == "nt":
            raise RuntimeError("Real serial/GPIO bench mode requires Linux/Raspberry Pi. Use --self-test on Windows.")
        if self.is_active():
            if not self.stop_service:
                raise RuntimeError(
                    "e220-pacer.service is ACTIVE and holds /dev/ttyAMA0 + GPIO 17/18/22/27. "
                    "Pass --stop-service (recommended) or stop it manually first."
                )
            print("[SERVICE] Stopping e220-pacer.service before acquiring /dev/ttyAMA0...", flush=True)
            res = self._systemctl("stop")
            if res.returncode != 0:
                raise RuntimeError(f"Failed to stop e220-pacer.service: {res.stderr.strip()}")
            t0 = time.monotonic()
            while (time.monotonic() - t0) < 3.0 and self.is_active():
                time.sleep(0.05)
            self.was_active = True
            print("[SERVICE] e220-pacer.service stopped cleanly.", flush=True)
        else:
            print("[SERVICE] e220-pacer.service already inactive.", flush=True)

    def release(self) -> None:
        if self.sim_mode or not self.was_active:
            return
        if self.leave_stopped:
            print("[SERVICE] --leave-stopped set: leaving e220-pacer.service stopped.", flush=True)
            return
        print("[SERVICE] Restoring e220-pacer.service...", flush=True)
        res = self._systemctl("start")
        state = "active" if self.is_active() else f"error({res.returncode})"
        print(f"[SERVICE] e220-pacer.service state: {state}", flush=True)


class BaseE220Radio:
    """Unified hardware interface for Pi GPIO, FTDI USB, and virtual Self-Test E220 nodes."""

    def __init__(self, name: str, port: str, baud: int = DEFAULT_BAUD):
        self.name = name
        self.port_name = port
        self.baud = baud
        self.air_rate_kbps = 19.2
        self.power_dbm = 21
        self.channel = DEFAULT_CHANNEL
        self.aux_resets = 0
        self.initial_regs: Optional[bytes] = None
        self.supports_mode_switch = True
        self.time_scale = 1.0

    def _sleep(self, real_s: float) -> None:
        if real_s > 0.0:
            time.sleep(real_s * self.time_scale)

    def _poll_interval(self, real_s: float) -> float:
        return max(0.0001, real_s * min(1.0, self.time_scale))

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def aux_is_high(self) -> bool:
        raise NotImplementedError

    def set_mode_pins(self, m0: int, m1: int) -> None:
        raise NotImplementedError

    def set_baudrate(self, baud: int) -> None:
        raise NotImplementedError

    def hardware_reset(self) -> bool:
        return False

    def write_bytes(self, data: bytes) -> None:
        raise NotImplementedError

    def flush_tx(self) -> None:
        raise NotImplementedError

    def drain_rx(self) -> bytes:
        raise NotImplementedError

    def reset_buffers(self) -> None:
        raise NotImplementedError

    def wait_aux_high(self, timeout_s: float = 1.5, guard_ms: float = 2.0) -> bool:
        t0 = time.monotonic()
        deadline = t0 + (timeout_s * max(0.05, self.time_scale if self.time_scale >= 1.0 else 0.1))
        while time.monotonic() < deadline:
            if self.aux_is_high():
                if guard_ms > 0:
                    self._sleep(guard_ms / 1000.0)
                return True
            time.sleep(self._poll_interval(0.0004))
        return False

    def configure_registers(
        self,
        air_rate_kbps: float,
        power_dbm: int,
        channel: int,
        permanent: bool = False,
        verify_read: bool = False,
    ) -> tuple[bool, int, str]:
        """
        Safely switch E220 into Mode 3 (M0=1, M1=1 BEFORE 9600 baud!), verify 0xC1 alignment,
        program target registers via 0xC2 (RAM default) or 0xC0 (EEPROM), and return to Mode 0.
        Returns (ok, attempts_used, detail_str).
        """
        if not self.supports_mode_switch:
            self.air_rate_kbps = air_rate_kbps
            self.power_dbm = power_dbm
            self.channel = channel
            return (True, 0, "skipped_no_modem_lines")

        target_6 = build_reg_bytes(air_rate_kbps, power_dbm, channel, self.baud, 200, True)
        cmd_op = 0xC0 if permanent else 0xC2

        self.wait_aux_high(1.0, 2.0)
        self.flush_tx()
        # Critical rule (docs/e220-reference.md Sec 4.1): assert M0=1, M1=1 BEFORE switching baud to 9600
        self.set_mode_pins(1, 1)
        self.set_baudrate(CONFIG_BAUD)
        self._sleep(0.065)
        self.wait_aux_high(1.0, 5.0)
        self.drain_rx()

        sync_ok = self._mode3_read_6() is not None
        ok = False
        used_attempts = 0
        last_hex = "none"

        for attempt in range(1, 5):
            used_attempts = attempt
            if attempt == 4:
                # Last-resort clean Mode 0 -> Mode 3 bounce with matched bauds
                self.set_mode_pins(0, 0)
                self.set_baudrate(self.baud)
                self._sleep(0.035)
                self.drain_rx()
                self.set_mode_pins(1, 1)
                self.set_baudrate(CONFIG_BAUD)
                self._sleep(0.065)
                self.drain_rx()
                self._mode3_read_6()
            elif attempt > 1:
                self._sleep(0.035)
            else:
                self._sleep(0.015)

            self.drain_rx()
            self.write_bytes(bytes([cmd_op, 0x00, 0x06]) + target_6)
            self.flush_tx()

            raw = self._read_mode3_frame(expected_len=9, timeout_s=0.30)
            last_hex = raw.hex(" ").upper() if raw else "none"
            for k in range(max(0, len(raw) - 8)):
                if raw[k] in (0xC0, 0xC1, 0xC2) and raw[k + 1] == 0x00 and raw[k + 2] == 0x06:
                    if raw[k + 3 : k + 9] == target_6:
                        ok = True
                        break
            if ok:
                break

        if ok and verify_read:
            # Guard >=20ms between Mode 3 C2 write echo and follow-up C1 readback
            self._sleep(0.020)
            rb = self._mode3_read_6()
            if rb is None:
                self._sleep(0.025)
                rb = self._mode3_read_6()
            if rb != target_6:
                ok = False
                last_hex = f"verify_mismatch:{rb.hex() if rb else 'none'}"

        # Restore Mode 0 (M0=0, M1=0) -> Mode 3 exit settle -> restore baud -> RF synth settle (>= 80ms)
        self.set_mode_pins(0, 0)
        self._sleep(0.025)
        self.set_baudrate(self.baud)
        self._sleep(0.055)
        self.wait_aux_high(1.0, 5.0)
        self.drain_rx()

        mode0_tag = "ok"
        if ok:
            # Verify live Mode 0 UART responsiveness (~5ms); rescues rare FTDI Mode 3->0 exit desyncs
            r_ok, _, _, _, r_note = self.query_mode0_rssi(timeout_s=0.12)
            if not (r_ok or r_note == "0x80_sentinel"):
                self.set_mode_pins(1, 1)
                self._sleep(0.040)
                self.set_mode_pins(0, 0)
                self._sleep(0.025)
                self.set_baudrate(self.baud)
                self._sleep(0.055)
                self.wait_aux_high(1.0, 5.0)
                self.drain_rx()
                r_ok2, _, _, _, r_note2 = self.query_mode0_rssi(timeout_s=0.12)
                if r_ok2 or r_note2 == "0x80_sentinel":
                    mode0_tag = "recovered"
                else:
                    ok = False
                    mode0_tag = f"FAIL({r_note2})"

        if ok:
            self.air_rate_kbps = air_rate_kbps
            self.power_dbm = power_dbm
            self.channel = channel
        return (ok, used_attempts, f"sync={sync_ok} op=0x{cmd_op:02X} raw={last_hex} m0={mode0_tag}")

    def read_registers_mode3(self) -> Optional[bytes]:
        """Enter Mode 3 cleanly, read 8 EEPROM/active registers via C1 00 08, return to Mode 0."""
        if not self.supports_mode_switch:
            return None
        self.wait_aux_high(1.0, 2.0)
        self.flush_tx()
        self.set_mode_pins(1, 1)
        self.set_baudrate(CONFIG_BAUD)
        self._sleep(0.065)
        self.wait_aux_high(1.0, 5.0)
        regs = self._mode3_read_8()
        self.set_mode_pins(0, 0)
        self.set_baudrate(self.baud)
        self._sleep(0.045)
        self.wait_aux_high(1.0, 5.0)
        self.drain_rx()
        return regs

    def _mode3_read_6(self) -> Optional[bytes]:
        self.drain_rx()
        self.write_bytes(bytes([0xC1, 0x00, 0x06]))
        self.flush_tx()
        raw = self._read_mode3_frame(9, 0.28)
        for k in range(max(0, len(raw) - 8)):
            if raw[k] == 0xC1 and raw[k + 1] == 0x00 and raw[k + 2] == 0x06:
                return raw[k + 3 : k + 9]
        return None

    def _mode3_read_8(self) -> Optional[bytes]:
        self.drain_rx()
        self.write_bytes(bytes([0xC1, 0x00, 0x08]))
        self.flush_tx()
        raw = self._read_mode3_frame(11, 0.30)
        for k in range(max(0, len(raw) - 10)):
            if raw[k] == 0xC1 and raw[k + 1] == 0x00 and raw[k + 2] == 0x08:
                return raw[k + 3 : k + 11]
        return None

    def _read_mode3_frame(self, expected_len: int, timeout_s: float) -> bytes:
        buf = bytearray()
        t0 = time.monotonic()
        last_rx = t0
        deadline = t0 + (timeout_s * max(0.05, self.time_scale))
        gap_s = 0.035 * max(0.05, self.time_scale)
        while time.monotonic() < deadline:
            chunk = self.drain_rx()
            if chunk:
                buf.extend(chunk)
                last_rx = time.monotonic()
                if len(buf) >= expected_len:
                    break
            else:
                if len(buf) >= 3 and (time.monotonic() - last_rx) >= gap_s:
                    break
                time.sleep(self._poll_interval(0.004))
        return bytes(buf)

    def send_paced_frame(
        self,
        data: bytes,
        mode: str = "toa_calibrated",
        guard_s: Optional[float] = None,
    ) -> dict[str, float | bool | int]:
        """
        Transmit a frame over E220 using:
          - toa_calibrated: Two-regime AUX + ToA floor pacing (matching e220_pacer.py v3.6.5)
          - aux_only:       Naive AUX==HIGH polling only (to expose why AUX-only overruns)
        Handles >400B cleanly when toa_calibrated is active by chunking into <=200B writes.
        All returned millisecond fields are in unscaled (1.0x real-radio equivalent) ms.
        """
        if not self.wait_aux_high(1.5, 2.0):
            if self.hardware_reset():
                self.aux_resets += 1

        if len(data) > 400 and mode == "toa_calibrated":
            t_start = time.monotonic()
            rem_off = 0
            tot_subs = 0
            any_drop = False
            while rem_off < len(data):
                part = data[rem_off : rem_off + 200]
                rem_off += len(part)
                sub_res = self.send_paced_frame(part, mode=mode, guard_s=guard_s)
                tot_subs += int(sub_res["subpkts"])
                any_drop = any_drop or bool(sub_res["aux_dropped"])
            return {
                "subpkts": tot_subs,
                "aux_dropped": any_drop,
                "aux_drop_ms": 0.0,
                "aux_low_ms": 0.0,
                "elapsed_ms": ((time.monotonic() - t_start) * 1000.0) / self.time_scale,
            }

        t0 = time.monotonic()
        self.write_bytes(data)
        self.flush_tx()
        t_flush = time.monotonic()

        n = len(data)
        subpkts = (n + 199) // 200
        aux_dropped = False
        aux_drop_ms = 0.0
        aux_low_ms = 0.0
        ts = self.time_scale

        if mode == "aux_only":
            probe_s = 0.015 * ts
            while (time.monotonic() - t_flush) < probe_s:
                if not self.aux_is_high():
                    aux_dropped = True
                    aux_drop_ms = ((time.monotonic() - t_flush) * 1000.0) / ts
                    break
                time.sleep(self._poll_interval(0.0003))
            if aux_dropped:
                t_low = time.monotonic()
                if not self.wait_aux_high(1.5, 2.0):
                    self.hardware_reset()
                    self.aux_resets += 1
                aux_low_ms = ((time.monotonic() - t_low) * 1000.0) / ts
            eff_g = 0.035 if guard_s is None else guard_s
            self._sleep(max(0.0, eff_g))
            return {
                "subpkts": subpkts,
                "aux_dropped": aux_dropped,
                "aux_drop_ms": aux_drop_ms,
                "aux_low_ms": aux_low_ms,
                "elapsed_ms": ((time.monotonic() - t0) * 1000.0) / ts,
            }

        # toa_calibrated mode (Regime A vs Regime B)
        if n <= 200:
            est_air_scaled = estimate_subpacket_air_s(n, self.air_rate_kbps) * ts
            probe_s = 0.008 * ts
            while (time.monotonic() - t_flush) < probe_s:
                if not self.aux_is_high():
                    aux_dropped = True
                    aux_drop_ms = ((time.monotonic() - t_flush) * 1000.0) / ts
                    break
                time.sleep(self._poll_interval(0.0003))
            if aux_dropped:
                t_low = time.monotonic()
                if not self.wait_aux_high(1.5, 2.0):
                    self.hardware_reset()
                    self.aux_resets += 1
                aux_low_ms = ((time.monotonic() - t_low) * 1000.0) / ts
            rem_scaled = est_air_scaled - (time.monotonic() - t_flush)
            if rem_scaled > 0:
                time.sleep(rem_scaled)
            default_g = 0.052 if self.air_rate_kbps <= 9.6 else 0.040
            eff_g = default_g if guard_s is None else guard_s
            self._sleep(max(0.0, eff_g))
        else:
            full_heads = subpkts - 1
            tail_len = n - (full_heads * 200)
            total_air_scaled = estimate_frame_air_s(n, self.air_rate_kbps) * ts
            tail_air_scaled = estimate_subpacket_air_s(tail_len, self.air_rate_kbps) * ts
            probe_s = 0.050 * ts
            while (time.monotonic() - t_flush) < probe_s:
                if not self.aux_is_high():
                    aux_dropped = True
                    aux_drop_ms = ((time.monotonic() - t_flush) * 1000.0) / ts
                    break
                time.sleep(self._poll_interval(0.0003))
            if aux_dropped:
                t_low = time.monotonic()
                if not self.wait_aux_high(1.5, 2.0):
                    self.hardware_reset()
                    self.aux_resets += 1
                aux_low_ms = ((time.monotonic() - t_low) * 1000.0) / ts
                elapsed_scaled = time.monotonic() - t_flush
                rem_scaled = max(tail_air_scaled, total_air_scaled - elapsed_scaled)
                if rem_scaled > 0:
                    time.sleep(rem_scaled)
            else:
                elapsed_scaled = time.monotonic() - t_flush
                rem_scaled = max(0.0, total_air_scaled - elapsed_scaled)
                if rem_scaled > 0:
                    time.sleep(rem_scaled)
            default_g = (0.050 if self.air_rate_kbps <= 5.0 else 0.048) + 0.008 * float(full_heads)
            eff_g = default_g if guard_s is None else guard_s
            self._sleep(max(0.0, eff_g))

        return {
            "subpkts": subpkts,
            "aux_dropped": aux_dropped,
            "aux_drop_ms": aux_drop_ms,
            "aux_low_ms": aux_low_ms,
            "elapsed_ms": ((time.monotonic() - t0) * 1000.0) / ts,
        }

    def query_mode0_rssi(self, timeout_s: float = 0.25) -> tuple[bool, int, int, float, str]:
        """Issue Mode 0 C0 C1 C2 C3 00 02 query and measure latency + noise/pkt dBm."""
        self.drain_rx()
        t0 = time.monotonic()
        self.write_bytes(bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02]))
        self.flush_tx()
        buf = bytearray()
        deadline = t0 + (timeout_s * max(0.2, self.time_scale))
        while time.monotonic() < deadline:
            chunk = self.drain_rx()
            if chunk:
                buf.extend(chunk)
                ok, n_dbm, p_dbm, note = parse_mode0_rssi_reply(bytes(buf))
                if ok or note == "0x80_sentinel":
                    return (ok, n_dbm, p_dbm, ((time.monotonic() - t0) * 1000.0) / self.time_scale, note)
            time.sleep(self._poll_interval(0.002))
        return (False, 0, 0, ((time.monotonic() - t0) * 1000.0) / self.time_scale, f"timeout(rx={buf.hex()})")


class PiGpioRadio(BaseE220Radio):
    """Raspberry Pi /dev/ttyAMA0 + BCM GPIO E220 driver."""

    def __init__(self, port: str = DEFAULT_TX_PORT, baud: int = DEFAULT_BAUD, m0: int = PIN_M0, m1: int = PIN_M1, aux: int = PIN_AUX, rst: int = PIN_RST):
        super().__init__("Base-GPIO", port, baud)
        self.pin_m0, self.pin_m1, self.pin_aux, self.pin_rst = m0, m1, aux, rst
        self.ser: Optional[serial.Serial] = None
        self.gpio_ready = False

    def open(self) -> None:
        if serial is None or GPIO is None:
            raise RuntimeError("PiGpioRadio requires pyserial and RPi.GPIO.")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in (self.pin_m0, self.pin_m1, self.pin_rst):
            GPIO.setup(p, GPIO.OUT)
        GPIO.setup(self.pin_aux, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.output(self.pin_rst, GPIO.HIGH)
        GPIO.output(self.pin_m0, GPIO.LOW)
        GPIO.output(self.pin_m1, GPIO.LOW)
        self.gpio_ready = True
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.02, exclusive=True)
        except TypeError:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.02)
        time.sleep(0.04)
        if not self.wait_aux_high(1.0, 5.0):
            self.hardware_reset()
        self.reset_buffers()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        if self.gpio_ready and GPIO is not None:
            try:
                GPIO.output(self.pin_m0, GPIO.LOW)
                GPIO.output(self.pin_m1, GPIO.LOW)
                GPIO.cleanup((self.pin_m0, self.pin_m1, self.pin_aux, self.pin_rst))
            except Exception:
                pass
            self.gpio_ready = False

    def aux_is_high(self) -> bool:
        return bool(GPIO.input(self.pin_aux) == GPIO.HIGH)

    def set_mode_pins(self, m0: int, m1: int) -> None:
        GPIO.output(self.pin_m0, GPIO.HIGH if m0 else GPIO.LOW)
        GPIO.output(self.pin_m1, GPIO.HIGH if m1 else GPIO.LOW)

    def set_baudrate(self, baud: int) -> None:
        if self.ser:
            self.ser.baudrate = baud

    def hardware_reset(self) -> bool:
        if not self.gpio_ready or GPIO is None:
            return False
        GPIO.output(self.pin_m0, GPIO.LOW)
        GPIO.output(self.pin_m1, GPIO.LOW)
        GPIO.output(self.pin_rst, GPIO.LOW)
        time.sleep(0.055)
        GPIO.output(self.pin_rst, GPIO.HIGH)
        time.sleep(0.100)
        return self.wait_aux_high(2.0, 10.0)

    def write_bytes(self, data: bytes) -> None:
        if self.ser:
            self.ser.write(data)

    def flush_tx(self) -> None:
        if not self.ser:
            return
        while True:
            try:
                self.ser.flush()
                return
            except _TERMIOS_ERRORS + (OSError,) as e:
                if getattr(e, "args", [None])[0] == errno.EINTR and not g_abort_requested:
                    continue
                raise

    def drain_rx(self) -> bytes:
        if not self.ser:
            return b""
        avail = self.ser.in_waiting
        if avail > 0:
            return self.ser.read(avail)
        return b""

    def reset_buffers(self) -> None:
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()


class FtdiUsbRadio(BaseE220Radio):
    """USB-FTDI E220 driver using DTR=M0, RTS=M1, CTS#=AUX (matches setup_e220.py)."""

    def __init__(self, port: str = DEFAULT_RX_PORT, baud: int = DEFAULT_BAUD, no_modem_lines: bool = False):
        super().__init__("FTDI-USB", port, baud)
        self.ser: Optional[serial.Serial] = None
        self.supports_mode_switch = not no_modem_lines
        self._latency_path: Optional[str] = None
        self._orig_latency: Optional[str] = None

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("FtdiUsbRadio requires pyserial.")
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.02, exclusive=True)
        except TypeError:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.02)
        if self.supports_mode_switch:
            self.ser.dtr = True  # M0 = 0 (Mode 0)
            self.ser.rts = True  # M1 = 0 (Mode 0)
        self._tune_ftdi_latency_timer()
        time.sleep(0.06)
        self.reset_buffers()

    def _tune_ftdi_latency_timer(self) -> None:
        base_dev = os.path.basename(os.path.realpath(self.port_name))
        lat_path = f"/sys/bus/usb-serial/devices/{base_dev}/latency_timer"
        if not os.path.exists(lat_path):
            return
        try:
            with open(lat_path, "r", encoding="utf-8") as f:
                orig = f.read().strip()
            if orig != "1":
                with open(lat_path, "w", encoding="utf-8") as f:
                    f.write("1")
                self._latency_path = lat_path
                self._orig_latency = orig
                print(f"[FTDI] Tuned {lat_path}: {orig}ms -> 1ms", flush=True)
            else:
                print(f"[FTDI] {lat_path} already 1ms.", flush=True)
        except OSError as e:
            print(
                f"[FTDI] Note: Could not tune {lat_path} to 1ms ({e}); RX jitter up to 16ms (re-run with sudo for 1ms latency).",
                flush=True,
            )

    def close(self) -> None:
        if self._latency_path and self._orig_latency is not None:
            try:
                with open(self._latency_path, "w", encoding="utf-8") as f:
                    f.write(self._orig_latency)
            except OSError:
                pass
        if self.ser and self.ser.is_open:
            try:
                if self.supports_mode_switch:
                    self.ser.dtr = True
                    self.ser.rts = True
                self.ser.close()
            except Exception:
                pass

    def aux_is_high(self) -> bool:
        if not self.supports_mode_switch or not self.ser:
            return True
        try:
            return not bool(self.ser.cts)  # AUX HIGH -> CTS# deasserted (ser.cts == False)
        except Exception:
            return True

    def set_mode_pins(self, m0: int, m1: int) -> None:
        if not self.supports_mode_switch or not self.ser:
            return
        # PySerial True = 0V (LOW), False = +3.3V (HIGH)
        self.ser.dtr = not bool(m0)
        self.ser.rts = not bool(m1)

    def set_baudrate(self, baud: int) -> None:
        if self.ser:
            m0_cached, m1_cached = (not self.ser.dtr, not self.ser.rts) if self.supports_mode_switch else (False, False)
            self.ser.baudrate = baud
            if self.supports_mode_switch:
                # Re-assert DTR/RTS right after termios baud change in case USB-serial toggles lines
                self.ser.dtr = not m0_cached
                self.ser.rts = not m1_cached

    def write_bytes(self, data: bytes) -> None:
        if self.ser:
            self.ser.write(data)

    def flush_tx(self) -> None:
        if self.ser:
            self.ser.flush()

    def drain_rx(self) -> bytes:
        if not self.ser:
            return b""
        avail = self.ser.in_waiting
        if avail > 0:
            return self.ser.read(avail)
        return b""

    def reset_buffers(self) -> None:
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()


@dataclass
class _SimAirFrame:
    tx_name: str
    data: bytes
    start_s: float
    end_s: float
    rate_kbps: float
    channel: int
    corrupted: bool = False


class _SimEther:
    """In-memory half-duplex E220 ether modeling FIFO limits, early AUX release, and collisions."""

    def __init__(self) -> None:
        self.in_flight: list[_SimAirFrame] = []
        self.radios: list["SimE220Radio"] = []
        self.last_pkt_dbm = -42

    def register(self, r: "SimE220Radio") -> None:
        self.radios.append(r)

    def pump(self, now_s: float) -> None:
        # Check mutual overlap collisions
        for i in range(len(self.in_flight)):
            a = self.in_flight[i]
            for j in range(i + 1, len(self.in_flight)):
                b = self.in_flight[j]
                if a.channel == b.channel and a.start_s < b.end_s and b.start_s < a.end_s:
                    a.corrupted = True
                    b.corrupted = True

        done = [f for f in self.in_flight if now_s >= f.end_s]
        self.in_flight = [f for f in self.in_flight if now_s < f.end_s]
        for pkt in done:
            for r in self.radios:
                if r.name == pkt.tx_name:
                    continue
                r._deliver_from_ether(pkt, now_s)


class SimE220Radio(BaseE220Radio):
    """Virtual E220-900T30S transceiver for offline --self-test verification (fast time scaling)."""

    def __init__(self, name: str, ether: _SimEther, time_scale: float = 0.02):
        super().__init__(name, f"sim://{name}", DEFAULT_BAUD)
        self.ether = ether
        self.time_scale = time_scale
        self.mode_pins = (0, 0)
        self.cur_baud = DEFAULT_BAUD
        self.rx_q = bytearray()
        self.regs = bytearray(build_reg_bytes(19.2, 21, DEFAULT_CHANNEL, DEFAULT_BAUD, 200, True) + b"\x00\x00")
        self.aux_busy_until_s = 0.0
        self.rf_busy_until_s = 0.0
        self.last_pkt_dbm = -42
        self.noise_dbm = -99
        self.ether.register(self)

    def open(self) -> None:
        self.reset_buffers()

    def close(self) -> None:
        pass

    def aux_is_high(self) -> bool:
        self.ether.pump(time.monotonic())
        return time.monotonic() >= self.aux_busy_until_s

    def set_mode_pins(self, m0: int, m1: int) -> None:
        self.mode_pins = (m0, m1)

    def set_baudrate(self, baud: int) -> None:
        self.cur_baud = baud

    def hardware_reset(self) -> bool:
        self.aux_busy_until_s = 0.0
        self.rf_busy_until_s = 0.0
        self.rx_q.clear()
        return True

    def write_bytes(self, data: bytes) -> None:
        now = time.monotonic()
        self.ether.pump(now)
        if self.mode_pins == (1, 1):
            if self.cur_baud != CONFIG_BAUD:
                self.rx_q.extend(b"\xFF\xFF\xFF")
                return
            if data[:3] == b"\xC1\x00\x06":
                self.rx_q.extend(b"\xC1\x00\x06" + bytes(self.regs[:6]))
            elif data[:3] == b"\xC1\x00\x08":
                self.rx_q.extend(b"\xC1\x00\x08" + bytes(self.regs[:8]))
            elif len(data) >= 9 and data[0] in (0xC0, 0xC2) and data[1] == 0x00 and data[2] == 0x06:
                self.regs[:6] = data[3:9]
                self.rx_q.extend(b"\xC1\x00\x06" + bytes(self.regs[:6]))
            return

        if len(data) == 6 and data == b"\xC0\xC1\xC2\xC3\x00\x02":
            # Models Mode 0 RSSI turnaround delay (scaled) + reply queuing behind active TX
            cmd_delay = (0.165 if self.air_rate_kbps <= 5.0 else 0.075) * self.time_scale
            wait_s = max(0.0, self.rf_busy_until_s - now) + cmd_delay
            self.aux_busy_until_s = now + wait_s
            n_u8 = (self.noise_dbm + 256) & 0xFF
            p_u8 = (self.last_pkt_dbm + 256) & 0xFF
            time.sleep(wait_s)
            self.rx_q.extend(bytes([0xC1, 0x00, n_u8, p_u8]))
            return

        # Simulate Mode 0 TX FIFO + half-duplex behavior
        if now < self.rf_busy_until_s:
            # Collision inside own radio FIFO if overrun early!
            for f in self.ether.in_flight:
                if f.tx_name == self.name:
                    f.corrupted = True
        offset = 0
        t_cur = max(now, self.rf_busy_until_s)
        if len(data) > 200:
            # Simulates E220 early AUX rise at ~66% of Subpkt 1 on <=9.6k
            s1_air = estimate_subpacket_air_s(200, self.air_rate_kbps) * self.time_scale
            self.aux_busy_until_s = now + (s1_air * (0.65 if self.air_rate_kbps <= 9.6 else 0.95))
        while offset < len(data):
            chunk = data[offset : offset + 200]
            offset += len(chunk)
            air = estimate_subpacket_air_s(len(chunk), self.air_rate_kbps) * self.time_scale
            self.ether.in_flight.append(
                _SimAirFrame(
                    tx_name=self.name,
                    data=chunk,
                    start_s=t_cur,
                    end_s=t_cur + air,
                    rate_kbps=self.air_rate_kbps,
                    channel=self.channel,
                    corrupted=(len(data) > 400),
                )
            )
            t_cur += air
        self.rf_busy_until_s = t_cur

    def _deliver_from_ether(self, pkt: _SimAirFrame, now_s: float) -> None:
        if self.mode_pins != (0, 0) or pkt.rate_kbps != self.air_rate_kbps or pkt.channel != self.channel:
            return
        if now_s < self.rf_busy_until_s or pkt.corrupted:
            # Half-duplex TX or collision corrupts tail bytes
            if len(pkt.data) > 4:
                self.rx_q.extend(pkt.data[: len(pkt.data) // 2] + b"\x00\xFF")
            return
        self.rx_q.extend(pkt.data)

    def flush_tx(self) -> None:
        pass

    def drain_rx(self) -> bytes:
        self.ether.pump(time.monotonic())
        if not self.rx_q:
            return b""
        out = bytes(self.rx_q)
        self.rx_q.clear()
        return out

    def reset_buffers(self) -> None:
        self.rx_q.clear()


@dataclass
class BenchContext:
    tx: BaseE220Radio
    rx: BaseE220Radio
    rates: list[float]
    power_dbm: int
    channel: int
    quick: bool
    permanent_cfg: bool
    csv_dir: str
    run_id: str
    results: list[tuple[str, bool, str]] = field(default_factory=list)

    def time_scale(self) -> float:
        return getattr(self.tx, "time_scale", 1.0)

    def sleep_scaled(self, real_s: float) -> None:
        if real_s > 0:
            time.sleep(real_s * self.time_scale())

    def switch_both_radios(self, air_rate: float) -> bool:
        ok_tx, att_tx, det_tx = self.tx.configure_registers(air_rate, self.power_dbm, self.channel, permanent=self.permanent_cfg)
        ok_rx, att_rx, det_rx = self.rx.configure_registers(air_rate, self.power_dbm, self.channel, permanent=self.permanent_cfg)
        if not (ok_tx and ok_rx):
            print(
                f"  [ERROR] Rate switch to {air_rate}k failed: TX={ok_tx}(att={att_tx},{det_tx}) RX={ok_rx}(att={att_rx},{det_rx})",
                flush=True,
            )
            return False
        self.tx.reset_buffers()
        self.rx.reset_buffers()
        return True

    def csv_path(self, test_name: str) -> str:
        os.makedirs(self.csv_dir, exist_ok=True)
        return os.path.join(self.csv_dir, f"bench_{self.run_id}_{test_name}.csv")


class AsyncRxCollector:
    """
    Concurrent background RX polling thread so Test 1 measures exact write-to-RX-complete
    wall latency while TX sleeps in send_paced_frame(), and Test 2/3/6 never overflow serial RX buffers.
    Must ONLY be active in Mode 0 at steady baud (stopped before Mode 3 register switches or Mode 0 RSSI queries).
    """

    def __init__(self, rx: BaseE220Radio):
        self.rx = rx
        self._io_lock = threading.Lock()
        self.lock = threading.Lock()
        self.buf = bytearray()
        self.frames: dict[int, tuple[int, int, float]] = {}
        self.crc_errors = 0
        self.orphan_bytes = 0
        self.last_error: Optional[str] = None
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop = False
        self.last_error = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None

    def reset_stats(self) -> None:
        with self._io_lock:
            self.rx.drain_rx()
            with self.lock:
                self.buf.clear()
                self.frames.clear()
                self.crc_errors = 0
                self.orphan_bytes = 0

    def _run(self) -> None:
        poll_s = self.rx._poll_interval(0.001)
        while not self._stop and not g_abort_requested:
            try:
                with self._io_lock:
                    if self._stop or g_abort_requested:
                        break
                    chunk = self.rx.drain_rx()
            except Exception as e:
                if not self._stop:
                    with self.lock:
                        self.last_error = str(e)
                    print(f"  [ERROR] AsyncRxCollector RX read failure: {e}", file=sys.stderr, flush=True)
                break
            if chunk:
                now = time.monotonic()
                with self.lock:
                    self.buf.extend(chunk)
                    val, ce, orp = parse_synthetic_rtcm(self.buf)
                    self.crc_errors += ce
                    self.orphan_bytes += orp
                    for mtype, flen, seq in val:
                        if seq not in self.frames:
                            self.frames[seq] = (mtype, flen, now)
            else:
                time.sleep(poll_s)

    def wait_for(self, expected_seqs: set[int], timeout_s: float) -> tuple[dict[int, tuple[int, int, float]], int, int]:
        deadline = time.monotonic() + (timeout_s * self.rx.time_scale)
        poll_s = self.rx._poll_interval(0.002)
        while time.monotonic() < deadline and not g_abort_requested:
            with self.lock:
                if self.last_error is not None or expected_seqs.issubset(self.frames.keys()):
                    break
            time.sleep(poll_s)
        with self.lock:
            out = {s: self.frames[s] for s in expected_seqs if s in self.frames}
            return (out, self.crc_errors, self.orphan_bytes + len(self.buf))


def _receive_expected_frames(
    rx: BaseE220Radio,
    expected_seqs: set[int],
    max_wait_s: float,
    time_scale: float = 1.0,
) -> tuple[dict[int, tuple[int, int, float]], int, int]:
    """Read RX synchronously until expected_seqs pass CRC24Q or max_wait_s expires."""
    found: dict[int, tuple[int, int, float]] = {}
    rx_buf = bytearray()
    crc_errs = 0
    orphans = 0
    t0 = time.monotonic()
    deadline = t0 + (max_wait_s * time_scale)

    while time.monotonic() < deadline and not g_abort_requested:
        chunk = rx.drain_rx()
        if chunk:
            now = time.monotonic()
            rx_buf.extend(chunk)
            valid, c_err, orph = parse_synthetic_rtcm(rx_buf)
            crc_errs += c_err
            orphans += orph
            for mtype, flen, seq in valid:
                if seq not in found:
                    found[seq] = (mtype, flen, ((now - t0) * 1000.0) / time_scale)
            if expected_seqs.issubset(found.keys()):
                break
        else:
            time.sleep(rx._poll_interval(0.001))
    return (found, crc_errs, orphans + len(rx_buf))


def run_test_1_toa_sweep(ctx: BenchContext) -> bool:
    """Test 1: Empirical ToA & AUX Waveform Sweeper (18 B .. 400 B + 456 B Split)."""
    print("\n======================================================================", flush=True)
    print("[TEST 1] Empirical ToA & AUX Waveform Sweeper (18 B -> 456 B)", flush=True)
    print("======================================================================", flush=True)

    lengths = [18, 27, 45, 144, 173, 189, 200, 201, 253, 316, 400, 456]
    repeats = 2 if ctx.quick else 4
    rows: list[list[object]] = []
    seq_id = 1000
    all_ok = True
    rx_col = AsyncRxCollector(ctx.rx)

    for rate in ctx.rates:
        if g_abort_requested or not ctx.switch_both_radios(rate):
            all_ok = False
            break
        print(f"\n--- Rate {rate} kbps ---", flush=True)
        print("  Len | Subs | AUX_Drop | AUX_Low_ms | TX_Pace_ms | Model_ToA | RX_Wall_ms | Pass", flush=True)
        rx_col.start()
        try:
            for length in lengths:
                aux_lows: list[float] = []
                tx_paces: list[float] = []
                rx_walls: list[float] = []
                passes = 0
                any_drop = False
                for _ in range(repeats):
                    if g_abort_requested or rx_col.last_error is not None:
                        break
                    seq_id += 1
                    frame = build_synthetic_rtcm(1074 if length > 200 else 1084, length, seq_id)
                    rx_col.reset_stats()
                    t_send = time.monotonic()
                    tx_res = ctx.tx.send_paced_frame(frame, mode="toa_calibrated")
                    found, c_err, orph = rx_col.wait_for({seq_id}, 0.8)
                    wall_ms = ((found[seq_id][2] - t_send) * 1000.0) / ctx.time_scale() if seq_id in found else -1.0
                    ok = (seq_id in found) and (c_err == 0) and (rx_col.last_error is None)
                    if ok:
                        passes += 1
                        rx_walls.append(wall_ms)
                    any_drop = any_drop or bool(tx_res["aux_dropped"])
                    if tx_res["aux_dropped"]:
                        aux_lows.append(float(tx_res["aux_low_ms"]))
                    tx_paces.append(float(tx_res["elapsed_ms"]))
                    rows.append([rate, length, tx_res["subpkts"], int(bool(tx_res["aux_dropped"])), round(float(tx_res["aux_low_ms"]), 2), round(tx_paces[-1], 2), round(estimate_frame_toa_ms(length, rate), 2), round(wall_ms, 2), int(ok), c_err, orph])
                    ctx.sleep_scaled(0.030)

                model_ms = estimate_frame_toa_ms(length, rate)
                avg_aux = statistics.mean(aux_lows) if aux_lows else 0.0
                avg_tx = statistics.mean(tx_paces) if tx_paces else 0.0
                avg_rx = statistics.mean(rx_walls) if rx_walls else -1.0
                print(
                    f"  {length:3d} |  {(length+199)//200:2d}  |   {'YES' if any_drop else ' NO'}    | {avg_aux:9.1f}  |  {avg_tx:9.1f} | {model_ms:9.1f} |  {avg_rx:9.1f} | {passes}/{repeats}",
                    flush=True,
                )
                if passes < repeats or rx_col.last_error is not None:
                    all_ok = False
        finally:
            rx_col.stop()

    with open(ctx.csv_path("test1_toa"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["rate_kbps", "len_b", "subpkts", "aux_dropped", "aux_low_ms", "tx_pace_ms", "model_toa_ms", "rx_wall_ms", "ok", "crc_err", "orphan_b"])
        w.writerows(rows)

    ctx.results.append(("test1_toa", all_ok, f"{len(rows)} probes saved"))
    return all_ok


def run_test_2_guard_sweep(ctx: BenchContext) -> bool:
    """Test 2: Minimum Inter-Frame Guard & Pacing Method Comparison (AUX-only vs ToA-floor vs Coalesce)."""
    print("\n======================================================================", flush=True)
    print("[TEST 2] Minimum Inter-Frame Guard & Pacing Method Comparison", flush=True)
    print("======================================================================", flush=True)

    pairs = [(200, 18), (253, 189), (316, 27), (400, 45)]
    guards_ms = [60, 50, 40, 30, 20, 10]
    repeats = 3 if ctx.quick else 8
    rows: list[list[object]] = []
    seq_id = 10000
    all_ok = True
    rx_col = AsyncRxCollector(ctx.rx)

    for rate in ctx.rates:
        if g_abort_requested or not ctx.switch_both_radios(rate):
            all_ok = False
            break
        print(f"\n--- Rate {rate} kbps ---", flush=True)
        rx_col.start()
        try:
            # 2a. Negative control: AUX-only pacing at 35ms (expected to fail/truncate <=9.6k or <=200B!)
            aux_fail = 0
            for p1, p2 in pairs[:2]:
                for _ in range(max(2, repeats // 2)):
                    seq_id += 1
                    s1 = seq_id
                    seq_id += 1
                    s2 = seq_id
                    f1 = build_synthetic_rtcm(1074, p1, s1)
                    f2 = build_synthetic_rtcm(1006, p2, s2)
                    rx_col.reset_stats()
                    ctx.tx.send_paced_frame(f1, mode="aux_only", guard_s=0.035)
                    ctx.tx.send_paced_frame(f2, mode="aux_only", guard_s=0.035)
                    found, c_err, _ = rx_col.wait_for({s1, s2}, 1.4)
                    ok = len(found) == 2 and c_err == 0
                    if not ok:
                        aux_fail += 1
                    rows.append([rate, "aux_only", 35, f"{p1}+{p2}", int(ok), c_err, len(found)])
                    ctx.sleep_scaled(0.05)
            print(f"  [CTRL] aux_only (35ms guard): failed {aux_fail}/{2 * max(2, repeats // 2)} bursts (expected due to missing/early AUX)", flush=True)

            # 2b. ToA-floor sweep across Guard = 60 -> 10 ms
            for gms in guards_ms:
                tot_runs = 0
                ok_runs = 0
                for p1, p2 in pairs:
                    for _ in range(repeats):
                        if g_abort_requested or rx_col.last_error is not None:
                            break
                        seq_id += 1
                        s1 = seq_id
                        seq_id += 1
                        s2 = seq_id
                        f1 = build_synthetic_rtcm(1074, p1, s1)
                        f2 = build_synthetic_rtcm(1084, p2, s2)
                        rx_col.reset_stats()
                        ctx.tx.send_paced_frame(f1, mode="toa_calibrated", guard_s=gms / 1000.0)
                        ctx.tx.send_paced_frame(f2, mode="toa_calibrated", guard_s=gms / 1000.0)
                        found, c_err, _ = rx_col.wait_for({s1, s2}, 0.6)
                        ok = (s1 in found and s2 in found and c_err == 0 and rx_col.last_error is None)
                        tot_runs += 1
                        if ok:
                            ok_runs += 1
                        rows.append([rate, "toa_floor", gms, f"{p1}+{p2}", int(ok), c_err, len(found)])
                        ctx.sleep_scaled(0.03)
                pct = 100.0 * ok_runs / max(1, tot_runs)
                print(f"  [TOA-FLOOR] Guard={gms:2d}ms: {ok_runs}/{tot_runs} ({pct:5.1f}%) intact", flush=True)
                if gms >= 50 and ok_runs < tot_runs:
                    all_ok = False

            # 2c. Coalesced 1006(27) + 1230(18) + 1084(144) = 189B single write vs 3 separate writes
            for mode_lbl in ("separate_3", "coalesced_189"):
                ok_c = 0
                dur_list: list[float] = []
                for _ in range(repeats):
                    if g_abort_requested or rx_col.last_error is not None:
                        break
                    seq_id += 1
                    s_a = seq_id
                    seq_id += 1
                    s_b = seq_id
                    seq_id += 1
                    s_c = seq_id
                    fa = build_synthetic_rtcm(1006, 27, s_a)
                    fb = build_synthetic_rtcm(1230, 18, s_b)
                    fc = build_synthetic_rtcm(1084, 144, s_c)
                    rx_col.reset_stats()
                    t_a = time.monotonic()
                    if mode_lbl == "coalesced_189":
                        ctx.tx.send_paced_frame(fa + fb + fc, mode="toa_calibrated")
                    else:
                        ctx.tx.send_paced_frame(fa, mode="toa_calibrated")
                        ctx.tx.send_paced_frame(fb, mode="toa_calibrated")
                        ctx.tx.send_paced_frame(fc, mode="toa_calibrated")
                    found, c_err, _ = rx_col.wait_for({s_a, s_b, s_c}, 0.6)
                    dur_list.append(((time.monotonic() - t_a) * 1000.0) / ctx.time_scale())
                    ok = len(found) == 3 and c_err == 0 and rx_col.last_error is None
                    if ok:
                        ok_c += 1
                    rows.append([rate, mode_lbl, 52 if rate <= 9.6 else 40, "27+18+144", int(ok), c_err, len(found)])
                avg_d = statistics.mean(dur_list) if dur_list else 0.0
                print(f"  [{mode_lbl:13s}]: {ok_c}/{repeats} intact | avg_dur={avg_d:6.1f}ms", flush=True)
                if ok_c < repeats:
                    all_ok = False
        finally:
            rx_col.stop()

    with open(ctx.csv_path("test2_guard"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["rate_kbps", "method", "guard_ms", "pair", "ok", "crc_err", "frames_rx"])
        w.writerows(rows)

    ctx.results.append(("test2_guard", all_ok, f"{ctx.tx.aux_resets} AUX resets"))
    return all_ok


def run_test_3_per_comparison(ctx: BenchContext) -> bool:
    """Test 3: PER Comparison (1-pkt vs 2-pkt vs Coalesced <=200B vs Arbitrary 200B Stream Slicing)."""
    print("\n======================================================================", flush=True)
    print("[TEST 3] Framing PER: 1-pkt vs 2-pkt vs Coalesced <=200B vs 200B Stream", flush=True)
    print("======================================================================", flush=True)

    burst_n = 15 if ctx.quick else 60
    rows: list[list[object]] = []
    seq_id = 30000
    all_ok = True
    rx_col = AsyncRxCollector(ctx.rx)

    for rate in ctx.rates:
        if g_abort_requested or not ctx.switch_both_radios(rate):
            all_ok = False
            break
        print(f"\n--- Rate {rate} kbps (N={burst_n} epochs/mode) ---", flush=True)
        rx_col.start()
        try:
            # Mode A: Whole-Frame isolated (1-pkt 173B, 2-pkt 253B, Max-Sat 316B)
            for label, sz in [("1pkt_173B", 173), ("2pkt_253B", 253), ("2pkt_316B", 316)]:
                expected: set[int] = set()
                rx_col.reset_stats()
                for _ in range(burst_n):
                    if g_abort_requested or rx_col.last_error is not None:
                        break
                    seq_id += 1
                    expected.add(seq_id)
                    ctx.tx.send_paced_frame(build_synthetic_rtcm(1074, sz, seq_id), mode="toa_calibrated")
                found, crc_errs, orphans = rx_col.wait_for(expected, 1.5)
                ok_cnt = len(found)
                pct = 100.0 * ok_cnt / max(1, len(expected))
                print(f"  [{label:14s}] OK={ok_cnt:3d}/{len(expected):3d} ({pct:5.1f}%) CRC_err={crc_errs} Orph={orphans}B", flush=True)
                rows.append([rate, label, len(expected), ok_cnt, round(pct, 2), crc_errs, orphans])

            # Mode B: Coalesced <=200B (189B = 27+18+144) vs Arbitrary 200B sliced stream (235+144+224)
            for stream_mode in ("coalesced_189B", "sliced_200B_stream"):
                expected_s: set[int] = set()
                blob_stream = bytearray()
                rx_col.reset_stats()

                for _ in range(burst_n):
                    if g_abort_requested or rx_col.last_error is not None:
                        break
                    seq_id += 1
                    s1 = seq_id
                    seq_id += 1
                    s2 = seq_id
                    seq_id += 1
                    s3 = seq_id
                    expected_s.update((s1, s2, s3))
                    if stream_mode == "coalesced_189B":
                        f_pack = (
                            build_synthetic_rtcm(1006, 27, s1)
                            + build_synthetic_rtcm(1230, 18, s2)
                            + build_synthetic_rtcm(1084, 144, s3)
                        )
                        ctx.tx.send_paced_frame(f_pack, mode="toa_calibrated")
                    else:
                        blob_stream.extend(
                            build_synthetic_rtcm(1074, 235, s1)
                            + build_synthetic_rtcm(1084, 144, s2)
                            + build_synthetic_rtcm(1094, 224, s3)
                        )
                        while len(blob_stream) >= 200:
                            ctx.tx.send_paced_frame(bytes(blob_stream[:200]), mode="toa_calibrated")
                            del blob_stream[:200]

                if len(blob_stream) > 0:
                    ctx.tx.send_paced_frame(bytes(blob_stream), mode="toa_calibrated")
                found_s, crc_errs, orphans = rx_col.wait_for(expected_s, 1.8)
                ok_cnt = len(found_s)
                pct = 100.0 * ok_cnt / max(1, len(expected_s))
                print(f"  [{stream_mode:18s}] OK={ok_cnt:3d}/{len(expected_s):3d} ({pct:5.1f}%) CRC_err={crc_errs} Orph={orphans}B", flush=True)
                rows.append([rate, stream_mode, len(expected_s), ok_cnt, round(pct, 2), crc_errs, orphans])
                if rx_col.last_error is not None:
                    all_ok = False
        finally:
            rx_col.stop()

    with open(ctx.csv_path("test3_per"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["rate_kbps", "mode", "sent_frames", "ok_frames", "ok_pct", "crc_err", "orphan_b"])
        w.writerows(rows)

    ctx.results.append(("test3_per", all_ok, f"{len(rows)} profiles logged"))
    return all_ok


def run_test_4_uplink_rssi_sweep(ctx: BenchContext) -> bool:
    """Test 4: Consolidated Uplink (48B) + Mode 0 RSSI Query Turnaround Collision Sweep."""
    print("\n======================================================================", flush=True)
    print("[TEST 4] Consolidated Uplink + Mode 0 C0 C1 C2 C3 RSSI Margin Sweep", flush=True)
    print("======================================================================", flush=True)

    turnarounds_ms = [30, 60, 100] if ctx.quick else [20, 40, 60, 80, 100]
    repeats = 3 if ctx.quick else 6
    rows: list[list[object]] = []
    seq_id = 50000
    all_ok = True

    for rate in ctx.rates:
        if g_abort_requested or not ctx.switch_both_radios(rate):
            all_ok = False
            break
        print(f"\n--- Rate {rate} kbps ---", flush=True)
        for t_ms in turnarounds_ms:
            dl_ok = 0
            ul_ok = 0
            rssi_ok = 0
            spills = 0
            rssi_lats: list[float] = []

            for i in range(repeats):
                if g_abort_requested:
                    break
                seq_id += 1
                # 1. Base transmits 253B MSM4 downlink
                ctx.rx.drain_rx()
                ctx.tx.send_paced_frame(build_synthetic_rtcm(1074, 253, seq_id), mode="toa_calibrated")
                f1, ce1, _ = _receive_expected_frames(ctx.rx, {seq_id}, 1.2, ctx.time_scale())
                if seq_id in f1 and ce1 == 0:
                    dl_ok += 1

                # 2. Half-duplex TX->RX Turnaround Gap
                ctx.sleep_scaled(t_ms / 1000.0)
                ctx.tx.drain_rx()

                # 3. Rover FTDI transmits 48B telemetry uplink (waits full ToA + 30ms)
                ul_seq = (i + 1) & 0xFF
                ul_frame = build_uplink_48b(ul_seq)
                ctx.rx.send_paced_frame(ul_frame, mode="toa_calibrated", guard_s=0.030)
                ctx.sleep_scaled(0.060)
                ul_raw = bytearray(ctx.tx.drain_rx())
                ul_seqs, ul_ce = verify_uplink_48b(ul_raw)
                if ul_seq in ul_seqs and ul_ce == 0:
                    ul_ok += 1

                # 4. Rover FTDI issues Mode 0 RSSI query inside remaining quiet window
                r_ok, n_dbm, p_dbm, r_lat, _ = ctx.rx.query_mode0_rssi(timeout_s=0.30)
                if r_ok:
                    rssi_ok += 1
                    rssi_lats.append(r_lat)

                # 5. Resume Base downlink and verify zero RSSI 5B spill or collision
                ctx.sleep_scaled(0.040)
                seq_id += 1
                ctx.tx.send_paced_frame(build_synthetic_rtcm(1006, 27, seq_id), mode="toa_calibrated")
                f2, ce2, orp2 = _receive_expected_frames(ctx.rx, {seq_id}, 0.8, ctx.time_scale())
                if seq_id not in f2 or ce2 > 0 or orp2 > 0:
                    spills += 1

            avg_lat = statistics.mean(rssi_lats) if rssi_lats else -1.0
            print(
                f"  Turnaround={t_ms:3d}ms: DL1={dl_ok}/{repeats} UL={ul_ok}/{repeats} RSSI={rssi_ok}/{repeats} (avg={avg_lat:5.1f}ms) Spills={spills}",
                flush=True,
            )
            rows.append([rate, t_ms, repeats, dl_ok, ul_ok, rssi_ok, round(avg_lat, 1), spills])
            if t_ms >= 60 and (dl_ok < repeats or ul_ok < repeats or rssi_ok < repeats or spills > 0):
                all_ok = False

    with open(ctx.csv_path("test4_uplink_rssi"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["rate_kbps", "turnaround_ms", "trials", "dl1_ok", "ul_ok", "rssi_ok", "rssi_lat_ms", "spills"])
        w.writerows(rows)

    ctx.results.append(("test4_uplink_rssi", all_ok, f"{len(rows)} turnaround sweeps"))
    return all_ok


def run_test_5_switch_endurance(ctx: BenchContext) -> bool:
    """Test 5: Automated Mode 3 Air-Rate Switch Endurance Loop (19.2k -> 9.6k -> 4.8k -> 19.2k)."""
    print("\n======================================================================", flush=True)
    print("[TEST 5] Automated Mode 3 3-Rate Switch Endurance Loop", flush=True)
    print("======================================================================", flush=True)

    cycles = 5 if ctx.quick else 30
    cycle_rates = [19.2, 9.6, 4.8, 19.2]
    rows: list[list[object]] = []
    seq_id = 70000
    total_legs = 0
    ok_legs = 0

    for cyc in range(1, cycles + 1):
        for rate in cycle_rates:
            if g_abort_requested:
                break
            total_legs += 1
            t0 = time.monotonic()
            ok_tx, att_tx, det_tx = ctx.tx.configure_registers(rate, ctx.power_dbm, ctx.channel, permanent=ctx.permanent_cfg, verify_read=True)
            ok_rx, att_rx, det_rx = ctx.rx.configure_registers(rate, ctx.power_dbm, ctx.channel, permanent=ctx.permanent_cfg, verify_read=True)
            link_ok = False
            if ok_tx and ok_rx:
                ctx.sleep_scaled(0.040)
                ctx.tx.reset_buffers()
                ctx.rx.reset_buffers()
                seq_id += 1
                ctx.tx.send_paced_frame(build_synthetic_rtcm(1084, 144, seq_id), mode="toa_calibrated")
                found, c_err, orph = _receive_expected_frames(ctx.rx, {seq_id}, 1.4, ctx.time_scale())
                link_ok = (seq_id in found and c_err == 0)
                if not link_ok:
                    det_rx = f"{det_rx} rf_fail(found={len(found)} crc={c_err} orph={orph})"
            if ok_tx and ok_rx and link_ok:
                ok_legs += 1
            dt_ms = ((time.monotonic() - t0) * 1000.0) / ctx.time_scale()
            rows.append([cyc, rate, int(ok_tx), att_tx, int(ok_rx), att_rx, int(link_ok), round(dt_ms, 1), det_tx, det_rx])
        if g_abort_requested:
            break
        print(f"  Cycle {cyc:2d}/{cycles}: cumulative legs passed = {ok_legs}/{total_legs}", flush=True)

    with open(ctx.csv_path("test5_switch"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["cycle", "rate_kbps", "tx_ok", "tx_att", "rx_ok", "rx_att", "rf_ok", "elapsed_ms", "tx_detail", "rx_detail"])
        w.writerows(rows)

    all_ok = (ok_legs == total_legs) and (total_legs > 0)
    ctx.results.append(("test5_switch", all_ok, f"{ok_legs}/{total_legs} transitions OK"))
    return all_ok


def run_test_6_thermal_soak(ctx: BenchContext, soak_minutes: float) -> bool:
    """Test 6: Sustained High-Duty (~85-90%) Soak for Passive 32 MHz XTAL Thermal Drift Tracking."""
    print("\n======================================================================", flush=True)
    print(f"[TEST 6] High-Duty Thermal Drift Soak ({soak_minutes:.2f} min/rate)", flush=True)
    print("======================================================================", flush=True)

    soak_rates = [r for r in ctx.rates if r <= 9.6] or ctx.rates[:1]
    rows: list[list[object]] = []
    seq_id = 90000
    all_ok = True
    rx_col = AsyncRxCollector(ctx.rx)

    for rate in soak_rates:
        if g_abort_requested or not ctx.switch_both_radios(rate):
            all_ok = False
            break
        print(f"\n--- Thermal Soak @ {rate} kbps ({ctx.power_dbm} dBm, {soak_minutes:.2f} min) ---", flush=True)
        end_time = time.monotonic() + (soak_minutes * 60.0 * ctx.time_scale())
        win_start = time.monotonic()
        win_sent: set[int] = set()
        minute_idx = 0
        rx_col.start()
        try:
            rx_col.reset_stats()
            while time.monotonic() < end_time and not g_abort_requested and rx_col.last_error is None:
                seq_id += 1
                win_sent.add(seq_id)
                sz = 253 if (seq_id % 2 == 0) else 173
                ctx.tx.send_paced_frame(build_synthetic_rtcm(1074 if sz > 200 else 1084, sz, seq_id), mode="toa_calibrated")

                if (time.monotonic() - win_start) >= (60.0 * ctx.time_scale()) or time.monotonic() >= end_time:
                    minute_idx += 1
                    found, win_crc_err, _ = rx_col.wait_for(win_sent, 1.0)
                    ok_n = len(found)
                    pct = 100.0 * ok_n / max(1, len(win_sent))
                    had_err = rx_col.last_error is not None
                    rx_col.stop()
                    r_ok, n_dbm, p_dbm, _, _ = ctx.rx.query_mode0_rssi(timeout_s=0.25)
                    if time.monotonic() < end_time and not g_abort_requested and not had_err:
                        rx_col.start()
                        rx_col.reset_stats()
                    print(
                        f"  Min {minute_idx:2d}: OK={ok_n}/{len(win_sent)} ({pct:5.1f}%) CRC_err={win_crc_err} "
                        f"Noise={n_dbm if r_ok else '--'}dBm Pkt={p_dbm if r_ok else '--'}dBm AUX_Rst={ctx.tx.aux_resets}",
                        flush=True,
                    )
                    rows.append([rate, minute_idx, len(win_sent), ok_n, round(pct, 2), win_crc_err, n_dbm if r_ok else None, p_dbm if r_ok else None, ctx.tx.aux_resets])
                    if pct < 95.0 or had_err:
                        all_ok = False
                    win_sent.clear()
                    win_start = time.monotonic()
        finally:
            rx_col.stop()

    with open(ctx.csv_path("test6_thermal"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["rate_kbps", "minute", "sent", "ok", "ok_pct", "crc_err", "noise_dbm", "pkt_dbm", "aux_resets"])
        w.writerows(rows)

    ctx.results.append(("test6_thermal", all_ok, f"{len(rows)} soak windows"))
    return all_ok


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=f"Stage 1 E220 Multi-Rate Bench Characterization Suite ({__version__})")
    parser.add_argument("--tx-port", default=DEFAULT_TX_PORT, help=f"Base GPIO UART port (default: {DEFAULT_TX_PORT})")
    parser.add_argument("--rx-port", default=DEFAULT_RX_PORT, help=f"Rover FTDI USB UART port (default: {DEFAULT_RX_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Operational UART baud (default: {DEFAULT_BAUD})")
    parser.add_argument("--rates", default="4.8,9.6,19.2", help="Comma-separated air rates in kbps (default: 4.8,9.6,19.2)")
    parser.add_argument("--power", type=int, default=DEFAULT_POWER_DBM, choices=[21, 24, 27, 30], help="TX power dBm (default: 21)")
    parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL, help=f"RF channel 0..83 (default: {DEFAULT_CHANNEL} -> 925.125MHz)")
    parser.add_argument("--tests", default="all", help=f"Comma-separated tests or 'all' ({','.join(ALL_TESTS)})")
    parser.add_argument("--log-dir", default="./bench_e220_logs", help="Output CSV directory (default: ./bench_e220_logs)")
    parser.add_argument("--thermal-minutes", type=float, default=5.0, help="Soak duration per rate in Test 6 (default: 5.0 min)")
    parser.add_argument("--stop-service", action="store_true", help="Safely stop active e220-pacer.service and restore on exit")
    parser.add_argument("--leave-stopped", action="store_true", help="Do not restart e220-pacer.service after exit")
    parser.add_argument("--eeprom-write", action="store_true", help="Use 0xC0 EEPROM writes instead of 0xC2 temporary RAM (wear warning)")
    parser.add_argument("--rx-no-modem-lines", action="store_true", help="Disable FTDI DTR/RTS/CTS# mode switching on RX port")
    parser.add_argument("--quick", action="store_true", help="Reduced repetition counts for rapid bench sanity checks")
    parser.add_argument("--self-test", action="store_true", help="Run full virtual in-memory E220 simulation self-test (no hardware required)")
    parser.add_argument("-v", "--version", action="version", version=f"{__filename__} {__version__} ({__date__})")
    return parser.parse_args()


def main() -> int:
    print(f"=== {__filename__} {__version__} ===", flush=True)
    args = parse_args()

    signal.signal(signal.SIGINT, _signal_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _signal_handler)

    try:
        rates = [float(r.strip()) for r in args.rates.split(",") if r.strip()]
        for r in rates:
            if r not in AIR_RATE_BITS:
                raise ValueError(f"Unsupported air rate {r} kbps (valid: {sorted(AIR_RATE_BITS)})")
    except ValueError as e:
        print(f"[ERROR] Invalid --rates specification: {e}", file=sys.stderr)
        return 2

    selected = list(ALL_TESTS) if args.tests.strip().lower() == "all" else [t.strip().lower() for t in args.tests.split(",") if t.strip()]
    for t in selected:
        if t not in ALL_TESTS:
            print(f"[ERROR] Unknown test '{t}'. Valid: {', '.join(ALL_TESTS)}", file=sys.stderr)
            return 2

    svc_guard = ServiceGuard(stop_service=args.stop_service, leave_stopped=args.leave_stopped, sim_mode=args.self_test)
    tx_radio: Optional[BaseE220Radio] = None
    rx_radio: Optional[BaseE220Radio] = None

    try:
        svc_guard.acquire()

        if args.self_test:
            ether = _SimEther()
            tx_radio = SimE220Radio("Sim-Base", ether, time_scale=0.015)
            rx_radio = SimE220Radio("Sim-Rover", ether, time_scale=0.015)
            soak_min = min(args.thermal_minutes, 0.08)
            quick_mode = True
        else:
            tx_radio = PiGpioRadio(port=args.tx_port, baud=args.baud)
            rx_radio = FtdiUsbRadio(port=args.rx_port, baud=args.baud, no_modem_lines=args.rx_no_modem_lines)
            soak_min = 1.5 if args.quick else args.thermal_minutes
            quick_mode = args.quick

        tx_radio.open()
        rx_radio.open()

        tx_regs = tx_radio.read_registers_mode3()
        rx_regs = rx_radio.read_registers_mode3()
        tx_radio.initial_regs = tx_regs
        rx_radio.initial_regs = rx_regs
        print(f"[RADIO TX] {args.tx_port}: {decode_reg_str(tx_regs or b'')}", flush=True)
        print(f"[RADIO RX] {args.rx_port}: {decode_reg_str(rx_regs or b'')}", flush=True)

        run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        ctx = BenchContext(
            tx=tx_radio,
            rx=rx_radio,
            rates=rates,
            power_dbm=args.power,
            channel=args.channel,
            quick=quick_mode,
            permanent_cfg=args.eeprom_write,
            csv_dir=args.log_dir,
            run_id=run_id,
        )

        test_dispatch = {
            "toa": lambda: run_test_1_toa_sweep(ctx),
            "guard": lambda: run_test_2_guard_sweep(ctx),
            "per": lambda: run_test_3_per_comparison(ctx),
            "uplink_rssi": lambda: run_test_4_uplink_rssi_sweep(ctx),
            "switch": lambda: run_test_5_switch_endurance(ctx),
            "thermal": lambda: run_test_6_thermal_soak(ctx, soak_min),
        }

        all_passed = True
        for t in selected:
            if g_abort_requested:
                break
            passed = test_dispatch[t]()
            if not passed:
                all_passed = False

        print("\n======================================================================", flush=True)
        print(f"SUMMARY ({__filename__} {__version__} | run={run_id})", flush=True)
        for name, ok, detail in ctx.results:
            print(f"  [{'PASS' if ok else 'FAIL'}] {name:18s} -> {detail}", flush=True)
        print("======================================================================", flush=True)
        return 0 if (all_passed and not g_abort_requested) else 1

    except Exception as e:
        print(f"[ERROR] Bench harness failed: {e}", file=sys.stderr, flush=True)
        return 1
    finally:
        if tx_radio is not None:
            if not (not args.eeprom_write and tx_radio.hardware_reset()) and tx_radio.initial_regs and len(tx_radio.initial_regs) >= 6:
                orig_air = INV_AIR_RATE.get(tx_radio.initial_regs[2] & 0x07, 19.2)
                orig_pwr = INV_POWER.get(tx_radio.initial_regs[3] & 0x03, 21)
                orig_ch = tx_radio.initial_regs[4]
                tx_radio.configure_registers(orig_air, orig_pwr, orig_ch, permanent=args.eeprom_write)
            tx_radio.close()
        if rx_radio is not None:
            if not (not args.eeprom_write and rx_radio.hardware_reset()) and rx_radio.initial_regs and len(rx_radio.initial_regs) >= 6:
                orig_air = INV_AIR_RATE.get(rx_radio.initial_regs[2] & 0x07, 19.2)
                orig_pwr = INV_POWER.get(rx_radio.initial_regs[3] & 0x03, 21)
                orig_ch = rx_radio.initial_regs[4]
                rx_radio.configure_registers(orig_air, orig_pwr, orig_ch, permanent=args.eeprom_write)
            rx_radio.close()
        svc_guard.release()


if __name__ == "__main__":
    sys.exit(main())
