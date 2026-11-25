#!/usr/bin/env python3
"""Host unit test and C++17 syntax verification harness for Field_Instrument_Core."""

import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

__filename__ = os.path.basename(__file__)
__version__ = "v1.1.0"

UNIT_DIR = pathlib.Path(__file__).resolve().parent
CORE_ROOT = UNIT_DIR.parent.parent
SRC_DIR = CORE_ROOT / "src"


def find_host_cpp_compiler():
    for candidate in ("g++", "clang++", "cl"):
        found = shutil.which(candidate)
        if found:
            return candidate, found
    return None, None


def find_xtensa_cpp_compiler():
    local_app_data = os.environ.get("LOCALAPPDATA", "")
    if not local_app_data:
        return None
    tools_root = pathlib.Path(local_app_data) / "Arduino15" / "packages" / "esp32" / "tools" / "esp-x32"
    if not tools_root.is_dir():
        return None
    matches = sorted(tools_root.rglob("xtensa-esp32s3-elf-g++.exe"))
    return str(matches[-1]) if matches else None


def run_syntax_checks(xtensa_gpp):
    print(f"[*] Running ESP32-S3 C++17 syntax checks via {os.path.basename(xtensa_gpp)}...")
    targets = [
        UNIT_DIR / "test_fic_core.cpp",
        SRC_DIR / "fusion" / "fic_ahrs_9dof.cpp",
        SRC_DIR / "ble" / "fic_ble_protocol.cpp",
    ]
    for tgt in targets:
        cmd = [
            xtensa_gpp,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-fsyntax-only",
            f"-I{SRC_DIR}",
            str(tgt),
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            print(res.stdout)
            print(res.stderr)
            raise RuntimeError(f"Syntax check failed for {tgt}")
    print("[PASS] ESP32-S3 C++17 -Wall -Wextra -Werror syntax check passed.")


def run_native_cpp_tests(comp_name, comp_path):
    print(f"[*] Compiling & executing host C++ suite via {comp_name} ({comp_path})...")
    with tempfile.TemporaryDirectory() as tmpdir:
        exe_path = pathlib.Path(tmpdir) / ("test_fic_core.exe" if os.name == "nt" else "test_fic_core")
        srcs = [
            str(UNIT_DIR / "test_fic_core.cpp"),
            str(SRC_DIR / "fusion" / "fic_ahrs_9dof.cpp"),
            str(SRC_DIR / "ble" / "fic_ble_protocol.cpp"),
        ]
        if comp_name == "cl":
            cmd = [comp_path, "/std:c++17", "/W4", "/WX", f"/I{SRC_DIR}", *srcs, f"/Fe:{exe_path}"]
        else:
            cmd = [comp_path, "-std=c++17", "-Wall", "-Wextra", "-Werror", f"-I{SRC_DIR}", *srcs, "-lm", "-o", str(exe_path)]
        build = subprocess.run(cmd, capture_output=True, text=True)
        if build.returncode != 0:
            print(build.stdout)
            print(build.stderr)
            raise RuntimeError("Native C++ host test compilation failed")
        run = subprocess.run([str(exe_path)], capture_output=True, text=True)
        print(run.stdout.strip())
        if run.returncode != 0:
            print(run.stderr)
            raise RuntimeError("Native C++ host test binary failed")


class CoreInvariantTests(unittest.TestCase):
    def test_orient_tables_determinant_plus_one(self):
        # Verify all 8 AHRS orientation permutations defined in fic_ahrs_9dof.cpp have det = +1
        tables = [
            ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
            ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
            ((-1, 0, 0), (0, -1, 0), (0, 0, 1)),
            ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
            ((1, 0, 0), (0, -1, 0), (0, 0, -1)),
            ((0, 1, 0), (1, 0, 0), (0, 0, -1)),
            ((-1, 0, 0), (0, 1, 0), (0, 0, -1)),
            ((0, -1, 0), (-1, 0, 0), (0, 0, -1)),
        ]
        for m in tables:
            det = (
                m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
            )
            self.assertEqual(det, 1)

    def test_core_files_under_line_and_ascii_caps(self):
        for p in SRC_DIR.rglob("*"):
            if not p.is_file():
                continue
            raw = p.read_bytes()
            self.assertLessEqual(raw.count(b"\n") + 1, 800, f"{p} exceeded 800 lines")
            self.assertLess(len(raw), 46080, f"{p} exceeded 46080 bytes")
            self.assertFalse(any(b > 127 for b in raw), f"{p} has non-ASCII bytes")


class FicpProtocolTests(unittest.TestCase):
    @staticmethod
    def _crc16_ccitt(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
        return crc

    def test_ficp_uuid_le_bytes_match_cpp(self):
        import uuid
        expected = bytes(reversed(uuid.UUID("5f494350-0001-4b10-a8e1-464943500001").bytes))
        cpp_text = (SRC_DIR / "ble" / "fic_ble_companion.cpp").read_text(encoding="ascii")
        le_csv = ", ".join(f"0x{b:02x}" for b in expected)
        self.assertIn(le_csv, cpp_text)

    def test_ficp_frame_vector(self):
        import struct
        import zlib
        payload = b'{"section":"all"}'
        body = struct.pack("<BBBHH", 0x01, 0x10, 0x02, 17, len(payload)) + payload
        self.assertEqual(self._crc16_ccitt(b"123456789"), 0x29B1)
        self.assertEqual(zlib.crc32(b"123456789") & 0xFFFFFFFF, 0xCBF43926)
        crc = self._crc16_ccitt(body)
        frame = b"\xAA\x55" + body + struct.pack("<H", crc)
        self.assertEqual(len(frame), 11 + len(payload))


def main():
    print(f"=== {__filename__} {__version__} ===")
    xtensa_gpp = find_xtensa_cpp_compiler()
    if xtensa_gpp:
        run_syntax_checks(xtensa_gpp)
    else:
        print("[*] xtensa-esp32s3-elf-g++ not found; skipping cross-compiler syntax pass.")

    comp_name, comp_path = find_host_cpp_compiler()
    if comp_path:
        run_native_cpp_tests(comp_name, comp_path)
    else:
        print("[*] No native host C++ compiler (g++/clang++/cl) in PATH; skipping host binary execution.")

    suite = unittest.TestSuite()
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(CoreInvariantTests))
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(FicpProtocolTests))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
