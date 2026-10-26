"""Host integration tests of the production C sources, without ARM hardware."""
import csv
import hashlib
import importlib.util
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("lap_generator", ROOT / "tools/csv_to_lap_data.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


def run(command):
    result = subprocess.run([str(arg) for arg in command], cwd=ROOT,
                            text=True, capture_output=True)
    if result.returncode:
        raise AssertionError(f"command failed ({result.returncode}): {command}\n"
                             f"{result.stdout[-4000:]}\n{result.stderr[-4000:]}")
    return result.stdout


def compile_host(directory, mode, source_root=ROOT, test_source=None, defines=(), motionfx=False,
                 uart_capture=False):
    """Use the exact main-loop body/callbacks from main.c, replacing only its boot."""
    directory.mkdir(parents=True, exist_ok=True)
    main = (source_root / "Core/Src/main.c").read_text()
    pv = main.split("/* USER CODE BEGIN PV */", 1)[1].split("/* USER CODE END PV */", 1)[0]
    loop = main.split("while (1)", 1)[1].split("/* USER CODE END WHILE */", 1)[0].strip()
    assert loop.startswith("{")
    loop = loop[1:]
    timer = main.split("/* USER CODE BEGIN 4 */", 1)[1].split("/* USER CODE END 4 */", 1)[0]
    button = main.split("void BSP_PB_Callback(Button_TypeDef Button)", 1)[1].split("/**", 1)[0]
    boot = main.split("/* USER CODE BEGIN BSP */", 1)[1].split("/* USER CODE END BSP */", 1)[0]
    generated = directory / "app_loop.c"
    generated.write_text('#include "main.h"\n#include "app_config.h"\n#include "imu.h"\n'
                         '#include "wing_control.h"\n#include "obd.h"\n#include "servo.h"\n'
                         '#include "diag_log.h"\n#include "lap_sim.h"\n#include "uart_log.h"\n'
                         '#include "motion_fx_app.h"\n'
                         + pv + "\nvoid Host_MainLoopOnce(void) {\n"
                         + ("Host_UART_Poll();\n" if uart_capture else "")
                         + loop + "\n}\n"
                         + timer + "\nvoid BSP_PB_Callback(Button_TypeDef Button)" + button
                         + "\nvoid Host_Boot(void) {\n" + boot + "\n}\n")
    production = []
    for name in ("wing_control", "imu", "servo", "debug_utils", "diag_log", "obd", "lap_sim", "lap_data", "uart_log"):
        local = source_root / f"Core/Src/{name}.c"
        production.append(local if local.exists() else ROOT / f"Core/Src/{name}.c")
    if motionfx:
        production += [ROOT / "Core/Src/motion_fx_app.c", ROOT / "tests/host/motion_fx.c"]
        defines = (*defines, "-DHOST_REAL_MOTIONFX=1", "-I" + str(ROOT / "MotionFX/Inc"))
    if uart_capture:
        production.append(ROOT / "tests/host/uart_capture.c")
        defines = (*defines, "-Wl,--wrap=printf", "-Wl,--wrap=__printf_chk",
                   "-Wl,--wrap=puts", "-DHOST_UART_CAPTURE=1")
    executable = directory / "controller"
    run(["gcc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
         "-Wno-unused-function", "-Wno-comment", "-fsanitize=undefined,float-cast-overflow",
         "-fno-sanitize-recover=all", f"-DAPP_LAP_SIM_ENABLED={mode}", *defines,
         "-I", ROOT / "tests/host", "-I", source_root / "Core/Inc", "-I", ROOT / "Core/Inc",
         *production, ROOT / "tests/host/hardware.c", test_source or ROOT / "tests/host/test_controller.c",
         generated, "-lm", "-o", executable])
    return executable


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="lap-host-")
        cls.directory = Path(cls.temp.name)
        cls.sim = compile_host(cls.directory / "sim", 1)
        cls.normal = compile_host(cls.directory / "normal", 0)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def case(self, name, sim=True):
        output = run([self.sim if sim else self.normal, name])
        self.assertIn(f"PASS: {name}", output)
        return output

    def test_regression_and_air_brake(self): self.case("regression")
    def test_open_and_speed_hysteresis(self): self.case("hysteresis")
    def test_brake_end_hold_and_reconfirmation(self): self.case("brake_end")
    def test_brake_hysteresis_and_minimum_speed(self): self.case("brake_hysteresis")
    def test_corner_filter_thresholds_and_priority(self): self.case("corner")
    def test_physical_imu_isolation_and_motionfx_inputs(self): self.case("physical_isolation")
    def test_small_delay_preserves_sample_timestamps(self):
        delayed = self.case("small_delay")
        nominal = self.case("no_delay")
        self.assertEqual([s for s in delayed.splitlines() if s.startswith("DELAY_TRACE,")],
                         [s for s in nominal.splitlines() if s.startswith("DELAY_TRACE,")])

    def test_timing_limit_is_inclusive(self): self.case("timing_limit")
    def test_large_delay_does_not_replay_history(self): self.case("large_delay")

    def test_full_lap_button_counts_end_no_obd_and_log_budget(self):
        output = self.case("full_lap")
        self.assertNotIn("VEH ", output)
        self.assertNotIn("IMU t_ms=", output)
        self.assertNotIn("FX t_ms=", output)
        lines = [line for line in output.splitlines() if line.startswith("SIM,")]
        self.assertGreater(len(lines), 1000)
        self.assertLess(len(lines), 1500)
        for line in lines:
            fields = next(csv.reader([line]))
            self.assertEqual(len(fields), 11)
            self.assertLessEqual(len(line) + 2, 80)  # <= 1.74 ms at 460800 8N1
            self.assertEqual(fields[6] == "0", fields[5] == "nan")
        self.assertIn("SIM_END,104500", output)

    def test_tick_wrap(self): self.case("tick_wrap")

    def test_timing_error_stops_before_stale_servo_decisions(self):
        output = self.case("timing_error")
        self.assertIn("SIM_TIMING_ERROR,", output)
        self.assertNotIn("SIM_END,", output)

    def test_normal_mode_obd_imu_diagnostics(self):
        output = self.case("normal_trace", sim=False)
        self.assertIn("VEH ", output)
        self.assertIn("IMU t_ms=", output)
        self.assertIn("FX t_ms=", output)
        self.assertIn("CORNER DETECTED", output)
        symbols = run(["nm", self.normal])
        self.assertNotRegex(symbols, r"\b(?:LapSim_\w+|lap_data)\b")
        self.assertRegex(symbols, r"\bControl_Reset\b")

    def test_main_source_selection(self):
        for mode in (0, 1):
            output = run(["gcc", "-E", "-P", f"-DAPP_LAP_SIM_ENABLED={mode}",
                          "-I", ROOT / "tests/host", "-I", ROOT / "Core/Inc",
                          ROOT / "Core/Src/main.c"])
            main = output.split("int main(void)", 1)[1].split("void SystemClock_Config(void)\n{", 1)[0]
            for call in ("CAN_Test_Init();", "CAN_Process_Rx();", "CAN_Send_OBD_Request(0x0D);"):
                self.assertEqual(call in main, mode == 0)

    def test_normal_mode_matches_original_firmware_trace(self):
        output = run([self.normal, "normal_trace"])
        def trace(text):
            return "\n".join(line for line in text.splitlines() if line.startswith("TRACE,")) + "\n"

        # 1001 snapshotow sprzed zmian, bez dodatkowych komunikatow failsafe
        self.assertEqual(len(trace(output).splitlines()), 1001)
        self.assertEqual(hashlib.sha256(trace(output).encode()).hexdigest(),
                         "5f2fc19d98bec7cbfef8806d3671314e927d1d55057ed747d7338d33106ddd4d")
        if os.environ.get("LAP_BASELINE_DIR"):
            original = compile_host(self.directory / "baseline", 0, Path(os.environ["LAP_BASELINE_DIR"]))
            self.assertEqual(trace(run([original, "normal_trace"])), trace(output))


class GeneratorTests(unittest.TestCase):
    def test_complete_reproducible_table_and_unchanged_csv(self):
        path = ROOT / "data/telemetry_50hz.csv"
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                         "e6dec04ff179c46385da9b857e42a40a81687b6e39a94030ae7d01bb0b5c5ecd")
        for relative, contents in generator.generate(path).items():
            self.assertEqual((ROOT / relative).read_text(), contents)
        source = (ROOT / "Core/Src/lap_data.c").read_text()
        rows = re.findall(r"\{(\d+)U, (-?\d+), (-?\d+), (\d+)U, (\d+)U\}", source)
        with path.open() as stream:
            original = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 5226)
        for generated, sample in zip(rows, original):
            t, lat, lon, speed, quality = map(int, generated)
            self.assertEqual(t, int(sample["t_ms"]))
            self.assertEqual(lat, int(generator.Decimal(sample["g_lateral_est"])*1000))
            self.assertEqual(lon, int(generator.Decimal(sample["g_longitudinal_est"])*1000))
            self.assertEqual(speed, int(sample["speed_kmh"]))
            self.assertEqual(quality, (sample["speed_quality"] == "interpolated") |
                             ((sample["g_quality"] == "extended_roi") << 1))
        self.assertEqual(int(rows[-1][0]), 104500)
        self.assertEqual(min(int(row[1]) for row in rows), -2101)

    def test_rejects_bad_timestamps_values_and_quality(self):
        source = ROOT / "data/telemetry_50hz.csv"
        with source.open() as stream:
            initial = list(csv.DictReader(stream))[:2]
        variants = [("t_ms", "0"), ("speed_kmh", "256"), ("speed_kmh", "55.5"),
                    ("g_lateral_est", "NaN"), ("g_longitudinal_est", "40"),
                    ("a_lateral_mps2_est", "999"), ("speed_quality", "unknown"),
                    ("g_quality", "unknown")]
        with tempfile.TemporaryDirectory(prefix="lap-csv-") as directory:
            path = Path(directory) / "invalid.csv"
            for key, value in variants:
                with self.subTest(column=key, value=value):
                    rows = [dict(row) for row in initial]
                    rows[1][key] = value
                    with path.open("w", newline="") as stream:
                        writer = csv.DictWriter(stream, fieldnames=generator.COLUMNS)
                        writer.writeheader()
                        writer.writerows(rows)
                    with self.assertRaises(ValueError): generator.generate(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
