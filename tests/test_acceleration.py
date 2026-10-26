from pathlib import Path
import tempfile
import unittest
import subprocess
from test_lap_sim import ROOT, compile_host, run


class AccelerationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="acceleration-host-")
        cls.directory = Path(cls.temp.name)
        cls.images = {}
        variants = {
            "raw": (),
            "fx": ("-DAPP_ACC_SOURCE=ACC_SOURCE_MOTIONFX",),
            "off": ("-DMOTIONFX_FILTER_ENABLE=0",),
            "div2": ("-DMOTIONFX_FILTER_DIVISOR=2",),
            "unavailable": ("-DMOTIONFX_DIAG_ENABLE=0", "-DAPP_ACC_SOURCE=ACC_SOURCE_MOTIONFX"),
        }
        for name, defines in variants.items():
            wrappers = tuple(f"-Wl,--wrap={name}" for name in
                             ("OBD_GetDiagSnapshot", "IMU_GetDiagSnapshot",
                              "Control_GetDiagSnapshot", "MotionFxApp_GetLatest"))
            cls.images[name] = compile_host(
                cls.directory / name, 0,
                test_source=ROOT / "tests/host/test_acceleration.c", motionfx=True,
                uart_capture=True, defines=(*defines, *wrappers))

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def case(self, name, variant="raw"):
        output = run([self.images[variant], name])
        self.assertIn(f"PASS: {name}", output)
        return output

    def test_raw_mapping_freshness_and_recovery(self): self.case("raw_mapping")
    def test_motionfx_mapping_and_first_sample(self): self.case("fx_mapping")
    def test_independent_filters(self): self.case("filter")
    def test_disabled_filters(self): self.case("filter", "off")
    def test_configurable_divisor(self): self.case("filter", "div2")
    def test_raw_selected(self): self.case("selected_source")
    def test_motionfx_selected(self): self.case("selected_source", "fx")
    def test_stale_result_and_filter_recovery(self): self.case("stale_recovery")
    def test_invalid_results_and_mapping_range(self): self.case("invalid_recovery")
    def test_reinitialization_and_sample_identity(self): self.case("reinitialize")
    def test_duplicate_input_does_not_run_fusion(self): self.case("duplicate")
    def test_imu_task_does_not_repeat_sample(self): self.case("imu_once")
    def test_unavailable_motionfx_with_raw_selected(self): self.case("unavailable")
    def test_unavailable_motionfx_does_not_fall_back(self): self.case("unavailable", "fx")
    def test_compiled_out_motionfx(self): self.case("unavailable", "unavailable")
    def test_tick_wrap(self): self.case("tick_wrap")
    def test_control_with_raw_interface(self): self.case("control_unchanged")
    def test_control_with_motionfx_interface(self): self.case("control_unchanged", "fx")
    def test_control_without_motionfx(self): self.case("control_unchanged", "unavailable")

    def test_rejects_invalid_configuration(self):
        for define in ("APP_ACC_SOURCE=2", "MOTIONFX_FILTER_ENABLE=2", "MOTIONFX_FILTER_DIVISOR=0"):
            result = subprocess.run(["gcc", "-E", "-x", "c", "-D" + define,
                                     "-I", str(ROOT / "Core/Inc"), "-"],
                                    input='#include "app_config.h"\n', text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("#error", result.stderr)

    def test_maximum_diagnostic_records_fit_without_truncation(self):
        for variant in ("raw", "fx"):
            output = self.case("logger_maximum", variant)
            for line in output.splitlines():
                prefix = line.split(" ", 1)[0]
                if prefix not in ("VEH", "IMU", "IMU_FX"):
                    continue
                fields = dict(field.split("=", 1) for field in line.split()[1:])
                self.assertEqual(len(fields), {"VEH": 32, "IMU": 21, "IMU_FX": 19}[prefix])
                self.assertLess(len(line) + 2, 642)
                if prefix == "VEH": self.assertIn("acc_lat_mg", fields)
                if prefix == "IMU": self.assertEqual(fields["latFilt_mg"], str(-2**31))
                if prefix == "IMU_FX": self.assertEqual(fields["vert_mg"], "-2000000000")

    def test_diagnostic_units_fields_frequency_and_dma(self):
        for variant, source in (("raw", "RAW"), ("fx", "MFX")):
            with self.subTest(variant=variant):
                output = self.case("logger", variant)
                records = {}
                for line in output.splitlines():
                    prefix = line.split(" ", 1)[0]
                    if prefix not in ("VEH", "IMU", "IMU_FX"):
                        continue
                    fields = dict(field.split("=", 1) for field in line.split()[1:])
                    self.assertEqual(len(fields), len(line.split()) - 1)
                    self.assertLess(len(line) + 2, 672)
                    self.assertNotRegex(line, r"(?i)(?:=|,)[+-]?(?:nan|inf)\b")
                    records.setdefault(prefix, []).append(fields)
                self.assertEqual(set(records), {"VEH", "IMU", "IMU_FX"})
                for prefix, lines in records.items():
                    self.assertEqual([line["t_ms"] for line in lines], ["100", "201"])
                    self.assertEqual(len(lines[0]), len(lines[1]))
                    self.assertEqual(len(lines[0]), {"VEH": 32, "IMU": 21, "IMU_FX": 19}[prefix])
                self.assertEqual(records["VEH"][0]["acc_src"], source)
                self.assertEqual(records["IMU"][0]["longRaw_mg"], "61")
                self.assertEqual(records["IMU"][0]["latRaw_mg"], "-999")
                self.assertIn("latFilt_g", records["IMU"][0])
                first, stale = records["IMU_FX"]
                self.assertEqual(first["longRaw_mg"], "1000")
                self.assertEqual(first["latRaw_mg"], "-500")
                self.assertEqual(first["longFilt_mg"], "1000")
                self.assertEqual(first["latFilt_mg"], "-500")
                self.assertEqual(stale["valid"], "0")
                self.assertEqual(stale["age_ms"], "101")
                self.assertEqual(stale["sample_ms"], "100")
                self.assertEqual(stale["seq"], first["seq"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
