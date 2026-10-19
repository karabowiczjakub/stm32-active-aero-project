from pathlib import Path
import tempfile
import unittest
from test_lap_sim import ROOT, compile_host, run


class FailsafeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="failsafe-host-")
        cls.directory = Path(cls.temp.name)
        cls.executable = compile_host(cls.directory / "normal", 0,
                                      test_source=ROOT / "tests/host/test_failsafe.c")

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def case(self, name):
        output = run([self.executable, name])
        self.assertIn(f"PASS: {name}", output)
        return output

    def test_no_obd_since_boot(self):
        output = self.case("no_obd")
        self.assertEqual(output.count("FAILSAFE flags="), 1)

    def test_obd_lost_in_normal(self): self.case("obd_normal")
    def test_obd_lost_in_open(self): self.case("obd_open")
    def test_obd_lost_in_air_brake(self): self.case("obd_brake")
    def test_stale_can_frame(self): self.case("queue_stale")
    def test_stale_can_frame_after_open(self): self.case("stale_frame_in_open")
    def test_can_freshness_limit_is_inclusive(self): self.case("queue_limit")
    def test_can_timestamp_is_preserved_in_regression(self): self.case("queued_timestamps")
    def test_no_imu_since_boot(self): self.case("no_imu")
    def test_imu_lost_on_straight(self): self.case("imu_straight")
    def test_imu_lost_in_corner(self): self.case("imu_corner")
    def test_invalid_regression_in_open(self): self.case("invalid_open")
    def test_invalid_regression_in_air_brake(self): self.case("invalid_brake")
    def test_open_confirmation_gap_with_fresh_sources(self): self.case("confirmation_gap")
    def test_open_confirmation_across_obd_failure(self): self.case("confirmation_long_gap")
    def test_obd_recovery_rebuilds_regression_and_reconfirms_open(self):
        output = self.case("recovery_obd")
        self.assertEqual(output.count("FAILSAFE flags=0\n"), 2)
    def test_imu_recovery_rebuilds_regression_and_reconfirms_open(self): self.case("recovery_imu")
    def test_imu_recovery_resets_old_corner_samples(self): self.case("imu_corner_recovery")
    def test_tick_wrap(self):
        for case in ("obd_wrap", "imu_wrap", "queue_wrap_fresh", "queue_wrap_stale", "recovery_wrap"):
            with self.subTest(case=case): self.case(case)

    def test_configurable_freshness_limits(self):
        executable = compile_host(self.directory / "limits", 0,
                                  test_source=ROOT / "tests/host/test_failsafe.c",
                                  defines=("-DAPP_OBD_SPEED_MAX_AGE_MS=700U", "-DAPP_IMU_MAX_AGE_MS=140U"))
        for case in ("obd_open", "imu_straight", "queue_limit", "queue_stale", "recovery_obd"):
            self.assertIn(f"PASS: {case}", run([executable, case]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
