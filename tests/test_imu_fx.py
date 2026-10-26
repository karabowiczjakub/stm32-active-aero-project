from pathlib import Path
import tempfile
import unittest
from test_lap_sim import ROOT, compile_host, run


class ImuMotionFxTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(prefix="imu-fx-host-")
        cls.directory=Path(cls.temp.name)
        cls.normal=compile_host(cls.directory / "normal",0,
                                test_source=ROOT / "tests/host/test_imu_fx.c",motionfx=True,
                                defines=("-Wl,--wrap=MotionFxApp_GetLatest",))
        cls.sim=compile_host(cls.directory / "sim",1,
                             test_source=ROOT / "tests/host/test_imu_fx.c",motionfx=True,
                                defines=("-Wl,--wrap=MotionFxApp_GetLatest",))

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def case(self,name,sim=False):
        output=run([self.sim if sim else self.normal,name])
        self.assertIn(f"PASS: {name}",output)
        return output

    def test_normal_completion(self): self.case("normal_read")
    def test_i2c_error_callback(self): self.case("error_callback")
    def test_missing_status_callback(self): self.case("status_timeout")
    def test_missing_data_callback(self): self.case("data_timeout")
    def test_recovery_and_timeout_counter(self): self.case("recovery")
    def test_abort_failure_and_retry(self): self.case("failed_recovery")
    def test_abort_without_completion_blocks_new_dma(self): self.case("abort_never_completes")
    def test_late_and_duplicate_callbacks(self): self.case("late_callback")
    def test_late_completion_before_timeout_poll(self): self.case("late_before_poll")
    def test_orphan_dma_after_i2c_error(self): self.case("orphan_dma")
    def test_hardware_readiness_before_restart(self): self.case("hardware_not_ready")
    def test_completion_race_and_primask_restore(self): self.case("completion_race")
    def test_completed_sample_is_not_published_after_stale_delay(self): self.case("stale_sample")
    def test_logger_defends_against_invalid_api_payload(self):
        output=self.case("logger_defense")
        lines=[line for line in output.splitlines() if line.startswith("IMU_FX t_ms=")]
        self.assertEqual(len(lines),108)
        for line in lines:
            self.assertIn("valid=0 age_ms=0",line)
            self.assertNotRegex(line,r"(?i)(?:=|,)[+-]?(?:nan|inf)\b")
    def test_integral_range_does_not_poison_recovery(self): self.case("integral_range")
    def test_existing_failsafe_after_stall(self): self.case("failsafe_after_stall")
    def test_dma_tick_wrap(self): self.case("dma_wrap")
    def test_valid_motionfx_and_log_format(self):
        output=self.case("valid_fx")
        self.assertIn("IMU_FX t_ms=100 valid=1 age_ms=0 yaw_deg=+12.500",output)
        self.assertIn("IMU_FX t=100",output)
    def test_motionfx_nan_in_each_output(self): self.case("nan_fx")
    def test_motionfx_positive_infinity_in_each_output(self): self.case("inf_fx")
    def test_motionfx_negative_infinity_in_each_output(self): self.case("negative_inf_fx")
    def test_motionfx_finite_outside_logging_range(self): self.case("huge_fx")
    def test_motionfx_stale_and_recovery(self):
        output=self.case("stale_fx")
        self.assertIn("IMU_FX t_ms=121 valid=0 age_ms=101 yaw_deg=+0.000",output)
        self.assertNotIn("IMU_FX t=",output)
    def test_motionfx_invalid_inputs_and_integral_recovery(self): self.case("bad_inputs")
    def test_motionfx_invalid_delta_time(self): self.case("invalid_dt")
    def test_motionfx_tick_wrap(self): self.case("fx_wrap")
    def test_motionfx_failed_reinitialization_invalidates_old_result(self): self.case("init_failure")
    def test_invalid_motionfx_does_not_control_wing(self): self.case("fx_has_no_control")
    def test_simulator_with_disconnected_imu(self): self.case("sim_disconnected",sim=True)
    def test_simulator_with_stalled_abort(self): self.case("sim_stalled",sim=True)


if __name__=="__main__": unittest.main(verbosity=2)
