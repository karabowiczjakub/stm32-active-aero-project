from pathlib import Path
import tempfile
import unittest
import subprocess
import re
from test_lap_sim import ROOT, compile_host, run


class UartLogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="uart-log-host-")
        cls.directory = Path(cls.temp.name)
        cls.images = {}
        for mode in (0, 1):
            for capture in (False, True):
                cls.images[mode, capture] = compile_host(
                    cls.directory / f"{mode}-{capture}", mode,
                    test_source=ROOT / "tests/host/test_uart_log.c", uart_capture=capture,
                    motionfx=True)

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def case(self, name):
        output = run([self.images[0, True], name])
        self.assertIn(f"PASS: {name}", output)

    def test_split_line(self): self.case("fragments")
    def test_printf_fragments(self): self.case("printf_fragments")
    def test_dma_buffer_is_immutable(self): self.case("immutable")
    def test_sequential_and_no_double_start(self): self.case("sequential")
    def test_full_queue_drops_entire_line_and_recovers(self): self.case("overflow")
    def test_oversized_line_drops_once(self): self.case("oversized")
    def test_maximum_line(self): self.case("exact_limit")
    def test_queue_wrap_keeps_order(self): self.case("ring_wrap")
    def test_hal_busy_retries_without_loss(self): self.case("busy")
    def test_waits_for_hardware_readiness_without_spinning(self): self.case("hardware_busy")
    def test_dma_complete_is_not_uart_complete(self): self.case("dma_then_uart")
    def test_late_duplicate_callbacks(self): self.case("late_callback")
    def test_start_error(self): self.case("start_error")
    def test_error_callback(self): self.case("callback_error")
    def test_abort_retains_buffer_until_ready(self): self.case("abort_pending")
    def test_failed_abort(self): self.case("abort_failure")
    def test_orphan_dma_and_failed_dma_abort(self): self.case("orphan_dma")
    def test_idle_error_does_not_discard_unsent_line(self): self.case("idle_error")
    def test_primask_is_preserved(self): self.case("primask")
    def test_completion_at_critical_section_boundary(self): self.case("completion_race")
    def test_other_uart_callbacks_are_ignored(self): self.case("wrong_uart")
    def test_write_arguments_and_stderr(self): self.case("arguments")
    def test_tick_wrap(self): self.case("tick_wrap")

    def test_complete_normal_log_stream_is_unchanged(self):
        direct = subprocess.run([self.images[0, False], "integration"],
                                capture_output=True, check=True).stdout
        dma = subprocess.run([self.images[0, True], "integration"],
                             capture_output=True, check=True).stdout
        self.assertEqual(dma, direct)
        for prefix in ("VEH ", "IMU ", "FX ", "FAILSAFE ", "SERVO STATE"):
            self.assertIn(prefix.encode(), dma)

    def test_complete_simulator_log_stream_is_unchanged(self):
        direct = subprocess.run([self.images[1, False], "integration"],
                                capture_output=True, check=True).stdout
        dma = subprocess.run([self.images[1, True], "integration"],
                             capture_output=True, check=True).stdout
        self.assertEqual(dma, direct)
        for prefix in ("SIM_HEADER,", "SIM_START", "SIM,", "SIM_END,104500"):
            self.assertIn(prefix.encode(), dma)

    def test_stalled_uart_does_not_stop_simulator(self):
        output = run([self.images[1, True], "integration_stall"])
        self.assertIn("PASS: integration_stall", output)

    def test_stalled_uart_does_not_stop_real_control(self):
        output = run([self.images[0, True], "integration_stall"])
        self.assertIn("PASS: integration_stall", output)

    def test_capacity_covers_existing_logger_with_crlf(self):
        logger = (ROOT / "Core/Src/diag_log.c").read_text()
        header = (ROOT / "Core/Inc/uart_log.h").read_text()
        log_size = int(re.search(r"#define DIAG_LOG_LINE_SIZE (\d+)", logger)[1])
        slot_size = int(re.search(r"#define UART_LOG_LINE_SIZE (\d+)", header)[1])
        self.assertGreaterEqual(slot_size, log_size - 1 + 2)


if __name__ == "__main__": unittest.main(verbosity=2)
