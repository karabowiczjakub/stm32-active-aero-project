from pathlib import Path
import hashlib
import re
import subprocess
import tempfile
import unittest
from test_lap_sim import ROOT, compile_host, run


class WingModeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(prefix='hybrid-tests-')
        cls.directory=Path(cls.temp.name)
        cls.images={}
        for name,defines in {'only':('-DAPP_WING_CONTROL_MODE=0',),
                             'shadow':('-DAPP_WING_CONTROL_MODE=1',),
                             'active':('-DAPP_WING_CONTROL_MODE=2',),
                             'default':(),
                             'api_fx':('-DAPP_WING_CONTROL_MODE=1', '-DAPP_ACC_SOURCE=ACC_SOURCE_MOTIONFX'),
                             'unavailable':('-DAPP_WING_CONTROL_MODE=1', '-DMOTIONFX_DIAG_ENABLE=0')}.items():
            cls.images[name]=compile_host(cls.directory/name,0,
                test_source=ROOT/'tests/host/test_hybrid.c',motionfx=True,
                defines=(*defines,'-Wl,--wrap=MotionFxApp_GetLatest'))
        cls.images['uart']=compile_host(cls.directory/'uart',0,
            test_source=ROOT/'tests/host/test_hybrid.c',motionfx=True,uart_capture=True,
            defines=('-DAPP_WING_CONTROL_MODE=1', '-Wl,--wrap=MotionFxApp_GetLatest','-DHOST_COMPARE_MAX=1',
                     '-Wl,--wrap=Control_GetCompareSnapshot','-Wl,--wrap=UARTLog_GetDiagSnapshot'))
        cls.reference={mode:compile_host(cls.directory/f'reference-{mode}',0,
            test_source=ROOT/'tests/host/test_legacy_equivalence.c',motionfx=True,
            defines=(f'-DAPP_WING_CONTROL_MODE={mode}',)) for mode in [0,1]}
        cls.sim={mode:compile_host(cls.directory/f'sim-{mode}',1,
            defines=(f'-DAPP_WING_CONTROL_MODE={mode}',)) for mode in range(3)}

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def case(self,name,variants=('shadow','active')):
        for mode in variants:
            with self.subTest(mode=mode):
                output=run([self.images[mode],name]);self.assertIn('PASS: '+name,output)
        return output

    def test_legacy_matches_unchanged_reference(self):
        for mode in [0,1]:
            for clock in ['normal','wrap']:
                text=run([self.reference[mode],clock])
                lines=[s for s in text.splitlines() if s.startswith('EQUIV,')]
                self.assertEqual(len(lines),6001)
                self.assertEqual(hashlib.sha256(('\n'.join(lines)+'\n').encode()).hexdigest(),
                                 '4224581808d6d0234e3dc273eb56e2350b8d8ae8f386d7a8cff442c42b3df951')

    def test_early_entry(self): self.case('early')
    def test_obd_takeover_and_vanishing_fx(self): self.case('takeover')
    def test_bounded_wait_and_rearm(self): self.case('rejection')
    def test_single_sample_rejected(self): self.case('pulse')
    def test_short_confirmation_rejected(self): self.case('short_series')
    def test_duplicate_samples(self): self.case('duplicate')
    def test_confirmation_uses_real_time(self): self.case('interval')
    def test_missing_sample_breaks_confirmation(self): self.case('gap')
    def test_independent_obd_entry(self): self.case('obd_only')
    def test_missing_fx_does_not_disable_obd(self): self.case('missing_fx',('unavailable',))
    def test_fx_loss_revokes_unconfirmed_entry(self): self.case('fx_wait_loss')
    def test_imu_loss_while_waiting(self): self.case('imu_wait_loss')
    def test_speed_gate_preempts_wait(self): self.case('speed')
    def test_fx_loss_after_takeover_keeps_obd(self): self.case('fx_loss_confirmed')
    def test_nan_after_takeover(self): self.case('nan')
    def test_inf_after_takeover(self): self.case('inf')
    def test_negative_inf_after_takeover(self): self.case('ninf')
    def test_nan_while_waiting(self): self.case('wait_nan')
    def test_inf_while_waiting(self): self.case('wait_inf')
    def test_raw_corner_has_priority_in_both_paths(self): self.case('raw_corner')
    def test_fx_lateral_does_not_control_corner(self): self.case('fx_lateral')
    def test_obd_loss(self): self.case('obd_loss')
    def test_imu_loss(self): self.case('imu_loss')
    def test_return_to_normal_and_post_brake_open_lock(self): self.case('end')
    def test_brief_release_does_not_fragment_braking(self): self.case('partial')
    def test_old_fx_tail_does_not_retrigger(self): self.case('tail')
    def test_no_repeated_off_confirmation_from_same_obd(self): self.case('off_duplicate')
    def test_acceleration_end_veto(self): self.case('accel_veto')
    def test_tick_wrap(self): self.case('wrap')
    def test_sequence_wrap(self): self.case('sequence_wrap')
    def test_ten_minutes_of_repeated_maneuvers(self): self.case('long_run')
    def test_routing_in_every_mode(self): self.case('mode',('only','shadow','active','default','api_fx'))

    def test_lap_sim_preserved_in_every_mode(self):
        for executable in self.sim.values():
            self.assertIn('SIM_END,104500',run([executable,'full_lap']))
            self.assertIn('SIM_TIMING_ERROR,',run([executable,'timing_error']))

    def test_uart_records_and_budget(self):
        text=self.case('logger',('uart',))
        records=[line for line in text.splitlines() if line.startswith('CTRL_CMP ')]
        self.assertGreater(len(records),20)
        for line in records:
            self.assertLess(len(line)+2,640)
            fields=dict(p.split('=',1) for p in line.split()[1:])
            self.assertEqual(len(fields),27)
            self.assertIn('wing_hybrid',fields)
            self.assertEqual(fields['uart_drop'],'0')
        total=sum(len(line)+2 for line in text.splitlines() if not line.startswith('PASS:'))
        self.assertLess(total/2.6,460800/10*.65)
        self.assertIn('reason=obd_takeover',text)
        self.assertNotRegex(text,r'(?i)(?:=|,)[+-]?(?:nan|inf)\b')

    def test_invalid_configuration_rejected(self):
        for define in ['APP_WING_CONTROL_MODE=3','HYBRID_FX_ON_MG=-10',
                       'HYBRID_FX_ON_CONFIRM_MS=0','HYBRID_OBD_CONFIRM_MS=0x80000000U']:
            result=subprocess.run(['gcc','-E','-x','c','-D'+define,'-I',str(ROOT/'Core/Inc'),'-'],
                input='#include "app_config.h"\n',text=True,capture_output=True)
            self.assertNotEqual(result.returncode,0)

    def test_maximum_comparison_record_fits(self):
        text=self.case('logger_maximum',('uart',))
        line=next(s for s in text.splitlines() if s.startswith('CTRL_CMP t_ms=4294967295 '))
        self.assertLess(len(line)+2,640)
        self.assertTrue(line.endswith('uart_q=16'))
        self.assertIn('rejected=4294967295',line)
        self.assertEqual(len(line.split())-1,27)


if __name__=='__main__':unittest.main(verbosity=2)
