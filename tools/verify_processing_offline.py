"""Measured-config gate and actual launch factory, using launch stand-ins only."""
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import yaml
from verify_timing_offline import load_ast, ROOT
from verify_kinematics_offline import calibration
for pkg in ('gimbal_tracker_bringup','gimbal_localization','gimbal_common'):
    sys.path.insert(0,str(ROOT/'src'/pkg))
from gimbal_tracker_bringup.deployment_config import load_vehicle_config


class ProcessingTests(unittest.TestCase):
    def test_unmounted_template_cannot_become_vehicle_identity_calibration(self):
        with self.assertRaisesRegex(ValueError, '未确认'):
            load_vehicle_config(ROOT/'src/gimbal_tracker_bringup/config/vehicle_template.yaml')

    def test_processing_has_no_serial_drivers_control_and_submission_opt_in(self):
        data = yaml.safe_load((ROOT/'src/gimbal_tracker_bringup/config/vehicle_template.yaml').read_text())
        p = data['/**']['ros__parameters']
        p.update({'kinematics.'+k:v for k,v in calibration().items()})
        p.update({'kinematics.calibrated_epoch':7,'estimation.geometry_confirmed':True,
                  'localization.sensor_extrinsics_confirmed':True,
                  'localization.parent_transform_confirmed':True})
        with tempfile.TemporaryDirectory() as directory:
            cfg = Path(directory)/'synthetic_config.yaml'; cfg.write_text(yaml.safe_dump(data))
            camera = ROOT/'src/gimbal_tracker_bringup/config/camera_team47.yaml'
            args = dict(vehicle_config=str(cfg),camera_config=str(camera),target_id='-1',submission='false',
                        team_id='47',output_dir=str(Path(directory)/'test_output'))
            env = load_ast('src/gimbal_tracker_bringup/launch/processing.launch.py', {'_setup'},
                dict(os=os,load_vehicle_config=load_vehicle_config,
                     LaunchConfiguration=lambda name:NS(perform=lambda context:args[name]),
                     Node=lambda **kw:kw, LogInfo=lambda **kw:kw))
            actions = env['_setup'](None)
            executables = [a.get('executable') for a in actions]
            self.assertIn('target_estimator_node',executables);self.assertIn('gimbal_kinematics_node',executables)
            self.assertNotIn('judge_io_node',executables)
            self.assertFalse(any(x and ('bridge' in x or 'controller' in x or 'driver' in x) for x in executables))
            args['submission']='true'
            actions = env['_setup'](None)
            judge = next(a for a in actions if a.get('executable')=='judge_io_node')
            self.assertEqual(judge['parameters'][-1]['judge.team_id'],'47')
            self.assertFalse(judge['parameters'][-1]['judge.fresh_run'])


if __name__ == '__main__': unittest.main(verbosity=2)
