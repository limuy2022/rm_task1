"""Measured-config gate plus real launch introspection under ROS 2 Humble."""
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
import yaml
from launch.launch_context import LaunchContext
from launch_ros.actions import Node as LaunchNodeAction
from launch_ros.utilities import evaluate_parameters
from verify_timing_offline import ROOT
from verify_kinematics_offline import calibration
for pkg in ('gimbal_tracker_bringup','gimbal_localization','gimbal_common'):
    sys.path.insert(0,str(ROOT/'src'/pkg))
from gimbal_tracker_bringup.deployment_config import load_vehicle_config

LAUNCH_PATH = ROOT/'src/gimbal_tracker_bringup/launch/processing.launch.py'


def setup_actions(args):
    """Run the real ``_setup`` OpaqueFunction with a filled LaunchContext."""
    spec = importlib.util.spec_from_file_location('processing_launch_under_test', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(args)
    opaque = next(entity for entity in description.entities
                  if type(entity).__name__ == 'OpaqueFunction')
    return opaque._OpaqueFunction__function(context)


def node_executable(action):
    return action._Node__node_executable


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
            actions = setup_actions(args)
            nodes = [a for a in actions if isinstance(a, LaunchNodeAction)]
            executables = [node_executable(a) for a in nodes]
            self.assertIn('target_estimator_node',executables);self.assertIn('gimbal_kinematics_node',executables)
            self.assertNotIn('judge_io_node',executables)
            self.assertFalse(any(x and ('bridge' in x or 'controller' in x or 'driver' in x) for x in executables))
            args['submission']='true'
            # team_id 必须以字符串原样透传（'0047'/'true' 不得被类型吞掉）
            for team_id in ('47','0047','true'):
                with self.subTest(team_id=team_id):
                    args['team_id']=team_id
                    actions = setup_actions(args)
                    judge = next(a for a in actions if isinstance(a, LaunchNodeAction)
                                 and node_executable(a)=='judge_io_node')
                    parameters = evaluate_parameters(
                        LaunchContext(), list(judge._Node__parameters))[-1]
                    self.assertIsInstance(parameters['judge.team_id'],str)
                    self.assertEqual(parameters['judge.team_id'],team_id)
                    self.assertFalse(parameters['judge.fresh_run'])


if __name__ == '__main__': unittest.main(verbosity=2)
