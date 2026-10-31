"""现场只读检查、标定比较与参考 PnP 入口回归；不访问设备或 DDS。"""
from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch, Mock

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]

def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

inspect=load('inspect_nuc')
calib=load('compare_calibrations')
from team43_perception.pnp_solver import PnpSolver, marker_object_points

OURS=ROOT/'src/team43_bringup/config/camera_team43.yaml'
OTHER=ROOT/'src/team43_bringup/config/camera_reference_other_team.yaml'

class FieldTests(unittest.TestCase):
    def test_symlink_states_without_device_open(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            (root/'missing_link').symlink_to(root/'absent')
            (root/'regular').write_text('not a tty')
            (root/'regular_link').symlink_to(root/'regular')
            with patch('builtins.open',side_effect=AssertionError('device open forbidden')):
                self.assertEqual(inspect.inspect_serial_alias(root/'absent')['status'],'missing')
                self.assertEqual(inspect.inspect_serial_alias(root/'missing_link')['status'],'dangling')
                self.assertEqual(inspect.inspect_serial_alias(root/'regular_link')['status'],'not_character_device')

    def test_valid_character_metadata_does_not_prove_protocol(self):
        p=Path('/fictional/gimbal')
        mode=NS(st_mode=stat.S_IFCHR|0o660,st_rdev=os.makedev(4,64))
        with patch.object(Path,'lstat',return_value=mode),patch.object(Path,'resolve',return_value=p),patch.object(Path,'stat',return_value=mode):
            r=inspect.inspect_serial_alias(p)
        self.assertEqual(r['status'],'character_device')
        self.assertFalse(r['opens_serial_device'])
        self.assertEqual(r['device_major'],4)

    def test_ros_type_support_and_expected_prefix(self):
        checker=Mock()
        modules={
            'ament_index_python.packages':NS(get_package_prefix=lambda name:'/team43/install/'+name),
            'rosidl_runtime_py.utilities':NS(get_message=lambda name:type(name.split('/')[-1],(),{})),
            'rclpy.type_support':NS(check_for_type_support=checker)}
        with patch.object(inspect.importlib,'import_module',side_effect=modules.__getitem__):
            r=inspect.inspect_ros(Path('/team43/install'))
        self.assertEqual(r['status'],'type_support_ok')
        self.assertTrue(r['prefix_in_expected_install'])
        self.assertEqual(checker.call_count,2)

    def test_ros_import_failure_preserves_cause(self):
        with patch.object(inspect.importlib,'import_module',side_effect=ImportError('missing generated type support')):
            r=inspect.inspect_ros()
        self.assertEqual(r['status'],'import_failed')
        self.assertIn('missing generated type support',r['traceback'])

    def test_calibration_uses_raw_k_instead_of_projection_p(self):
        c=calib.read_calibration(OTHER)['camera']
        data={'image_width':1440,'image_height':1080,'distortion_model':'plumb_bob',
              'camera_matrix':{'data':[c['fx'],0,c['cx'],0,c['fy'],c['cy'],0,0,1]},
              'distortion_coefficients':{'data':c['distortion']},
              'projection_matrix':{'data':[924,0,691,0,0,927,581,0,0,0,1,0]}}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'ost.txt';p.write_text(yaml.safe_dump(data))
            r=calib.read_calibration(p)
        self.assertEqual(r['camera'],c)
        self.assertTrue(r['projection_matrix_present'])

    def test_compare_never_selects_unknown_hardware(self):
        r=calib.compare([OURS,OTHER])
        self.assertTrue(r['same_resolution'])
        self.assertIsNone(r['selected_calibration'])
        self.assertAlmostEqual(r['first_over_second_focal_ratio'][0],1.780,places=2)

    def test_wrong_focal_length_can_have_small_reprojection_error(self):
        # A known synthetic camera, NOT a claim about either real calibration.
        c=calib.read_calibration(OTHER)['camera']
        k=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
        uv,_=cv2.projectPoints(marker_object_points(.10),np.array([math.pi,0.,0.]),np.array([0.,0.,2.]),k,np.array(c['distortion']))
        r=calib.compare([OURS,OTHER],uv.reshape(4,2).tolist(),marker_size=.10)
        ours,correct=[v['pnp'] for v in r['calibrations']]
        self.assertTrue(ours['valid'])
        self.assertTrue(correct['valid'])
        self.assertLess(ours['reprojection_error_px'],2)
        self.assertGreater(abs(ours['xyz_m'][2]-2),.5)
        self.assertAlmostEqual(correct['xyz_m'][2],2,places=5)

    def test_comparison_entry_maps_output_and_keeps_threshold(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);capture=d/'args';fake=d/'ros2'
            fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$CAPTURE"\n')
            fake.chmod(0o755)
            env=dict(os.environ,PATH=str(d)+os.pathsep+os.environ['PATH'],CAPTURE=str(capture))
            subprocess.run(['bash',str(ROOT/'tools/run_reference_pnp.bash'),str(OTHER),'3'],env=env,check=True)
            args=capture.read_text().splitlines()
            self.assertIn('/perception/marker_cam:=/team43_check/reference/marker_cam',args)
            self.assertIn('perception.reproj_max_px:=2.0',args)
            self.assertIn('perception.marker_corners_topic:=/team43_check/marker_corners',args)
            self.assertNotIn('team43_serial_bridge',args)
            bad=subprocess.run(['bash',str(ROOT/'tools/run_reference_pnp.bash'),str(OTHER),'50'],env=env,capture_output=True)
            self.assertEqual(bad.returncode,2)
