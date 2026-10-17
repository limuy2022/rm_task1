"""Offline save/commit/argument checks; does not run ROS, GUI, or calibration."""
from pathlib import Path
import importlib.util
import io
import json
import sys
import tarfile
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('entry',ROOT/'tools/run_camera_calibration.py')
entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)

class BaseCalibration:
    def do_save(self): raise AssertionError('must never save to global /tmp')

class Calibration(BaseCalibration):
    def do_tarfile_save(self,tf):
        for name,data in [('ost.yaml',b'image_width: 1440\n'),('ost.txt',b'example'),('left-0000.png',b'example_image')]:
            member=tarfile.TarInfo(name);member.size=len(data);tf.addfile(member,io.BytesIO(data))

class GUI:
    def do_upload(self): raise AssertionError('SetCameraInfo must not be called')

class Tests(unittest.TestCase):
    def test_save_unique_in_team_dir_and_preserve_previous(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);first=entry.create_archive(Calibration(),d)
            before=first.read_bytes();second=entry.create_archive(Calibration(),d)
            self.assertNotEqual(first,second);self.assertEqual(before,first.read_bytes())
            with tarfile.open(first) as tf:
                self.assertIn('ost.yaml',tf.getnames());self.assertIn('left-0000.png',tf.getnames())
            self.assertEqual((d/'latest_saved.txt').read_text().strip(),str(second))

    def test_save_and_commit_redirect_are_local_and_restored(self):
        old_save,old_upload=BaseCalibration.do_save,GUI.do_upload
        with tempfile.TemporaryDirectory() as d:
            with entry.local_gui_actions(NS(Calibrator=BaseCalibration),NS(CalibrationNode=GUI),Path(d)):
                Calibration().do_save()
                self.assertFalse(GUI().do_upload())
            self.assertIs(BaseCalibration.do_save,old_save);self.assertIs(GUI.do_upload,old_upload)

    def test_real_cli_receives_measured_board_and_ros_remaps(self):
        def run():
            self.assertIn('8x6',sys.argv);self.assertIn('0.025',sys.argv)
            self.assertIn('image:=/image_raw',sys.argv)
            self.assertIn('--no-service-check',sys.argv)
            Calibration().do_save();self.assertFalse(GUI().do_upload())
        modules={
            'camera_calibration.calibrator':NS(Calibrator=BaseCalibration,MonoCalibrator=Calibration,__file__='/example/calibrator.py'),
            'camera_calibration.camera_calibrator':NS(CalibrationNode=GUI),
            'camera_calibration.nodes.cameracalibrator':NS(main=run)}
        with tempfile.TemporaryDirectory() as d,patch.object(entry.importlib,'import_module',side_effect=modules.__getitem__):
            code=entry.main(['--output-dir',d,'--cols','8','--rows','6','--square-mm','25'])
            self.assertEqual(code,0)
            self.assertEqual(json.loads((Path(d)/'session.json').read_text())['square_m'],.025)

    def test_bad_board_rejected_before_loading_ros(self):
        for square in ('0','nan','25000'):
            with self.subTest(square=square),patch.object(entry.importlib,'import_module') as importer:
                with self.assertRaises(SystemExit):
                    entry.main(['--output-dir','/unused','--cols','8','--rows','6','--square-mm',square])
                importer.assert_not_called()

    def test_missing_installed_tool_is_failure_not_install(self):
        with patch.object(entry.importlib,'import_module',side_effect=ImportError('no camera_calibration')):
            self.assertEqual(entry.main(['--output-dir','/unused','--cols','8','--rows','6','--square-mm','25']),1)

if __name__=='__main__': unittest.main(verbosity=2)
