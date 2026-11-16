import csv
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch, Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from devices import load_device, validate_transform, check_dataset
from runtime import bind_mount, command, execute
from trajectory import export_target, matrix_quaternion, quaternion_matrix
from jobs import Job

HERE=Path(__file__).resolve().parents[1]


class StationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        self.source=self.root/'中文 输入 &'; self.source.mkdir()
        self.settings=self.root/'标定.yaml'; self.settings.write_text('settings')
        self.device=HERE/'devices/generic_euroc.json'
    def tearDown(self): self.tmp.cleanup()

    def test_windows_drive_mount_and_csv_escape(self):
        path='C:\\采集 数据,&实验\\run'
        fields=next(csv.reader(io.StringIO(bind_mount(path,'/input',True))))
        self.assertEqual(fields,['type=bind','source='+path,'target=/input','readonly'])
        with patch('runtime.os.getuid',side_effect=AssertionError('Windows must not call getuid')):
            cmd=command(self.source,self.root/'out/result',self.settings,self.device,windows=True)
        self.assertNotIn('--user',cmd); self.assertIn('never',cmd)
        self.assertIn('/output/result',cmd); self.assertEqual(cmd[cmd.index('--network')+1],'none')

    def test_linux_user_and_readonly_source(self):
        cmd=command(self.source,self.root/'out/result',self.settings,self.device)
        self.assertIn(f'{os.getuid()}:{os.getgid()}',cmd)
        mounts=[cmd[i+1] for i,x in enumerate(cmd) if x=='--mount']
        self.assertTrue(mounts[0].endswith('readonly'))
        self.assertFalse(mounts[1].endswith('readonly'))

    def test_refuse_nested_existing_and_invalid_params(self):
        for out in (self.source/'out',self.root):
            with self.assertRaises(ValueError): command(self.source,out,self.settings,self.device)
        for kwargs in ({'stride':0},{'timeout':float('nan')},{'max_frames':-1}):
            with self.assertRaises(ValueError): command(self.source,self.root/'out',self.settings,self.device,**kwargs)

    def test_unknown_protocol_units_and_bad_transform(self):
        d=load_device(self.device)
        for key,value in [('adapter','ego_video'),('gyro_unit','deg/s'),('camera_to_imu_offset_ns',.1),('T_imu_target',[[0]*4]*4)]:
            p=self.root/'device.json'; p.write_text(json.dumps({**d,key:value}))
            with self.assertRaises(ValueError): load_device(p)
        for p in (HERE/'devices').glob('*.json'): load_device(p)
        with self.assertRaises(ValueError): validate_transform([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])

    def test_euroc_does_not_accept_legacy_raw(self):
        (self.source/'stereo.mkv').touch()
        with self.assertRaisesRegex(ValueError,'EuRoC'): check_dataset(self.source,load_device(self.device))

    def test_target_transform_rotates_translation_and_preserves_gaps(self):
        source=self.root/'imu_trajectory.csv'
        source.write_text('timestamp_ns,time_from_input_start_s,x_m,y_m,z_m,qx,qy,qz,qw,segment_id\n1789459434308037880,0,1,2,3,0,0,0.7071067811865476,0.7071067811865476,3\n')
        original=source.read_bytes()
        d={'target_frame':'gripper_tcp','timestamp_clock':'hardware','T_imu_target':[[1,0,0,1],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}
        target=export_target(self.root,d)
        with target.open() as f: row=next(csv.DictReader(f))
        self.assertAlmostEqual(float(row['x_m']),1); self.assertAlmostEqual(float(row['y_m']),3)
        self.assertEqual(row['timestamp_ns'],'1789459434308037880'); self.assertEqual(row['segment_id'],'3')
        self.assertEqual(source.read_bytes(),original)
        with self.assertRaises(FileExistsError): export_target(self.root,d)
        self.assertIsNone(export_target(self.root,{'T_imu_target':None}))

    def test_rotation_roundtrip_at_180_degrees(self):
        for q in ([1,0,0,0],[0,1,0,0],[0,0,1,0],[.5,.5,.5,.5],[0,0,0,1]):
            r=quaternion_matrix(q); actual=quaternion_matrix(matrix_quaternion(r))
            for a,b in zip(sum(r,[]),sum(actual,[])): self.assertAlmostEqual(a,b)

    def test_cancel_before_start_never_creates_container(self):
        event=threading.Event(); event.set()
        with patch('runtime.subprocess.Popen') as popen:
            self.assertEqual(execute(['docker','--name','test'],event,io.StringIO()),130); popen.assert_not_called()

    def test_cancel_retries_container_creation_race(self):
        event=Mock(); event.is_set.return_value=False; event.wait.return_value=True
        process=Mock(); process.poll.side_effect=[None,None]; process.wait.return_value=0
        with patch('runtime.subprocess.Popen',return_value=process), patch('runtime.time.sleep'), patch('runtime.docker_call',side_effect=[Mock(returncode=1),Mock(returncode=0)]) as stop:
            self.assertEqual(execute(['docker','--name','specific-job'],event,io.StringIO()),130)
        self.assertEqual(stop.call_count,2)
        for call in stop.call_args_list: self.assertEqual(call.args[0][-1],'specific-job')

    def test_job_no_false_success_and_config_snapshot(self):
        for cam in ('cam0','cam1'): (self.source/f'mav0/{cam}/data').mkdir(parents=True)
        (self.source/'mav0/imu0').mkdir(); (self.source/'mav0/imu0/data.csv').touch()
        (self.source/'timestamps.txt').touch()
        job=Job(self.source,self.root/'results',self.settings,self.device)
        self.settings.write_text('changed')
        self.assertEqual((job.folder/'config/settings.yaml').read_text(),'settings')
        with patch('jobs.diagnose',return_value={'id':'test'}),patch('jobs.execute',return_value=0): job.run()
        self.assertEqual(job.state['status'],'failed')
        self.assertIn('后端未完成',job.state['error'])


if __name__=='__main__': unittest.main()
