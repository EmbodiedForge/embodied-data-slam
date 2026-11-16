import csv
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest

import cv2
import numpy as np

SPEC=importlib.util.spec_from_file_location('pipeline',Path(__file__).resolve().parents[1]/'pipeline.py')
pipeline=importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(pipeline)


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.path=Path(self.tmp.name)

    def tearDown(self): self.tmp.cleanup()

    def test_uart_fragments_and_units(self):
        data=struct.pack('<6h',0,0,8192,0,16384,0)
        packet=b'\x55\x55\x03\x0c'+data
        packet+=bytes([sum(packet)&255])
        parts=[packet[:3],packet[3:]+packet]
        file=self.path/'imu.bin'
        file.write_bytes(b''.join(struct.pack('<QI',t,len(v))+v for t,v in zip([90000000,100000000],parts)))
        ts,vals=pipeline.decode_legacy_imu(file,250,4,2000)
        self.assertEqual(ts,[96000000,100000000])
        self.assertAlmostEqual(vals[0,5],9.80665)
        self.assertAlmostEqual(vals[0,1],1000*np.pi/180)

    def test_truncated_uart_rejected(self):
        f=self.path/'imu.bin'; f.write_bytes(struct.pack('<QI',1,17)+b'123')
        with self.assertRaisesRegex(ValueError,'Truncated'): pipeline.decode_legacy_imu(f,250,4,2000)

    def test_timestamp_precision_and_order(self):
        self.assertEqual(pipeline.integer_ns('1789459434308037880.000000'),1789459434308037880)
        for bad in ('nan','inf','1.1','-1'):
            with self.assertRaises(ValueError): pipeline.integer_ns(bad)
        with self.assertRaises(ValueError): pipeline.strictly_increasing([1,1],'IMU')

    def make_export(self,initialized=True):
        (self.path/'imu_euroc.txt').write_text('1000000000.000000 0 0 0 0 0 0 1\n1040000000.000000 1 0 0 0 0 0 1\n1200000000.000000 2 0 0 0 0 0 1\n')
        (self.path/'tracking.csv').write_text('timestamp_ns,state,imu_initialized,map_id\n'+''.join(f'{t},2,{int(initialized)},0\n' for t in (1000000000,1040000000,1200000000)))
        return {'first_timestamp_ns':1000000000,'selected_frames':6,'median_frame_period_s':.04,'warnings':[]}

    def test_export_preserves_imu_pose_and_gaps(self):
        quality=pipeline.export_result(self.path,self.make_export())
        self.assertEqual(quality['segments'],2); self.assertEqual(quality['coverage_fraction'],.5)
        self.assertEqual(quality['pose'],'T_world_imu')
        with (self.path/'imu_trajectory.csv').open() as f: rows=list(csv.DictReader(f))
        self.assertEqual(float(rows[-1]['x_m']),2); self.assertEqual(rows[-1]['segment_id'],'1')
        self.assertTrue((self.path/'imu_trajectory.tum').read_text().startswith('1.000000000 '))

    def test_uninitialized_not_success(self):
        with self.assertRaisesRegex(ValueError,'initialized'): pipeline.export_result(self.path,self.make_export(False))

    def test_export_float_ns_roundoff_matches_original_stamp(self):
        report=self.make_export()
        file=self.path/'imu_euroc.txt'
        file.write_text(file.read_text().replace('1000000000.000000','1000000000.015625'))
        pipeline.export_result(self.path,report)
        with (self.path/'imu_trajectory.csv').open() as f: rows=list(csv.DictReader(f))
        self.assertEqual(rows[0]['timestamp_ns'],'1000000000')

    def test_prepare_camera_imu_shift_and_size(self):
        src=self.path/'source'; dst=self.path/'output'; dst.mkdir()
        for camera in ('cam0','cam1'):
            d=src/f'mav0/{camera}/data'; d.mkdir(parents=True)
            for t in (1000000000,1040000000,1080000000): cv2.imwrite(str(d/f'{t}.png'),np.zeros((720,1280),np.uint8))
        (src/'timestamps.txt').write_text('1000000000\n1040000000\n1080000000\n')
        (src/'mav0/imu0').mkdir()
        (src/'mav0/imu0/data.csv').write_text(''.join(f'{t},0,0,0,0,0,9.80665\n' for t in range(996000000,1090000000,4000000)))
        settings=Path(__file__).resolve().parents[1]/'profiles/umi_20260908.yaml'
        report=pipeline.prepare(src,dst,{'camera_to_imu_offset_ns':1000000},settings,1,0)
        self.assertEqual(report['selected_frames'],3)
        self.assertTrue((dst/'imu.csv').read_text().startswith('995000000,'))
        self.assertIn('Camera.fps: 25',(dst/'settings.yaml').read_text())


if __name__=='__main__': unittest.main()
