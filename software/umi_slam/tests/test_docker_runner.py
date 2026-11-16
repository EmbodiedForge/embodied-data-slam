import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('docker_runner',Path(__file__).resolve().parents[1]/'docker_runner.py')
runner=importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)


class DockerLauncherTests(unittest.TestCase):
    def test_paths_with_spaces_chinese_and_ampersand(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); data=root/'双目 数据&IMU'; data.mkdir()
            settings=root/'标定 文件.yaml'; settings.write_text('settings')
            a=SimpleNamespace(input=data,output=root/'results'/'一次测试',settings=settings,
                              profile=None,stride=5,max_frames=0,timeout=0,image='umi-slam:0.1',
                              prepare_only=True,prepared_input=False)
            cmd=runner.command(a,'test-name')
            self.assertIn(str(data)+':/input:ro',cmd)
            self.assertIn(str(settings)+':/config/settings.yaml:ro',cmd)
            self.assertIn('/output/一次测试',cmd)
            self.assertIn('--prepare-only',cmd)
            self.assertEqual(cmd[cmd.index('--network')+1],'none')

    def test_refuses_nested_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); settings=root/'settings.yaml'; settings.write_text('settings')
            a=SimpleNamespace(input=root,output=root/'output',settings=settings)
            with self.assertRaisesRegex(ValueError,'nested'): runner.command(a,'test')


if __name__=='__main__': unittest.main()
