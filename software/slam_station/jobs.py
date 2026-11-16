"""UI-independent job API shared by CLI and Windows/Linux desktop."""
import hashlib
import json
import shutil
import threading
import time
import uuid
from pathlib import Path
from devices import load_device, pipeline_profile, check_dataset
from runtime import command, diagnose, execute
from trajectory import export_target


def save_json(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False),encoding='utf-8')
    tmp.replace(path)


class Job:
    def __init__(self, source, parent, settings, device, stride=5, max_frames=0,
                 timeout=0, prepare_only=False, image='umi-slam:0.1'):
        source,parent,settings,device = (Path(p).expanduser().resolve() for p in (source,parent,settings,device))
        if source == parent or parent.is_relative_to(source) or source.is_relative_to(parent):
            raise ValueError('采集目录和结果保存目录不能互相嵌套')
        config = load_device(device); check_dataset(source, config)
        if not settings.is_file(): raise ValueError('请选择与当前设备对应的标定 YAML')
        # Validate every parameter before allocating a job directory.
        command(source, parent/('_validate_'+uuid.uuid4().hex), settings, device,
                stride,max_frames,timeout,prepare_only,image)
        self.folder = parent/('run_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        self.folder.mkdir(parents=True,exist_ok=False)
        self.result = self.folder/'result'; self.cancel = threading.Event(); self.thread = None
        self.state = {'schema_version':1,'status':'queued','source':str(source),'device':config,
                      'settings_source':str(settings),'image':image,'stride':stride,'max_frames':max_frames,
                      'timeout':timeout,'prepare_only':prepare_only,'created_unix_s':time.time()}
        cfg = self.folder/'config'; cfg.mkdir()
        shutil.copyfile(settings,cfg/'settings.yaml')
        self.state['calibration_sha256'] = hashlib.sha256((cfg/'settings.yaml').read_bytes()).hexdigest()
        save_json(cfg/'device.json', config); save_json(cfg/'profile.json', pipeline_profile(config))
        self.cmd = command(source,self.result,cfg/'settings.yaml',cfg/'profile.json',stride,max_frames,
                           timeout,prepare_only,image,'slam-station-'+uuid.uuid4().hex[:16])
        self.state['command'] = self.cmd
        self._save()

    def _save(self): save_json(self.folder/'station_job.json',self.state)

    def start(self):
        if self.thread is not None: raise RuntimeError('任务不能重复启动')
        self.thread = threading.Thread(target=self.run,daemon=False); self.thread.start()

    def run(self):
        with (self.folder/'host.log').open('w',encoding='utf-8') as log:
            try:
                self.state['status']='checking'; self._save()
                self.state['backend']=diagnose(self.state['image'])
                if self.cancel.is_set():
                    self.state['status']='cancelled'; return
                self.state['status']='running'; self._save()
                code=execute(self.cmd,self.cancel,log)
                self.state['exit_code']=code
                if code == 130 or self.cancel.is_set(): self.state['status']='cancelled'; return
                state_path=self.result/'job.json'
                backend_state=json.loads(state_path.read_text(encoding='utf-8')) if state_path.exists() else {}
                expected='prepared' if self.state['prepare_only'] else 'completed'
                if code != 0 or backend_state.get('status') != expected:
                    raise RuntimeError(backend_state.get('error') or f'后端未完成，退出码 {code}，请查看日志')
                if expected == 'completed':
                    self.state['coverage_fraction']=backend_state.get('coverage_fraction')
                    exported=export_target(self.result,self.state['device'])
                    if exported: self.state['target_trajectory']=str(exported)
                self.state['status']=expected
            except Exception as exc:
                self.state['status']='failed'
                self.state['error']=str(exc); log.write('\nERROR: '+str(exc)+'\n')
            finally:
                self.state['finished_unix_s']=time.time(); self._save()

    def stop(self): self.cancel.set()

    @property
    def active(self): return self.thread is not None and self.thread.is_alive()
