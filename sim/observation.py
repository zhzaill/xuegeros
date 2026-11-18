"""Bounded workbench image receipts, distinct from camera frames."""
import base64
import math
import time
from collections import OrderedDict
from threading import Lock


class WorkbenchFrames:
    def __init__(self):
        self.frames = OrderedDict()
        self.receipts = OrderedDict()
        self.lock = Lock()
        self.serial = 0

    def publish(self, payload):
        view = payload.get('view') or {}
        stamp = float(view.get('state_ts') or 0)
        if not math.isfinite(stamp) or not 0 <= time.time() - stamp < 3:
            raise ValueError('stale_render')
        data = base64.b64decode(payload.get('png', ''), validate=True)
        if not data.startswith(b'\x89PNG\r\n\x1a\n') or len(data) > 2_000_000:
            raise ValueError('invalid_png')
        with self.lock:
            self.serial += 1
            meta = dict(kind='workbench', availability='ok', seq=self.serial,
                        captured_at=time.time(), state_ts=stamp, view=view,
                        pose=view.get('pose_m_xy_yaw'), physical=False,
                        provenance='glimpsed', camera_evidence=False,
                        workbench_not_camera_evidence=True)
            self.frames[self.serial] = (data, meta)
            while len(self.frames) > 8:
                self.frames.popitem(last=False)
            return meta

    def latest(self):
        with self.lock:
            if not self.frames:
                return dict(ok=False, availability='unavailable', error='workbench_not_open',
                            reason='打开仿真工作台后才能取得实际渲染画面', physical=False)
            _, meta = next(reversed(self.frames.values()))
            age = time.time() - meta['state_ts']
            if age > 3:
                return dict(ok=False, availability='unavailable', error='workbench_stale', age_s=age, physical=False)
            self.receipts[meta['seq']] = self.frames[meta['seq']]
            while len(self.receipts) > 32:
                self.receipts.popitem(last=False)
            return dict(meta, ok=True, age_s=round(age, 3), url=f"/api/workbench/frames/{meta['seq']}.png")

    def image(self, seq):
        with self.lock:
            return self.receipts.get(seq) or self.frames.get(seq)
