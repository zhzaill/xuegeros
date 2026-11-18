"""Exercise the actual image HTTP routes; a missing renderer stays unavailable."""
import base64
import json
import time
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from sim.local_sim import start_http, model_metadata
from sim.probe.tour_regression import make_sim


class ObservationApiTests(unittest.TestCase):
    def setUp(self):
        self.sim, _ = make_sim()
        self.server = start_http(self.sim, 0, model_metadata())
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def test_render_roundtrip_and_stale_render_rejection(self):
        with urlopen(self.base + '/api/workbench/latest') as r:
            self.assertEqual(json.load(r)['error'], 'workbench_not_open')
        png = base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aLu0AAAAASUVORK5CYII=')
        payload = dict(png=base64.b64encode(png).decode(), view=dict(state_ts=time.time(), state_seq=12, pose_m_xy_yaw=[.1, .2, 0]))
        req = Request(self.base + '/api/workbench/frame', data=json.dumps(payload).encode(), headers={'content-type':'application/json'})
        with urlopen(req) as r:
            self.assertTrue(json.load(r)['ok'])
        with urlopen(self.base + '/api/workbench/latest') as r:
            meta = json.load(r)
        self.assertEqual(meta['view']['state_seq'], 12)
        self.assertFalse(meta['camera_evidence'])
        with urlopen(self.base + meta['url']) as r:
            self.assertEqual(r.read(), png)
        payload['view']['state_ts'] -= 10
        req.data = json.dumps(payload).encode()
        with self.assertRaises(HTTPError) as caught:
            urlopen(req)
        self.assertEqual(caught.exception.code, 400)
        caught.exception.close()

    def test_map_url_returns_an_actual_png(self):
        with urlopen(self.base + '/api/map.png') as r:
            self.assertEqual(r.headers.get_content_type(), 'image/png')
            self.assertTrue(r.read().startswith(b'\x89PNG\r\n\x1a\n'))
