import ast
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

from ros_shim import Message, Node
from go2_vision_test.video import VideoReader
from go2_vision_test.video_node import VideoCamera
from go2_vision_test.inputs_node import VideoTestInputs
from go2_vision_test.topics import MANAGER_REMAPS, DETECTOR_REMAPS
from go2_mission_manager.node import MissionManager


class Capture:
    def __init__(self, fps=25, count=25, actual_count=None):
        self.fps, self.count = fps, count
        self.actual_count = count if actual_count is None else actual_count
        self.index = -1
        self.released = False
        self.rewinds = 0

    def isOpened(self):
        return True

    def get(self, key):
        return self.fps if key == 'fps' else self.count

    def grab(self):
        self.index += 1
        return self.index < self.actual_count

    def retrieve(self):
        return True, np.full((2, 3, 3), self.index % 255, dtype=np.uint8)

    def set(self, key, value):
        self.index = int(value) - 1
        self.rewinds += 1
        return True

    def release(self):
        self.released = True


class VideoReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'video with spaces.mp4'
        self.path.touch()
        self.addCleanup(setattr, Node, 'overrides', {})

    def reader(self, capture=None, **kwargs):
        capture = capture or Capture()
        cv = NS(VideoCapture=lambda path: capture, CAP_PROP_FPS='fps',
                CAP_PROP_FRAME_COUNT='count', CAP_PROP_POS_FRAMES='pos')
        return VideoReader(self.path, cv_module=cv, **kwargs)

    def test_five_fps_samples_one_second_of_25fps_video_at_original_speed(self):
        reader = self.reader()
        indices = [int(reader.read_at(t)[0, 0, 0]) for t in (0, .2, .4, .6, .8)]
        self.assertEqual(indices, [0, 5, 10, 15, 20])
        self.assertIsNone(reader.read_at(1.0))
        self.assertTrue(reader.finished)
        self.assertIsNone(reader.read_at(1.2))

    def test_default_one_point_five_fps_camera_preserves_original_video_speed(self):
        self.now, self.wall = 0.0, 1000.0
        clock = NS(monotonic=lambda: self.now, time=lambda: self.wall,
                   time_ns=lambda: int(self.wall * 1_000_000_000))
        Node.overrides = {'video_path': str(self.path), 'wait_for_detector': False}
        with patch('go2_vision_test.video_node.time', clock):
            camera = VideoCamera(lambda *a, **kw: self.reader(Capture(count=100), **kw))
            self.assertEqual(camera.config['publish_rate_hz'], 1.5)
            for elapsed in (0.0, .667, 1.334):
                self.now, self.wall = elapsed, 1000.0 + elapsed
                camera.tick()
            self.assertEqual([m.data[0] for m in camera.image_pub.messages], [0, 16, 33])
            camera.close()

    def test_slow_source_does_not_repeat_images_and_loop_rewinds(self):
        reader = self.reader(Capture(fps=2, count=2))
        self.assertIsNotNone(reader.read_at(0))
        self.assertIsNone(reader.read_at(.2))
        self.assertEqual(int(reader.read_at(.6)[0, 0, 0]), 1)
        capture = Capture()
        reader = self.reader(capture, loop=True)
        reader.read_at(.8)
        self.assertEqual(int(reader.read_at(1.2)[0, 0, 0]), 5)
        self.assertEqual(reader.cycle, 1)
        self.assertEqual(capture.rewinds, 1)

    def test_bad_metadata_and_truncated_video_fail_explicitly(self):
        capture = Capture(fps=0)
        with self.assertRaises(ValueError):
            self.reader(capture)
        self.assertTrue(capture.released)
        self.assertEqual(self.reader(Capture(fps=0), source_fps=25).fps, 25)
        reader = self.reader(Capture(actual_count=2))
        reader.read_at(0)
        with self.assertRaises(ValueError):
            reader.read_at(.2)

    def test_camera_waits_then_publishes_bgr_with_fresh_wall_timestamps(self):
        clock = NS(monotonic=lambda: self.now, time=lambda: self.wall,
                   time_ns=lambda: int(self.wall * 1_000_000_000))
        self.now, self.wall = 10.0, 1000.0
        Node.overrides = {'video_path': str(self.path), 'publish_rate_hz': 5.0}
        with patch('go2_vision_test.video_node.time', clock):
            camera = VideoCamera(lambda *a, **kw: self.reader(**kw))
            camera.tick()
            self.assertEqual(camera.sent, 0)
            self.now, self.wall = 20.0, 1010.0
            camera.on_detector(Message(data=json.dumps(dict(
                model_ready=True, enabled=True, stamp_unix=self.wall))))
            camera.tick()
            msg = camera.image_pub.messages[-1]
            self.assertEqual((msg.width, msg.height, msg.step, msg.encoding), (3, 2, 9, 'bgr8'))
            self.assertEqual(msg.header.stamp.sec, 1010)
            self.assertEqual(msg.data[0], 0)
            self.now, self.wall = 20.21, 1010.21
            camera.tick()
            self.assertEqual(camera.image_pub.messages[-1].data[0], 5)
            position = camera.position
            camera.on_detector(Message(data=json.dumps(dict(
                model_ready=True, enabled=False, stamp_unix=self.wall))))
            self.now, self.wall = 30.0, 1020.0
            camera.tick()
            self.assertEqual(camera.position, position)
            self.assertEqual(camera.state, 'PAUSED')
            camera.on_detector(Message(data=json.dumps(dict(
                model_ready=True, enabled=True, stamp_unix=self.wall))))
            camera.tick()
            self.assertEqual(camera.position, position)
            self.assertEqual(camera.sent, 2)  # same source frame is not republished on resume
            camera.close()

    def test_camera_eof_does_not_republish_last_frame(self):
        self.now, self.wall = 0.0, 1000.0
        clock = NS(monotonic=lambda: self.now, time=lambda: self.wall,
                   time_ns=lambda: int(self.wall * 1_000_000_000))
        Node.overrides = {'video_path': str(self.path), 'wait_for_detector': False}
        with patch('go2_vision_test.video_node.time', clock):
            camera = VideoCamera(lambda *a, **kw: self.reader(**kw))
            camera.tick()
            self.now, self.wall = 1.1, 1001.1
            camera.tick()
            camera.tick()
            self.assertEqual(camera.state, 'EOF')
            self.assertEqual(camera.sent, 1)
            camera.close()

    def test_synthetic_inputs_enable_actual_manager_without_motion(self):
        Node.overrides = {'enable_motion': False}
        manager = MissionManager()
        Node.overrides = {}
        inputs = VideoTestInputs()
        inputs.tick()
        manager.on_goal(inputs.goal_pub.messages[-1])
        manager.on_nav(inputs.nav_pub.messages[-1])
        manager.on_bridge(inputs.bridge_pub.messages[-1])
        manager.tick()
        self.assertTrue(manager.identification.enabled)
        self.assertEqual(manager.cmd_pub.messages, [])
        self.assertEqual(manager.permit_pub.messages, [])
        inputs.on_mission(manager.status_pub.messages[-1])
        self.assertTrue(inputs.goal_accepted)
        goal_count = len(inputs.goal_pub.messages)
        inputs.tick()
        self.assertEqual(len(inputs.goal_pub.messages), goal_count)

    def test_every_production_endpoint_is_remapped_in_test_launch(self):
        root = Path(__file__).resolve().parents[1]
        for path, mappings in (
                ('src/go2_mission_manager/go2_mission_manager/node.py', MANAGER_REMAPS),
                ('src/go2_animal_detector/go2_animal_detector/node.py', DETECTOR_REMAPS)):
            endpoints = set()
            for call in ast.walk(ast.parse((root / path).read_text())):
                if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                        and call.func.attr in ('create_publisher', 'create_subscription', 'create_service', 'create_client')
                        and len(call.args) >= 2 and isinstance(call.args[1], ast.Constant)
                        and isinstance(call.args[1].value, str) and call.args[1].value.startswith('/')):
                    endpoints.add(call.args[1].value)
            remaps = dict(mappings)
            self.assertTrue(endpoints <= remaps.keys(), endpoints - remaps.keys())
            self.assertTrue(all(target.startswith('/vision_test/') for target in remaps.values()))

        path = root / 'src/go2_vision_test/launch/video_test.launch.py'
        nodes = [n for n in ast.walk(ast.parse(path.read_text()))
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'Node']
        packages = {ast.literal_eval(next(k.value for k in n.keywords if k.arg == 'package')) for n in nodes}
        self.assertEqual(packages, {'go2_vision_test', 'go2_animal_detector', 'go2_mission_manager'})
        manager = next(n for n in nodes if any(k.arg == 'package' and ast.literal_eval(k.value) == 'go2_mission_manager' for k in n.keywords))
        params = ast.literal_eval(next(k.value for k in manager.keywords if k.arg == 'parameters'))
        self.assertIs(params[0]['enable_motion'], False)
