import json
import threading
import time
import unittest
from types import SimpleNamespace as NS

from ros_shim import Message, Node
from go2_animal_detector.node import AnimalDetector
from go2_mission_manager.node import MissionManager


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('Timed out waiting for worker')


def frame(value=1):
    msg = Message(encoding='bgr8', height=1, width=1, step=3, data=bytes([value] * 3))
    wall = time.time_ns()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(wall, 1_000_000_000)
    return msg


def enable(node, session='s', enabled=True):
    node.on_control(Message(data=json.dumps(dict(
        mission_id='m', session_id=session, enabled=enabled, stamp_unix=time.time()))))


class DetectorPipelineTests(unittest.TestCase):
    def setUp(self):
        # Explicit fast profile for timing tests; production default is 1 Hz.
        Node.overrides = {'inference_rate_hz': 5.0}
        self.addCleanup(setattr, Node, 'overrides', {})

    def detector(self, factory):
        node = AnimalDetector(factory)
        self.addCleanup(node.close)
        wait_until(lambda: node.model_ready or node.model_error)
        return node

    def test_two_real_callbacks_confirm_and_stop_manager(self):
        detector = self.detector(lambda cfg: NS(predict=lambda image: .9))
        Node.overrides = dict(enable_motion=True)
        manager = MissionManager()
        Node.overrides = {}
        manager.on_goal(Message(latitude=45.0, longitude=11.0))
        manager.logic.state = 'NAVIGATING_ACTIVE'
        manager.run_id, manager.epoch = 'run', 'bridge'
        manager.identification.update('NAVIGATING_ACTIVE', 10, True, time.monotonic(), time.time())
        manager._publish_identification_control()
        detector.on_control(manager.ident_control_pub.messages[-1])
        for index in range(2):
            detector.on_image(frame())
            wait_until(lambda: detector.pending_result is not None)
            detector.publish_result()
            message = detector.result_pub.messages[-1]
            self.assertEqual(json.loads(message.data)['detected'], index == 1)
            manager.on_identification(message)
        self.assertTrue(manager.wolf_flag_pub.messages[-1].data)
        self.assertEqual(manager.logic.state, 'WOLF_DETECTED')
        self.assertEqual(manager.cmd_pub.messages[-1].twist.linear.x, 0)
        self.assertFalse(manager.permit_pub.messages[-1].enabled)
        self.assertEqual(len(manager.stop_client.calls), 1)
        detector.on_control(manager.ident_control_pub.messages[-1])
        detector.publish_status()
        self.assertEqual(json.loads(detector.status_pub.messages[-1].data)['state'], 'DISABLED')

    def test_inflight_previous_session_is_discarded(self):
        started, release, done = threading.Event(), threading.Event(), threading.Event()

        def predict(image):
            started.set()
            release.wait(2)
            done.set()
            return .99

        detector = self.detector(lambda cfg: NS(predict=predict))
        self.addCleanup(release.set)
        enable(detector)
        detector.on_image(frame())
        self.assertTrue(started.wait(1))
        enable(detector, session='next')
        release.set()
        self.assertTrue(done.wait(1))
        time.sleep(.03)
        detector.publish_result()
        self.assertEqual(detector.result_pub.messages, [])
        self.assertIsNone(detector.last_result)

    def test_latest_frame_replaces_queue_and_rate_is_bounded(self):
        started, release = threading.Event(), threading.Event()
        calls = []

        def predict(image):
            calls.append((int(image[0, 0, 0]), time.monotonic()))
            if len(calls) == 1:
                started.set()
                release.wait(2)
            return .1

        detector = self.detector(lambda cfg: NS(predict=predict))
        self.addCleanup(release.set)
        enable(detector)
        detector.on_image(frame(1))
        self.assertTrue(started.wait(1))
        for value in range(2, 21):
            detector.on_image(frame(value))
        release.set()
        wait_until(lambda: len(calls) == 2)
        self.assertEqual([c[0] for c in calls], [1, 20])
        self.assertGreaterEqual(calls[1][1] - calls[0][1], .19)

    def test_model_error_is_unavailable_not_a_false_result(self):
        def missing(cfg):
            raise FileNotFoundError('missing wolf.pt')

        detector = self.detector(missing)
        enable(detector)
        detector.publish_status()
        status = json.loads(detector.status_pub.messages[-1].data)
        self.assertEqual(status['state'], 'MODEL_ERROR')
        self.assertFalse(status['valid'])
        self.assertIsNone(status['detected'])
        self.assertEqual(detector.detected_pub.messages, [])

    def test_stale_or_duplicate_images_do_not_count(self):
        calls = []
        detector = self.detector(lambda cfg: NS(predict=lambda image: calls.append(1) or .9))
        enable(detector)
        old = frame()
        old.header.stamp.sec -= 5
        detector.on_image(old)
        self.assertIsNone(detector.frame)
        current = frame()
        detector.on_image(current)
        wait_until(lambda: detector.last_result is not None)
        for _ in range(10):
            detector.on_image(current)
        time.sleep(.25)
        self.assertEqual(len(calls), 1)
        self.assertFalse(detector.last_result['detected'])

    def test_cold_warmup_finishes_before_ready_and_never_counts_as_detection(self):
        started, release = threading.Event(), threading.Event()

        def warmup():
            started.set()
            release.wait(2)
            return .99  # synthetic warmup output must not vote or publish

        detector = AnimalDetector(lambda cfg: NS(warmup=warmup, predict=lambda image: .9))
        self.addCleanup(detector.close)
        self.addCleanup(release.set)
        self.assertTrue(started.wait(1))
        enable(detector)
        detector.on_image(frame())
        detector.publish_status()
        status = json.loads(detector.status_pub.messages[-1].data)
        self.assertEqual(status['state'], 'WARMING_MODEL')
        self.assertFalse(status['model_ready'])
        self.assertFalse(status['valid'])
        self.assertIsNone(detector.frame)
        self.assertEqual(detector.detected_pub.messages, [])
        release.set()
        wait_until(lambda: detector.model_ready)
        self.assertGreaterEqual(detector.warmup_ms, 0)
        for index in range(2):
            detector.on_image(frame())
            wait_until(lambda: detector.pending_result is not None)
            detector.publish_result()
            self.assertEqual(json.loads(detector.result_pub.messages[-1].data)['detected'], index == 1)
        self.assertEqual(len(detector.window.samples), 2)

    def test_failed_warmup_remains_model_error_without_a_boolean(self):
        def warmup():
            raise RuntimeError('warmup failed')

        detector = self.detector(lambda cfg: NS(warmup=warmup, predict=lambda image: .9))
        detector.publish_status()
        self.assertEqual(json.loads(detector.status_pub.messages[-1].data)['state'], 'MODEL_ERROR')
        self.assertFalse(detector.model_ready)
        self.assertEqual(detector.detected_pub.messages, [])

    def test_slow_real_frame_after_warmup_is_discarded_then_recovers(self):
        calls = []

        def predict(image):
            calls.append(1)
            if len(calls) == 1:
                time.sleep(.12)
            return .99

        Node.overrides = {'inference_rate_hz': 50.0, 'max_frame_age_sec': .05}
        detector = self.detector(lambda cfg: NS(warmup=lambda: None, predict=predict))
        enable(detector)
        detector.on_image(frame())
        wait_until(lambda: bool(detector.inference_error))
        detector.publish_result()
        self.assertEqual(detector.result_pub.messages, [])
        self.assertIsNone(detector.last_result)
        detector.on_image(frame())
        wait_until(lambda: detector.last_result is not None)
        self.assertFalse(detector.last_result['detected'])
        self.assertEqual(detector.last_result['positive_frames'], 1)
        self.assertEqual(detector.inference_error, '')
