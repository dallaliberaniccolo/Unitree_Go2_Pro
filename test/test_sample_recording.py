import csv
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

from ros_shim import Node
from go2_animal_detector.node import AnimalDetector
from go2_animal_detector.recording import SampleRecorder
from go2_vision_test.analyze_video import analyze
from go2_vision_test.review_ros import ReviewState, DiagnosticControl, stop_launch, add_source_positions
from test_detector_pipeline import enable, frame, wait_until


class FakeCV:
    FONT_HERSHEY_SIMPLEX = 0

    def imwrite(self, path, pixels):
        Path(path).write_bytes(pixels.tobytes())
        return True

    def rectangle(self, *args):
        pass

    def putText(self, *args):
        pass


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.addCleanup(setattr, Node, 'overrides', {})

    def recorder(self):
        return SampleRecorder(self.root, {'mode': 'test'}, cv=FakeCV())

    def test_positive_negative_invalid_frames_and_boxes_are_all_saved(self):
        recorder = self.recorder()
        image = np.zeros((4, 5, 3), dtype=np.uint8)
        box = dict(label='wolf', class_id=0, confidence=.8, xyxy=[0, 0, 3, 3])
        for score, valid, error in ((.8, True, ''), (0, True, ''),
                                     (.9, False, '<expired>')):
            recorder.save(image, dict(confidence=score, frame_positive=score > .2,
                                      detected=False if valid else None, valid=valid, error=error),
                          [box] if score else [])
        rows = [json.loads(line) for line in (recorder.path / 'results.jsonl').read_text().splitlines()]
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]['boxes'], [box])
        self.assertEqual(rows[1]['boxes'], [])
        self.assertIsNone(rows[2]['detected'])
        for row in rows:
            self.assertTrue((recorder.path / row['image']).is_file())
            self.assertTrue((recorder.path / row['annotated']).is_file())
        with (recorder.path / 'results.csv').open() as stream:
            self.assertEqual(len(list(csv.DictReader(stream))), 3)
        self.assertIn('&lt;expired&gt;', (recorder.path / 'index.html').read_text())
        self.assertNotEqual(recorder.path, self.recorder().path)

    def test_image_write_error_is_explicit(self):
        recorder = self.recorder()
        recorder.cv.imwrite = lambda *args: False
        with self.assertRaises(OSError):
            recorder.save(np.zeros((2, 2, 3)), dict(valid=True), [])
        self.assertEqual(recorder.count, 0)

    def test_ros_source_positions_are_added_to_gallery_json_and_csv(self):
        recorder = self.recorder()
        recorder.save(np.zeros((4, 5, 3), dtype=np.uint8),
                      dict(valid=True, frame_stamp_unix=1000.25, confidence=0,
                           detected=False, frame_positive=False), [])
        event = dict(topic='sample', data=dict(frame_stamp_unix=1000.25,
                                              source_frame=42, video_time_sec=1.68))
        (self.root / 'events.jsonl').write_text(json.dumps(event) + '\n')
        add_source_positions(self.root)
        row = json.loads((recorder.path / 'results.jsonl').read_text())
        self.assertEqual((row['source_frame'], row['video_time_sec']), (42, 1.68))
        self.assertIn('1.680 s', (recorder.path / 'index.html').read_text())
        with (recorder.path / 'results.csv').open() as stream:
            self.assertEqual(next(csv.DictReader(stream))['source_frame'], '42')

    def test_node_records_negative_and_positive_actual_images_and_not_warmup(self):
        recorder = self.recorder()
        Node.overrides = {'record_directory': str(self.root), 'inference_rate_hz': 30.0}
        scores = iter([0.0, .8, .9])
        detector = AnimalDetector(lambda cfg: NS(warmup=lambda: .99,
                                  predict=lambda image: next(scores)),
                                  recorder_factory=lambda *args: recorder)
        self.addCleanup(detector.close)
        wait_until(lambda: detector.model_ready)
        self.assertEqual(recorder.count, 0)
        enable(detector)
        for i in range(3):
            detector.on_image(frame(i + 1))
            wait_until(lambda: recorder.count == i + 1)
        rows = [json.loads(s) for s in (recorder.path / 'results.jsonl').read_text().splitlines()]
        self.assertEqual([r['confidence'] for r in rows], [0, .8, .9])
        self.assertEqual([r['detected'] for r in rows], [False, False, True])
        self.assertEqual((recorder.path / rows[0]['image']).read_bytes(), b'\x01\x01\x01')

    def test_recording_failure_cannot_suppress_confirmed_stop_event(self):
        recorder = Mock(path=self.root)
        recorder.save.side_effect = OSError('disk full')
        Node.overrides = {'record_directory': str(self.root), 'inference_rate_hz': 30.0}
        detector = AnimalDetector(lambda cfg: NS(predict=lambda image: .9),
                                  recorder_factory=lambda *args: recorder)
        self.addCleanup(detector.close)
        wait_until(lambda: detector.model_ready)
        enable(detector)
        for i in range(2):
            detector.on_image(frame())
            wait_until(lambda: recorder.save.call_count == i + 1)
        detector.publish_result()
        self.assertTrue(json.loads(detector.result_pub.messages[-1].data)['detected'])
        detector.publish_status()
        self.assertIn('disk full', json.loads(detector.status_pub.messages[-1].data)['recording_error'])

    def test_entire_video_continues_after_confirmation_with_source_timestamps(self):
        class Reader:
            fps = 25.0
            frame_index = -1
            finished = False
            def read_at(self, elapsed):
                if elapsed >= 6:
                    self.finished = True
                    return None
                self.frame_index = round(elapsed * self.fps)
                return np.zeros((4, 5, 3), dtype=np.uint8)
            def close(self):
                self.closed = True
        reader = Reader()
        profile = dict(imgsz=448, conf=.2, nms_iou=.6, max_det=300, wolf_id=0)
        (self.root / 'inference_config.json').write_text(json.dumps(profile))
        scores = iter([.9, 0, .9, 0, 0, 0])
        recorder = self.recorder()
        result = analyze(self.root / 'video.mp4', self.root / 'model.pt', self.root,
                         backend_factory=lambda cfg: NS(warmup=lambda: None,
                             predict_details=lambda img: dict(confidence=next(scores), boxes=[])),
                         reader_factory=lambda *args, **kwargs: reader,
                         recorder_factory=lambda *args: recorder)
        rows = [json.loads(s) for s in (result / 'results.jsonl').read_text().splitlines()]
        self.assertEqual([r['source_frame'] for r in rows], [0, 25, 50, 75, 100, 125])
        self.assertEqual([r['detected'] for r in rows], [False, False, True, True, False, False])
        self.assertTrue(reader.closed)
        self.assertEqual(json.loads((result / 'summary.json').read_text())['state'], 'complete')


class RosReviewTests(unittest.TestCase):
    def test_full_video_preserves_flag_but_does_not_finish_until_eof(self):
        state = self.ready_state()
        state.full_video = True
        state.receive('mission', dict(mode='DRY_RUN_ONLY', state='WOLF_DETECTED'), 10)
        state.receive('flag', True, 10)
        state.receive('control', dict(enabled=False), 10)
        self.assertIsNone(state.outcome(10))
        self.assertTrue(state.confirmed)
        state.receive('video', dict(state='PLAYING', playback_sec=100), 11)
        self.assertIsNone(state.outcome(11))
        state.receive('video', dict(state='EOF'), 12)
        self.assertIsNone(state.outcome(12))
        for key in ('video', 'detector', 'mission', 'control'):
            state.receive(key, state.latest[key][0], 15)
        self.assertEqual(state.outcome(15), 'EOF_AFTER_WOLF_DETECTED')

    def test_full_video_diagnostic_control_waits_for_manager_then_keeps_analyzing(self):
        controller = DiagnosticControl()
        self.assertFalse(controller.command(dict(enabled=False), 10)['enabled'])
        self.assertFalse(controller.command(dict(enabled=True), 11)['enabled'])
        first = controller.command(dict(enabled=True, mission_id='m', session_id='s'), 12)
        self.assertTrue(first['enabled'])
        after_stop = controller.command(dict(enabled=False, mission_id='', session_id=''), 13)
        self.assertEqual(after_stop, dict(enabled=True, mission_id='m', session_id='s', stamp_unix=13))

    def ready_state(self, now=10):
        state = ReviewState()
        for key, data in (
                ('mission', dict(mode='DRY_RUN_ONLY', state='NAVIGATING_DRY_RUN')),
                ('detector', dict(state='RUNNING', recording_error='')),
                ('video', dict(state='PLAYING')), ('control', dict(enabled=True)),
                ('flag', False), ('result', dict(valid=True, detected=False))):
            state.receive(key, data, now)
        return state

    def test_only_real_manager_flag_and_disabled_control_finish_positive_trial(self):
        state = self.ready_state()
        state.receive('result', dict(valid=True, detected=True), 10)
        self.assertIsNone(state.outcome(10))
        state.receive('mission', dict(mode='DRY_RUN_ONLY', state='WOLF_DETECTED'), 10)
        state.receive('flag', True, 10)
        self.assertIsNone(state.outcome(10))
        state.receive('control', dict(enabled=False), 10)
        self.assertEqual(state.outcome(10), 'WOLF_DETECTED')
        self.assertIsNone(state.outcome(14))  # old state is not a completed live test

    def test_eof_waits_for_last_inference_and_requires_valid_results(self):
        state = self.ready_state()
        state.receive('video', dict(state='EOF'), 10)
        self.assertIsNone(state.outcome(10))
        for key in ('video', 'detector', 'mission', 'control'):
            state.receive(key, state.latest[key][0], 13)
        self.assertEqual(state.outcome(13), 'EOF_WITHOUT_CONFIRMATION')
        state.counts['valid_results'] = 0
        with self.assertRaisesRegex(RuntimeError, 'without a valid'):
            state.outcome(13)

    def test_motion_recording_and_model_errors_fail_explicitly(self):
        state = self.ready_state()
        state.receive('detector', dict(state='RUNNING', recording_error='disk full'), 10)
        with self.assertRaisesRegex(RuntimeError, 'disk full'):
            state.outcome(10)
        state = self.ready_state()
        state.receive('mission', dict(mode='LIVE', state='NAVIGATING_ACTIVE'), 10)
        with self.assertRaisesRegex(RuntimeError, 'motion disabled'):
            state.outcome(10)
        state = self.ready_state()
        state.violation = 'Unexpected nonzero velocity'
        with self.assertRaisesRegex(RuntimeError, 'nonzero'):
            state.outcome(10)

    def test_supervisor_stops_only_owned_ros_launcher(self):
        import signal
        process = Mock()
        process.poll.return_value = None
        stop_launch(process)
        process.send_signal.assert_called_once_with(signal.SIGINT)
        process.wait.assert_called_once_with(timeout=20)
