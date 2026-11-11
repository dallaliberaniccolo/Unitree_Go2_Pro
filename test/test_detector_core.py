import unittest
from types import SimpleNamespace as NS

from go2_animal_detector.core import ControlLease, DetectionWindow, image_to_bgr, resolve_target_classes


class DetectorCoreTests(unittest.TestCase):
    def test_two_distinct_nonconsecutive_hits_in_last_three_seconds(self):
        window = DetectionWindow()
        self.assertFalse(window.add(10, .8)['detected'])
        self.assertFalse(window.add(11, .0)['detected'])
        self.assertTrue(window.add(12, .9)['detected'])
        self.assertTrue(window.add(13, .0)['detected'])  # inclusive 3-second boundary
        self.assertFalse(window.add(13.01, .0)['detected'])
        self.assertIsNone(window.add(13.01, .9))
        window.clear()
        self.assertFalse(window.add(14, .9)['detected'])

    def test_age_is_checked_at_result_time_and_all_frames_in_window_count(self):
        window = DetectionWindow()
        window.add(10, .9, now=10.5)
        self.assertFalse(window.add(12.9, .9, now=13.1)['detected'])
        window.clear()
        window.add(20, .9)
        for i in range(1, 30):
            window.add(20 + i / 10, .0)
        self.assertTrue(window.add(23, .9)['detected'])  # not limited to last five frames

    def test_one_fps_confirmation_handles_timer_jitter_and_expires_old_votes(self):
        window = DetectionWindow(required=3, size=5, max_age=5.0)
        self.assertFalse(window.add(10.0, .9)['detected'])
        self.assertFalse(window.add(11.1, .9)['detected'])
        self.assertTrue(window.add(12.2, .9)['detected'])
        self.assertFalse(window.add(18.0, .9)['detected'])

    def test_confirm_three_of_five_unique_recent_images(self):
        window = DetectionWindow(required=3, size=5, max_age=2.0)
        for stamp, score, detected in ((1, .8, False), (1.2, .1, False), (1.4, .9, False),
                                       (1.6, .9, True), (1.8, .1, True)):
            self.assertEqual(window.add(stamp, score)['detected'], detected)
        self.assertIsNone(window.add(1.8, 1.0))
        self.assertFalse(window.add(5.0, .9)['detected'])

    def test_duplicates_cannot_confirm_and_new_session_clears_votes(self):
        window = DetectionWindow(required=3, size=5, max_age=2.0)
        window.add(1, .9)
        for _ in range(10):
            self.assertIsNone(window.add(1, .9))
        self.assertFalse(window.add(1.2, .9)['detected'])
        window.clear()
        self.assertFalse(window.add(1.4, .9)['detected'])

    def test_lease_expires_and_new_generation_invalidates_running_work(self):
        lease = ControlLease(1.5)
        command = dict(enabled=True, mission_id='m', session_id='s', stamp_unix=100)
        self.assertTrue(lease.receive(command, 1, 100))
        generation = lease.generation
        self.assertFalse(lease.receive(dict(command, stamp_unix=99), 1.1, 100.1))
        self.assertFalse(lease.receive(dict(command, stamp_unix=102), 1.1, 100.1))
        self.assertEqual(lease.active(2), ('m', 's'))
        self.assertIsNone(lease.active(2.6))
        self.assertGreater(lease.generation, generation)
        self.assertTrue(lease.receive(dict(command, stamp_unix=102), 3, 102))
        self.assertGreater(lease.generation, generation)
        self.assertTrue(lease.receive(dict(command, enabled=False, stamp_unix=102.1), 3.1, 102.1))
        self.assertIsNone(lease.active(3.1))

    def test_target_metadata_does_not_assume_class_zero_is_wolf(self):
        self.assertEqual(resolve_target_classes({0: 'dog', 1: 'Wolf'}), [1])
        self.assertEqual(resolve_target_classes(['Lupo']), [0])
        self.assertEqual(resolve_target_classes(['animal'], class_id=0), [0])
        with self.assertRaises(ValueError):
            resolve_target_classes(['dog', 'cat'])
        with self.assertRaises(ValueError):
            resolve_target_classes(['wolf'], class_id=2)

    def test_rgb_padding_and_mono_conversion(self):
        image = NS(encoding='rgb8', height=2, width=1, step=4, data=bytes([1, 2, 3, 99, 4, 5, 6, 99]))
        bgr = image_to_bgr(image)
        self.assertEqual(bgr.tolist(), [[[3, 2, 1]], [[6, 5, 4]]])
        self.assertTrue(bgr.flags.c_contiguous)
        image = NS(encoding='mono8', height=1, width=2, step=2, data=bytes([10, 20]))
        self.assertEqual(image_to_bgr(image).tolist(), [[[10, 10, 10], [20, 20, 20]]])
        image.step = 1
        with self.assertRaises(ValueError):
            image_to_bgr(image)

    def test_broken_image_and_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            image_to_bgr(NS(encoding='bgr8', height=1, width=2, step=6, data=b'bad'))
        for params in (dict(threshold=float('nan')), dict(required=6, size=5), dict(max_age=0)):
            with self.assertRaises(ValueError):
                DetectionWindow(**params)
