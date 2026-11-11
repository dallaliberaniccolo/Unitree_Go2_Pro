import unittest

from go2_mission_manager.identification import IdentificationGate


class IdentificationTests(unittest.TestCase):
    def test_radius_matches_examples_and_has_no_cap(self):
        gate = IdentificationGate()
        gate.new_goal(100.0)
        for seconds, radius in ((0, 50), (60, 100), (300, 300), (600, 500), (1200, 900), (1800, 1300), (3600, 2500)):
            with self.subTest(seconds=seconds):
                self.assertAlmostEqual(gate.radius(100 + seconds), radius)

    def test_delayed_start_counts_time_since_receipt(self):
        gate = IdentificationGate()
        gate.new_goal(10)
        self.assertFalse(gate.update('LOADED', 800, True, 1209, 2000))
        self.assertTrue(gate.update('WAIT_LOCALIZATION', 800, True, 1210, 2001))

    def test_initial_threshold_and_bad_distance(self):
        gate = IdentificationGate()
        gate.new_goal(0)
        for distance, valid in ((49, False), (51, True), (None, True), (float('nan'), True), (-1, True)):
            self.assertFalse(gate.update('NAVIGATING_ACTIVE', distance, valid, 0, 100))
        self.assertTrue(gate.update('NAVIGATING_ACTIVE', 50, True, 0, 100))
        self.assertTrue(gate.update('GPS_GRACE_HOLD', None, False, 1, 101))
        self.assertTrue(gate.update('NAVIGATING_ACTIVE', 500, True, 2, 102))

    def test_pause_resume_rejects_old_session_without_resetting_age(self):
        gate = IdentificationGate()
        gate.new_goal(0)
        gate.update('NAVIGATING_ACTIVE', 20, True, 60, 1000)
        old_session = gate.session_id
        self.assertFalse(gate.update('PAUSED', 20, True, 61, 1001))
        gate.update('WAIT_LOCALIZATION', 20, True, 600, 1540)
        self.assertNotEqual(old_session, gate.session_id)
        self.assertEqual(gate.radius(600), 500)
        self.assertFalse(gate.accept_result(self.result(gate, session_id=old_session, frame_stamp_unix=1540), 1540.1))

    def result(self, gate, **values):
        data = dict(mission_id=gate.mission_id, session_id=gate.session_id, detected=True,
                    valid=True, frame_stamp_unix=1000.1, stamp_unix=1000.2)
        data.update(values)
        return data

    def test_stale_future_invalid_and_previous_mission_results(self):
        gate = IdentificationGate()
        gate.new_goal(0)
        gate.update('NAVIGATING_ACTIVE', 10, True, 0, 1000)
        for change in (dict(mission_id='previous'), dict(session_id='previous'), dict(valid=False),
                       dict(detected='true'), dict(frame_stamp_unix=999.9),
                       dict(frame_stamp_unix=1004, stamp_unix=1004), dict(stamp_unix=float('nan'))):
            with self.subTest(change=change):
                self.assertFalse(gate.accept_result(self.result(gate, **change), 1000.3))
        self.assertFalse(gate.accept_result(self.result(gate), 1003))
        self.assertTrue(gate.accept_result(self.result(gate), 1000.3))
        self.assertFalse(gate.accept_result(self.result(gate), 1000.4))
        old = self.result(gate)
        gate.new_goal(1)
        gate.update('NAVIGATING_ACTIVE', 10, True, 1, 1001)
        self.assertFalse(gate.accept_result(old, 1001))

    def test_arrival_enables_analysis_but_idle_does_not(self):
        gate = IdentificationGate()
        self.assertFalse(gate.update('IDLE', 0, True, 1, 1))
        gate.new_goal(2)
        self.assertTrue(gate.update('ARRIVED_WAIT_IDENTIFICATION', None, False, 2, 2))
        gate.clear()
        self.assertFalse(gate.enabled)
        self.assertIsNone(gate.age(3))
