import json
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from ros_shim import Message, Node
from go2_mission_manager.node import MissionManager


class MissionDetectionTests(unittest.TestCase):
    def setUp(self):
        self.now, self.wall = 10.0, 1000.0
        fake_time = NS(monotonic=lambda: self.now, time=lambda: self.wall)
        self.timer_patch = patch('go2_mission_manager.node.time', fake_time)
        self.timer_patch.start()
        self.addCleanup(self.timer_patch.stop)
        Node.overrides = dict(enable_motion=True, sport_state_code_is_fault=False)
        self.manager = MissionManager()
        Node.overrides = {}
        self.manager.on_goal(Message(latitude=45.0, longitude=11.0))
        self.manager.bridge = NS(connected=True, robot_ready=True, motion_enabled=True,
                                 stop_requested=False, transport_error='', rejected_messages=0,
                                 reason='', bridge_epoch='bridge', stop_confirmed=False)
        self.manager.nav = dict(goal=[45.0, 11.0], gps_ok=True, satellites_used=12,
                                imu_source='external_icm20948', external_imu_ok=True,
                                imu_ok=True, gyro_calibrated=True, heading_ok=True,
                                unitree_imu_ok=True, go2_telemetry_ok=True, go2_error_code=0,
                                distance_m=20.0, heading_error_deg=0.0, ready_for_dry_run=True,
                                gps_updates=1, within_goal_tolerance=False)

    def tick(self, delta=0.0):
        self.now += delta
        self.wall += delta
        self.manager.nav_received = self.manager.bridge_received = self.now
        self.manager.tick()

    def result(self, **values):
        gate = self.manager.identification
        data = dict(mission_id=gate.mission_id, session_id=gate.session_id, valid=True,
                    detected=True, frame_stamp_unix=self.wall, stamp_unix=self.wall)
        data.update(values)
        return Message(data=json.dumps(data))

    def start_motion(self):
        self.tick()
        self.tick(1.6)
        self.tick(0.1)
        self.assertEqual(self.manager.logic.state, 'NAVIGATING_ACTIVE')
        self.assertGreater(self.manager.last_cmd_vel[0], 0)

    def test_detection_sends_zero_revokes_permit_stops_and_latches_flag(self):
        self.start_motion()
        manager = self.manager
        manager.on_identification(self.result())
        self.assertEqual(manager.logic.state, 'WOLF_DETECTED')
        self.assertTrue(manager.wolf_flag_pub.messages[-1].data)
        self.assertEqual(manager.cmd_pub.messages[-1].twist.linear.x, 0)
        self.assertFalse(manager.permit_pub.messages[-1].enabled)
        self.assertEqual(len(manager.stop_client.calls), 1)
        self.assertFalse(manager.identification.enabled)
        permit_count = len(manager.permit_pub.messages)
        self.tick(0.1)
        self.assertEqual(len(manager.permit_pub.messages), permit_count)
        manager.on_identification(self.result(detected=False))
        self.assertTrue(manager.logic.wolf_detected)
        response = manager.on_start(None, NS())
        self.assertFalse(response.success)
        self.assertFalse(manager._goal_rearm_authorized)

    def test_pause_blocks_late_result_and_same_goal_echo_preserves_timer(self):
        self.start_motion()
        manager = self.manager
        old = self.result()
        received = manager.identification.received_at
        manager.on_goal(Message(latitude=45.0, longitude=11.0))
        self.assertEqual(manager.identification.received_at, received)
        manager.on_pause(None, NS())
        manager.on_identification(old)
        self.assertFalse(manager.logic.wolf_detected)
        manager.on_start(None, NS())
        self.tick(0.1)
        manager.on_identification(old)
        self.assertFalse(manager.logic.wolf_detected)
        self.assertEqual(manager.identification.received_at, received)

    def test_new_goal_invalidates_results_and_resets_flag(self):
        self.start_motion()
        manager = self.manager
        old = self.result()
        manager.on_identification(old)
        manager.on_pause(None, NS())
        self.assertFalse(manager.on_start(None, NS()).success)
        manager.on_goal(Message(latitude=46.0, longitude=12.0))
        manager.on_identification(old)
        self.assertFalse(manager.logic.wolf_detected)
        self.assertFalse(manager.wolf_flag_pub.messages[-1].data)
        self.assertEqual(manager.identification.age(self.now), 0)

    def test_false_keeps_navigation_and_arrival_does_not_explore(self):
        self.start_motion()
        manager = self.manager
        manager.on_identification(self.result(detected=False))
        self.assertEqual(manager.logic.state, 'NAVIGATING_ACTIVE')
        manager.nav.update(distance_m=2.0, within_goal_tolerance=True, gps_updates=2)
        self.tick(0.1)
        manager.nav['gps_updates'] = 3
        self.tick(0.1)
        self.assertEqual(manager.logic.state, 'ARRIVED_WAIT_IDENTIFICATION')
        self.assertTrue(manager.identification.enabled)
        self.assertEqual(manager.last_cmd_vel, (0, 0, 0))
        manager.on_identification(self.result(detected=False))
        self.assertEqual(manager.logic.state, 'ARRIVED_WAIT_IDENTIFICATION')

    def test_invalid_status_cannot_poison_mission_json(self):
        self.start_motion()
        self.manager.on_detector_status(Message(data='{"inference_ms": NaN}'))
        self.manager.on_identification(self.result(extra=float('inf')))
        self.tick(0.1)
        self.assertFalse(self.manager.logic.wolf_detected)
        json.loads(self.manager.status_pub.messages[-1].data)
