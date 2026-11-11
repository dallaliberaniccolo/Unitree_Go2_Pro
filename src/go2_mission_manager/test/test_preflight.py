import unittest
from types import SimpleNamespace
from go2_mission_manager.node import MissionManager


class PreflightTests(unittest.TestCase):
    def make_manager(self, goal=None):
        nav = dict(goal=goal, gps_ok=True, satellites_used=12,
                   imu_source='external_icm20948', external_imu_ok=True,
                   imu_ok=True, gyro_calibrated=True, heading_ok=True,
                   unitree_imu_ok=True, go2_telemetry_ok=True, go2_error_code=0,
                   distance_m=20.0, heading_error_deg=5.0, ready_for_dry_run=True)
        bridge = SimpleNamespace(connected=True, robot_ready=True,
                                 motion_enabled=True, stop_requested=False,
                                 transport_error='', rejected_messages=0)
        return SimpleNamespace(nav_received=10.0, bridge_received=10.0,
                               status_max_age=1.0, nav=nav, bridge=bridge,
                               logic=SimpleNamespace(goal=goal), min_satellites=7,
                               enable_motion=True, sport_state_code_is_fault=False,
                               _last_rejected_messages=None)

    def test_idle_reports_actual_gps_and_no_distance(self):
        manager = self.make_manager()
        manager.nav['goal'] = [45.0, 11.0]  # previous goal in another node
        checks = MissionManager._preflight(manager, 10.1)
        self.assertTrue(checks['gps_ok'])
        self.assertFalse(checks['nav_ready'])
        self.assertIsNone(checks['distance'])
        self.assertIsNone(checks['heading_error'])
        self.assertEqual(checks['errors'], [])

    def test_mismatched_waypoint_blocks_motion_without_false_gps_fault(self):
        manager = self.make_manager([45.0, 11.0])
        manager.nav['goal'] = [46.0, 12.0]
        checks = MissionManager._preflight(manager, 10.1)
        self.assertTrue(checks['gps_ok'])
        self.assertFalse(checks['nav_ready'])
        self.assertIsNone(checks['distance'])
        self.assertIn('navigation_has_not_received_current_waypoint', checks['errors'])
        self.assertNotIn('gps_fix_or_quality_invalid', checks['errors'])

    def test_valid_matching_goal_is_ready(self):
        checks = MissionManager._preflight(self.make_manager([45.0, 11.0]), 10.1)
        self.assertTrue(checks['nav_ready'])
        self.assertTrue(checks['bridge_ready'])
        self.assertEqual(checks['errors'], [])

    def test_stale_telemetry_blocks(self):
        checks = MissionManager._preflight(self.make_manager([45.0, 11.0]), 12.0)
        self.assertFalse(checks['nav_ready'])
        self.assertFalse(checks['bridge_ready'])
        self.assertIn('navigation_status_missing_or_stale', checks['errors'])
        self.assertIn('bridge_state_missing_or_stale', checks['errors'])

    def test_go2_pro_state_codes_do_not_create_false_faults(self):
        for code in (1001, 1013):
            with self.subTest(code=code):
                manager = self.make_manager([45.0, 11.0])
                manager.nav['go2_error_code'] = code
                checks = MissionManager._preflight(manager, 10.1)
                self.assertTrue(checks['nav_ready'])
                self.assertEqual(checks['errors'], [])

    def test_fault_code_profile_still_blocks_nonzero_faults(self):
        manager = self.make_manager([45.0, 11.0])
        manager.sport_state_code_is_fault = True
        manager.nav['go2_error_code'] = 4201
        checks = MissionManager._preflight(manager, 10.1)
        self.assertFalse(checks['nav_ready'])
        self.assertIn('go2_fault:4201', checks['errors'])

    def test_informational_api_code_does_not_bypass_sensor_gates(self):
        for field in ('gps_ok', 'external_imu_ok', 'heading_ok',
                      'unitree_imu_ok', 'go2_telemetry_ok'):
            with self.subTest(field=field):
                manager = self.make_manager([45.0, 11.0])
                manager.nav['go2_error_code'] = 1013
                manager.nav[field] = False
                checks = MissionManager._preflight(manager, 10.1)
                self.assertFalse(checks['nav_ready'])
                self.assertTrue(checks['errors'])

    def test_informational_api_code_does_not_bypass_bridge_stop(self):
        manager = self.make_manager([45.0, 11.0])
        manager.nav['go2_error_code'] = 1013
        manager.bridge.stop_requested = True
        manager.bridge.reason = 'stop_service'
        checks = MissionManager._preflight(manager, 10.1)
        self.assertFalse(checks['bridge_ready'])
        self.assertIn('bridge_stop_latched:stop_service', checks['errors'])

    def test_bounded_magnetic_hold_allowed_with_valid_heading(self):
        manager = self.make_manager([45.0, 11.0])
        manager.nav['heading_source'] = 'unitree_imu_magnetic_hold'
        checks = MissionManager._preflight(manager, 10.1)
        self.assertTrue(checks['nav_ready'])
        self.assertIn('heading_using_bounded_unitree_imu_magnetic_hold', checks['warnings'])


if __name__ == '__main__':
    unittest.main()
