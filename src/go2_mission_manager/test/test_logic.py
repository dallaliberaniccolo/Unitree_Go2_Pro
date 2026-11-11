import math
import unittest

from go2_mission_manager.logic import MissionLogic


class MissionLogicTests(unittest.TestCase):
    def update(self, mission, *, now, gps=True, nav=True, bridge=True, within=False,
               gps_updates=1, distance=12.0, stable=1.5):
        mission.update(
            gps_ok=gps,
            nav_ready=nav,
            bridge_connected=bridge,
            bridge_ready=bridge,
            within_goal=within,
            gps_updates=gps_updates,
            enable_motion=True,
            now=now,
            readiness_stable_sec=stable,
            distance_m=distance,
        )

    def test_waypoint_waits_for_stable_readiness(self):
        mission = MissionLogic()
        mission.load_goal([45.4, 11.7])
        self.assertTrue(mission.start()[0])
        self.update(mission, now=1.0, gps=False)
        self.assertEqual(mission.state, 'WAIT_LOCALIZATION')
        self.update(mission, now=2.0)
        self.assertEqual(mission.state, 'WAIT_LOCALIZATION')
        self.update(mission, now=3.6)
        self.assertEqual(mission.state, 'NAVIGATING_ACTIVE')

    def test_sensor_loss_holds_then_resumes_automatically(self):
        mission = MissionLogic()
        mission.load_goal([45.4, 11.7])
        mission.start()
        self.update(mission, now=1.0)
        self.update(mission, now=2.6)
        self.assertEqual(mission.state, 'NAVIGATING_ACTIVE')
        self.update(mission, now=2.7, gps=False)
        self.assertEqual(mission.state, 'GPS_GRACE_HOLD')
        self.update(mission, now=3.0)
        self.update(mission, now=4.6)
        self.assertEqual(mission.state, 'NAVIGATING_ACTIVE')

    def test_arrival_uses_distinct_gps_updates_and_freezes_distance(self):
        mission = MissionLogic(arrival_samples=2)
        mission.load_goal([45.4, 11.7])
        mission.start()
        self.update(mission, now=1.0)
        self.update(mission, now=2.6)
        self.update(mission, now=2.7, within=True, gps_updates=10, distance=2.4)
        self.assertEqual(mission.state, 'NAVIGATING_ACTIVE')
        self.update(mission, now=2.8, within=True, gps_updates=10, distance=2.2)
        self.assertEqual(mission.state, 'NAVIGATING_ACTIVE')
        self.update(mission, now=2.9, within=True, gps_updates=11, distance=1.8)
        self.assertEqual(mission.state, 'ARRIVED_WAIT_IDENTIFICATION')
        self.assertTrue(math.isclose(mission.arrival_distance_m, 1.8))
        self.update(mission, now=3.0, gps_updates=12, distance=19.4)
        self.assertEqual(mission.state, 'ARRIVED_WAIT_IDENTIFICATION')
        self.assertTrue(math.isclose(mission.arrival_distance_m, 1.8))

    def test_operator_pause_does_not_auto_resume(self):
        mission = MissionLogic()
        mission.load_goal([45.4, 11.7])
        mission.start()
        self.update(mission, now=1.0)
        self.update(mission, now=2.6)
        mission.pause('user_paused')
        self.update(mission, now=10.0)
        self.assertEqual(mission.state, 'PAUSED')

    def test_bad_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            MissionLogic(loss_timeout=0)


if __name__ == '__main__':
    unittest.main()
