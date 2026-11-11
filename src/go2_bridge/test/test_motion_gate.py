import math
import unittest
from go2_bridge.core import MotionGate, Limits


class GateTests(unittest.TestCase):
    def setUp(self):
        self.g = MotionGate(motion_enabled=True)
        self.g.update_robot(True, True)
        self.epoch = self.g.epoch

    def grant(self, t=0., seq=0):
        return self.g.permit(self.epoch, 'mission-A', seq, True, .6, t)

    def command(self, t=.01, seq=0, velocity=(.1, 0., 0.)):
        return self.g.command(self.epoch, 'mission-A', seq, velocity, .3, t)

    def test_no_motion_at_boot(self):
        self.assertIsNone(self.g.output(0))

    def test_read_only_rejects_permit(self):
        self.g.motion_enabled = False
        self.assertFalse(self.grant())

    def test_no_permit_without_robot_ready(self):
        self.g.update_robot(True, False)
        self.assertFalse(self.grant())

    def test_command_before_permit_is_discarded(self):
        self.assertFalse(self.command(t=0))
        self.assertTrue(self.grant(t=.1))
        self.assertIsNone(self.g.output(.11))

    def test_valid_session(self):
        self.assertTrue(self.grant())
        self.assertTrue(self.command())
        self.assertEqual(self.g.output(.1), (.1, 0., 0.))

    def test_command_deadline_latches(self):
        self.grant(); self.command()
        self.assertIsNone(self.g.output(.31))
        self.assertEqual(self.g.reason, 'command_expired')
        self.assertTrue(self.g.stop_latched)

    def test_renewing_permit_does_not_refresh_command(self):
        self.grant(); self.command()
        self.assertTrue(self.grant(t=.2, seq=1))
        self.assertIsNone(self.g.output(.32))

    def test_expired_permit_cannot_be_revived(self):
        self.grant(); self.command()
        self.command(t=.2, seq=1); self.command(t=.4, seq=2)
        self.assertFalse(self.grant(t=.6, seq=1))
        self.assertEqual(self.g.reason, 'permit_expired')

    def test_replayed_sequence_rejected(self):
        self.grant(); self.command()
        self.assertFalse(self.command(t=.2, seq=0))
        self.assertFalse(self.grant(t=.2, seq=0))

    def test_wrong_session_rejected(self):
        self.grant()
        self.assertFalse(self.g.command(self.epoch, 'other', 1, (.1, 0., 0.), .3, .1))

    def test_disconnect_requires_new_epoch_and_reset(self):
        self.grant(); self.command()
        self.g.update_robot(False, False)
        self.g.update_robot(True, True)
        self.assertIsNone(self.g.output(.1))
        self.assertNotEqual(self.g.epoch, self.epoch)
        self.assertTrue(self.g.reset_stop())
        self.assertFalse(self.grant(t=.1, seq=2))

    def test_readiness_loss_revokes(self):
        self.grant(); self.command()
        self.g.update_robot(True, False)
        self.assertIsNone(self.g.output(.1))
        self.assertTrue(self.g.stop_latched)

    def test_stop_then_reset_requires_new_command(self):
        self.grant(); self.command()
        self.g.trip('operator_stop')
        self.assertTrue(self.g.reset_stop())
        self.epoch = self.g.epoch
        self.assertTrue(self.grant(t=.1))
        self.assertIsNone(self.g.output(.11))

    def test_reset_during_active_run_rejected(self):
        self.grant(); self.command()
        self.assertFalse(self.g.reset_stop())

    def test_invalid_velocity(self):
        for velocity in ((math.nan, 0., 0.), (math.inf, 0., 0.), (.16, 0., 0.)):
            with self.subTest(velocity=velocity):
                self.setUp(); self.grant()
                self.assertFalse(self.command(velocity=velocity))
                self.assertTrue(self.g.stop_latched)

    def test_revocation(self):
        self.grant(); self.command()
        self.assertTrue(self.g.permit(self.epoch, 'mission-A', 1, False, .5, .1))
        self.assertIsNone(self.g.output(.1))

    def test_ttl_capped(self):
        self.g.permit(self.epoch, 'mission-A', 0, True, 100, 0.)
        self.assertEqual(self.g.permit_until, .6)

    def test_no_command_after_grant_times_out(self):
        self.grant()
        self.assertIsNone(self.g.output(.3))
        self.assertTrue(self.g.stop_latched)

    def test_invalid_limits_rejected(self):
        with self.assertRaises(ValueError):
            Limits(command_timeout=-1)


if __name__ == '__main__':
    unittest.main()
