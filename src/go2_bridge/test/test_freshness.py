import unittest
from go2_bridge.core import remaining_ttl


class FreshnessTests(unittest.TestCase):
    def test_delay_consumes_lease(self):
        self.assertAlmostEqual(remaining_ttl(100., .6, 100.2, .6, .05), .4)

    def test_huge_ttl_cannot_revive_old_message(self):
        self.assertIsNone(remaining_ttl(100., 1000., 101., .6, .05))

    def test_unstamped_rejected(self):
        self.assertIsNone(remaining_ttl(0., .6, 100., .6, .05))

    def test_future_stamp_rejected(self):
        self.assertIsNone(remaining_ttl(101., .6, 100., .6, .05))

    def test_tolerated_clock_skew_does_not_extend_ttl(self):
        self.assertEqual(remaining_ttl(100.01, .6, 100., .6, .05), .6)

    def test_invalid_duration_rejected(self):
        self.assertIsNone(remaining_ttl(100., -1., 100., .6, .05))
