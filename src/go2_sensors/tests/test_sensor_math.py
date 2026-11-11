import math
import unittest
from go2_sensors.math_sensor import apply_axis_map,calibrated_field,calibration_from_json,tilt_compass_deg
from go2_sensors.gps_protocol import parse_nmea

class SensorMathTests(unittest.TestCase):
    def test_nmea_rmc(self):
        res=parse_nmea('$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A')
        self.assertEqual(res[0],'RMC')
        self.assertTrue(res[1])
        self.assertAlmostEqual(res[2],48.1173,places=4)
        self.assertAlmostEqual(res[3],11.51666666,places=4)
    def test_nmea_gga(self):
        self.assertEqual(parse_nmea('$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47'),('GGA',1,8))
    def test_nmea_reject_checksum(self):
        self.assertIsNone(parse_nmea('$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00'))
    def test_heading_cardinals(self):
        self.assertAlmostEqual(tilt_compass_deg((0,0,1),(50,0,0)),0)
        self.assertAlmostEqual(tilt_compass_deg((0,0,1),(0,50,0)),90)
        self.assertAlmostEqual(tilt_compass_deg((0,0,1),(0,-50,0)),270)
    def test_axis_map(self):
        self.assertEqual(apply_axis_map((1,2,3),(2,-1,3)),(2.,-1.,3.))
        with self.assertRaises(ValueError):apply_axis_map((1,2,3),(1,1,3))
    def test_calibration(self):
        raw={'offset_ut':[10,20,30],'scale_axes':[1,2,0.5],
             'intensita_riferimento_ut':40,'variazione_p10_p90_percento':49.23}
        off,scale,ref,spread=calibration_from_json(raw)
        self.assertEqual(calibrated_field((11,22,32),off,scale),(1.,4.,1.))
        self.assertGreater(spread,20)

if __name__=='__main__':unittest.main()
