import json
import unittest
from types import SimpleNamespace
from go2_bridge.backends.webrtc import WebRTCBackend


class WireTests(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.b = WebRTCBackend.__new__(WebRTCBackend)
        self.b.commands = {'Move': 1008, 'StopMove': 1003}
        self.b.topics = {'SPORT_MOD': 'rt/api/sport/request'}
        self.b.request_id = 0
        self.b.conn = SimpleNamespace(
            pc=SimpleNamespace(connectionState='connected'),
            datachannel=SimpleNamespace(data_channel_opened=True,
                channel=SimpleNamespace(readyState='open'),
                pub_sub=SimpleNamespace(publish_without_callback=lambda *a: self.sent.append(a))))

    def test_move_envelope_and_axis_mapping(self):
        self.b.move((.1, -.1, .2))
        topic, payload = self.sent[0]
        self.assertEqual(topic, 'rt/api/sport/request')
        self.assertEqual(payload['header']['identity']['api_id'], 1008)
        self.assertTrue(payload['header']['policy']['noreply'])
        self.assertEqual(json.loads(payload['parameter']), {'x': .1, 'y': -.1, 'z': .2})

    def test_stop_has_no_posture_command(self):
        self.b.stop()
        self.assertEqual(self.sent[0][1]['header']['identity']['api_id'], 1003)

    def test_closed_channel_raises_instead_of_silent_drop(self):
        self.b.conn.datachannel.channel.readyState = 'closed'
        with self.assertRaises(ConnectionError):
            self.b.move((.1, 0, 0))
        self.assertFalse(self.sent)
