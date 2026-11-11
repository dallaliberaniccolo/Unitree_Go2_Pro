"""Adapter for the upstream driver, revision in requirements-webrtc.txt.

Wire envelope adapted from examples/go2/data_channel/sportmode_mcf/
sportmode_mcf.py (MIT; notice in third_party). No mode/posture changes.
"""
import asyncio
import json
import os
import time


class WebRTCBackend:
    def __init__(self, config, on_state, on_frame):
        from unitree_webrtc_connect.webrtc_driver import (
            UnitreeWebRTCConnection, WebRTCConnectionMethod)
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD
        self.topics, self.commands = RTC_TOPIC, SPORT_CMD
        self.config, self.on_state, self.on_frame = config, on_state, on_frame
        method = config['connection_method']
        if method not in ('LocalSTA', 'LocalAP'):
            raise ValueError('Only local WebRTC connections are supported')
        options = {}
        if method == 'LocalSTA':
            ip = os.environ.get('UNITREE_ROBOT_IP') or config['robot_ip']
            if not ip:
                raise ValueError('Set robot_ip or UNITREE_ROBOT_IP')
            options['ip'] = ip
        if os.environ.get('UNITREE_AES_128_KEY'):
            options['aes_128_key'] = os.environ['UNITREE_AES_128_KEY']
        self.conn = UnitreeWebRTCConnection(getattr(WebRTCConnectionMethod, method), **options)
        self.request_id = 0

    async def connect(self):
        await self.conn.connect()
        self.conn.datachannel.pub_sub.subscribe(
            self.topics['LF_SPORT_MOD_STATE'], self._state)
        if self.config['video_enabled']:
            self.conn.video.add_track_callback(self._video)
            self.conn.video.switchVideoChannel(True)

    def _state(self, message):
        data = message.get('data')
        if isinstance(data, dict):
            self.on_state(data, time.monotonic(), time.time_ns())

    async def _video(self, track):
        # Keep AV frames only; conversion to BGR happens outside the WebRTC loop.
        while True:
            frame = await track.recv()
            self.on_frame(frame, time.time_ns())

    def healthy(self):
        pc = self.conn.pc
        dc = getattr(self.conn, 'datachannel', None)
        return bool(pc and pc.connectionState == 'connected' and dc
                    and dc.data_channel_opened and dc.channel.readyState == 'open')

    def congested(self):
        return self.conn.datachannel.channel.bufferedAmount > self.config['max_buffered_bytes']

    def _send(self, name, parameter=None):
        if not self.healthy():
            raise ConnectionError('WebRTC channel unavailable')
        self.request_id = (self.request_id + 1) % 2147483647
        payload = {
            'header': {'identity': {'id': self.request_id, 'api_id': self.commands[name]},
                       'policy': {'priority': 0, 'noreply': True}},
            'parameter': json.dumps(parameter) if parameter is not None else '',
            'binary': [],
        }
        self.conn.datachannel.pub_sub.publish_without_callback(self.topics['SPORT_MOD'], payload)

    def _virtual_joystick(self, velocity):
        # Unitree wireless-controller axes are NORMALIZED joystick inputs.
        # These are NOT metrical speed commands; do not use for odometry.
        vx, vy, wz = velocity
        def scaled(value, limit, maximum):
            return max(-maximum, min(maximum, (value / limit) * maximum))
        payload = {
            'lx': scaled(-vy, self.config['max_vy'], self.config['joystick_max_lateral']),
            'ly': scaled(vx, self.config['max_vx'], self.config['joystick_max_forward']),
            'rx': scaled(wz, self.config['max_wz'], self.config['joystick_max_yaw']),
            'ry': 0.0,
            'keys': 0,
        }
        if not self.healthy():
            raise ConnectionError('WebRTC channel unavailable')
        self.conn.datachannel.pub_sub.publish_without_callback(
            self.topics['WIRELESS_CONTROLLER'], payload)

    def move(self, velocity):
        if getattr(self, 'config', {}).get('motion_transport', 'sport') == 'virtual_joystick':
            self._virtual_joystick(velocity)
        else:
            self._send('Move', dict(zip(('x', 'y', 'z'), velocity)))

    def stop(self):
        if getattr(self, 'config', {}).get('motion_transport', 'sport') == 'virtual_joystick':
            self._virtual_joystick((0.0, 0.0, 0.0))
        self._send('StopMove')

    async def close(self):
        await self.conn.disconnect()


class MockBackend:
    """No robot connection. Emits only synthetic mode telemetry; never a permit."""
    def __init__(self, config, on_state, on_frame):
        self.on_state = on_state
        self.up = False
        self.last_state = 0.0
        self.last_velocity = None

    async def connect(self):
        self.up = True

    def healthy(self):
        now = time.monotonic()
        if self.up and now - self.last_state >= 0.1:
            self.last_state = now
            self.on_state({'mode': 2, 'mock': True}, now, time.time_ns())
        return self.up

    def congested(self):
        return False

    def move(self, velocity):
        self.last_velocity = velocity

    def stop(self):
        self.last_velocity = (0., 0., 0.)

    async def close(self):
        self.up = False
