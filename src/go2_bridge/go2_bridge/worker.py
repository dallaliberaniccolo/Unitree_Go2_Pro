"""Owns the transport in one asyncio thread; no FIFO of velocity requests."""
import asyncio
import math
import threading
import time
from .backends.webrtc import MockBackend, WebRTCBackend


class TransportWorker:
    def __init__(self, gate, config):
        self.gate, self.config = gate, config
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._entry, name='go2-transport', daemon=True)
        self.raw_state = None
        self.state_time = -math.inf
        self.state_wall = 0
        self.frame = None
        self.error = ''
        self.stop_sent = False
        self.stop_count = 0
        self.last_stop = -math.inf

    def start(self):
        self.thread.start()

    def on_state(self, data, received, wall):
        with self.lock:
            self.raw_state, self.state_time, self.state_wall = data, received, wall

    def on_frame(self, frame, wall):
        with self.lock:
            self.frame = (frame, wall)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            result = self.gate.snapshot(now)
            result.update(stop_sent=self.stop_sent, stop_confirmed=False,
                          telemetry_age=now-self.state_time, transport_error=self.error)
            return result

    def _entry(self):
        try:
            asyncio.run(self._run())
        except Exception as exc:
            with self.lock:
                self.error = type(exc).__name__
                self.gate.update_robot(False, False)

    async def _run(self):
        backend_type = {'webrtc': WebRTCBackend, 'mock': MockBackend}[self.config['backend']]
        while not self.done.is_set():
            backend = None
            try:
                with self.lock:
                    self.raw_state, self.frame = None, None
                    self.state_time = -math.inf
                backend = backend_type(self.config, self.on_state, self.on_frame)
                await asyncio.wait_for(backend.connect(), self.config['connect_timeout'])
                with self.lock:
                    self.error = ''
                    self.stop_count = 0
                    self.last_stop = -math.inf
                while not self.done.is_set():
                    now = time.monotonic()
                    healthy = backend.healthy()
                    with self.lock:
                        mode = (self.raw_state or {}).get('mode')
                        ready = (now-self.state_time < self.config['telemetry_timeout']
                                 and type(mode) is int
                                 and mode in self.config['ready_modes'])
                        self.gate.update_robot(healthy, ready)
                        if not healthy:
                            raise ConnectionError('Connection lost')
                        if self.gate.run_id and backend.congested():
                            self.gate.trip('transport_congested')
                        velocity = self.gate.output(now)
                        # Final gate check and synchronous send share the same lock.
                        if self.gate.stop_latched and self.gate.motion_enabled:
                            if self.stop_count < 10 and now-self.last_stop >= 0.1:
                                backend.stop()
                                self.stop_sent = True
                                self.stop_count += 1
                                self.last_stop = now
                        elif velocity is not None:
                            backend.move(velocity)
                        else:
                            self.stop_count = 0
                            self.stop_sent = False
                    await asyncio.sleep(1.0 / self.config['command_rate'])
            except Exception as exc:
                with self.lock:
                    # Avoid logging credentials or full signaling responses.
                    self.error = type(exc).__name__
                    self.gate.update_robot(False, False)
            finally:
                if backend:
                    # Best effort only: no claim of physical stop confirmation.
                    try:
                        if self.gate.motion_enabled and (self.gate.run_id or self.gate.stop_latched):
                            if backend.healthy():
                                backend.stop()
                        await asyncio.wait_for(backend.close(), 3.0)
                    except Exception:
                        pass
                with self.lock:
                    self.gate.update_robot(False, False)
            until = time.monotonic() + self.config['reconnect_delay']
            while not self.done.is_set() and time.monotonic() < until:
                await asyncio.sleep(0.1)

    def close(self):
        self.done.set()
        # Upstream signaling includes blocking HTTP: timeout is not a hard bound.
        self.thread.join(timeout=4.0)
        return not self.thread.is_alive()
