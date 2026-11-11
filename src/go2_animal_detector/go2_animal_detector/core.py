"""ROS-independent filtering, control lease and image conversion."""
from collections import deque
import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class DetectionWindow:
    def __init__(self, threshold=0.2, required=2, size=0, max_age=3.0):
        if (not finite(threshold) or not 0 < threshold <= 1
                or not isinstance(required, int) or not isinstance(size, int)
                or required < 1 or size < 0 or (size and required > size)
                or not finite(max_age) or max_age <= 0):
            raise ValueError('Invalid temporal confirmation parameters')
        self.threshold, self.required, self.max_age = threshold, required, max_age
        self.samples = deque(maxlen=size or None)  # zero: time window, no frame-count cap
        self.last_stamp = None

    def clear(self):
        self.samples.clear()
        self.last_stamp = None

    def add(self, stamp, confidence, now=None):
        now = stamp if now is None else now
        if (not finite(stamp) or not finite(confidence) or not 0 <= confidence <= 1
                or not finite(now) or now < stamp
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            return None  # duplicates and out-of-order frames never count
        self.last_stamp = stamp
        while self.samples and now - self.samples[0][0] > self.max_age:
            self.samples.popleft()
        hit = confidence >= self.threshold
        if now - stamp <= self.max_age:
            self.samples.append((stamp, hit))
        hits = sum(item[1] for item in self.samples)
        return dict(detected=bool(hits >= self.required), frame_positive=hit,
                    positive_frames=hits, window_frames=len(self.samples))


class ControlLease:
    """A missing manager stops inference; generation also invalidates in-flight work."""
    def __init__(self, timeout=1.5):
        if not finite(timeout) or timeout <= 0:
            raise ValueError('Control timeout must be positive')
        self.timeout = timeout
        self.identity = None
        self.received = None
        self.last_stamp = None
        self.started_wall = None
        self.generation = 0

    def active(self, now):
        if self.identity and (self.received is None or now - self.received > self.timeout):
            self.identity = None
            self.generation += 1
        return self.identity

    def receive(self, data, now, wall_now):
        self.active(now)
        if not isinstance(data, dict) or not isinstance(data.get('enabled'), bool):
            return False
        stamp = data.get('stamp_unix')
        if (not finite(stamp) or not -0.05 <= wall_now - stamp <= self.timeout
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            return False
        identity = None
        if data['enabled']:
            mission, session = data.get('mission_id'), data.get('session_id')
            if not isinstance(mission, str) or not mission or not isinstance(session, str) or not session:
                return False
            identity = (mission, session)
        if identity != self.identity:
            self.identity = identity
            self.generation += 1
            self.started_wall = wall_now if identity else None
        self.received, self.last_stamp = now, stamp
        return True


def resolve_target_classes(names, aliases=('wolf', 'lupo'), class_id=-1):
    names = dict(enumerate(names)) if isinstance(names, (tuple, list)) else names
    if not isinstance(names, dict) or not names:
        raise ValueError('Model has no class metadata')
    names = {int(k): str(v) for k, v in names.items()}
    if class_id >= 0:
        if class_id not in names:
            raise ValueError(f'Target class ID {class_id} absent from model classes {names}')
        return [class_id]
    wanted = {str(name).strip().casefold() for name in aliases}
    found = [idx for idx, name in names.items() if name.strip().casefold() in wanted]
    if not found:
        raise ValueError(f'No wolf class matches {sorted(wanted)}; model classes are {names}')
    return sorted(found)


def image_to_bgr(msg):
    """Respect ROS row padding; Ultralytics NumPy input is contiguous BGR uint8."""
    import numpy as np
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    if height <= 0 or width <= 0 or step < width * channels:
        raise ValueError('Invalid image dimensions or stride')
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size != height * step:
        raise ValueError('Image payload does not match height and step')
    pixels = raw.reshape(height, step)[:, :width * channels].reshape(height, width, channels)
    if channels == 1:
        pixels = np.repeat(pixels, 3, axis=2)
    elif msg.encoding == 'rgb8':
        pixels = pixels[:, :, ::-1]
    return np.ascontiguousarray(pixels)
