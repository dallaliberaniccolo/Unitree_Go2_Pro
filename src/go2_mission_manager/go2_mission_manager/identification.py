"""Mission-owned identification gate; clocks are injected for deterministic tests."""
import math
import uuid


SEARCH_STATES = frozenset((
    'WAIT_LOCALIZATION', 'NAVIGATING_ACTIVE', 'NAVIGATING_DRY_RUN',
    'GPS_GRACE_HOLD', 'ARRIVED_WAIT_IDENTIFICATION',
))


def number(value):
    return (isinstance(value, (float, int)) and not isinstance(value, bool)
            and math.isfinite(value))


class IdentificationGate:
    def __init__(self, initial_radius=50.0, early_rate=50.0, late_rate=40.0,
                 breakpoint_sec=300.0, result_max_age=2.0):
        values = (initial_radius, early_rate, late_rate, breakpoint_sec, result_max_age)
        if not all(number(v) and v > 0 for v in values):
            raise ValueError('Identification radius, rates and timeouts must be positive')
        self.initial_radius = initial_radius
        self.early_rate = early_rate
        self.late_rate = late_rate
        self.breakpoint_sec = breakpoint_sec
        self.result_max_age = result_max_age
        self.clear()

    def clear(self):
        self.mission_id = ''
        self.received_at = None
        self.entered_radius = False
        self.deactivate()

    def new_goal(self, now):
        self.clear()
        self.received_at = now
        self.mission_id = str(uuid.uuid4())

    def age(self, now):
        return None if self.received_at is None else max(0.0, now - self.received_at)

    def radius(self, now):
        age = self.age(now)
        if age is None:
            return self.initial_radius
        return (self.initial_radius
                + self.early_rate * min(age, self.breakpoint_sec) / 60.0
                + self.late_rate * max(0.0, age - self.breakpoint_sec) / 60.0)

    def deactivate(self):
        self.enabled = False
        self.session_id = ''
        self.enabled_wall = None
        self.last_frame_stamp = None

    def update(self, state, distance, distance_valid, now, wall_now):
        if not self.mission_id or state not in SEARCH_STATES:
            self.deactivate()
            return False
        if (state == 'ARRIVED_WAIT_IDENTIFICATION'
                or (distance_valid and number(distance) and 0 <= distance <= self.radius(now))):
            self.entered_radius = True
        if self.entered_radius and not self.enabled:
            self.enabled = True
            self.session_id = str(uuid.uuid4())
            self.enabled_wall = wall_now
            self.last_frame_stamp = None
        return self.enabled

    def control(self, wall_now):
        return dict(mission_id=self.mission_id, session_id=self.session_id,
                    enabled=self.enabled, stamp_unix=wall_now)

    def accept_result(self, result, wall_now):
        """Accept only fresh, ordered frames from this exact activation session."""
        if (not isinstance(result, dict) or not self.enabled
                or result.get('mission_id') != self.mission_id
                or result.get('session_id') != self.session_id
                or result.get('valid') is not True
                or not isinstance(result.get('detected'), bool)):
            return False
        frame = result.get('frame_stamp_unix')
        stamp = result.get('stamp_unix')
        if not number(frame) or not number(stamp):
            return False
        if (frame < self.enabled_wall or stamp < frame
                or not -0.05 <= wall_now - stamp <= self.result_max_age
                or not -0.05 <= wall_now - frame <= self.result_max_age
                or (self.last_frame_stamp is not None and frame <= self.last_frame_stamp)):
            return False
        self.last_frame_stamp = frame
        return True
