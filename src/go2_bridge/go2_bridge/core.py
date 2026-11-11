"""Transport-independent motion gate. Caller serializes access with one lock.

Timeouts use monotonic time; ROS source stamps are checked separately on input.
No network or ROS imports: these rules can be tested without a robot.
"""
from dataclasses import dataclass
import math
import uuid


def remaining_ttl(sent, requested_ttl, wall_now, max_ttl, max_future_skew):
    """Convert a bounded source-time validity into a local monotonic lease."""
    if not all(math.isfinite(v) for v in (sent, requested_ttl, wall_now, max_ttl, max_future_skew)):
        return None
    age = wall_now - sent
    ttl = min(requested_ttl, max_ttl) - max(age, 0.0)
    if sent <= 0 or age < -max_future_skew or ttl <= 0:
        return None
    return ttl


@dataclass(frozen=True)
class Limits:
    permit_timeout: float = 0.6
    command_timeout: float = 0.3
    vx: float = 2.0
    vy: float = 2.0
    wz: float = 0.30

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('All limits must be finite and positive')


class MotionGate:
    def __init__(self, limits=Limits(), motion_enabled=False):
        self.limits = limits
        self.motion_enabled = motion_enabled
        self.connected = False
        self.ready = False
        self.stop_latched = False
        self.reason = 'disconnected'
        self.epoch = ''
        self._clear()

    def _clear(self):
        self.epoch = str(uuid.uuid4())
        self.run_id = ''
        self.permit_until = -math.inf
        self.command_until = -math.inf
        self.permit_sequence = -1
        self.command_sequence = -1
        self.velocity = None

    def trip(self, reason):
        if not self.stop_latched:
            self._clear()
        self.stop_latched = True
        self.reason = reason

    def update_robot(self, connected, ready):
        connected, ready = bool(connected), bool(connected and ready)
        changed = self.connected != connected
        if self.run_id and (not connected or not ready):
            self.trip('connection_lost' if not connected else 'robot_not_ready')
        elif changed:
            self._clear()
        self.connected, self.ready = connected, ready

    def expire(self, now):
        if self.run_id and not self.stop_latched:
            if now >= self.permit_until:
                self.trip('permit_expired')
            elif now >= self.command_until:
                self.trip('command_expired')

    def permit(self, epoch, run_id, sequence, enabled, ttl, now):
        self.expire(now)
        if epoch != self.epoch or not run_id:
            return False
        if not enabled:
            if self.run_id == run_id:
                self.trip('permit_revoked')
                return True
            return False
        if (not self.motion_enabled or not self.connected or not self.ready
                or self.stop_latched or not math.isfinite(ttl) or ttl <= 0):
            return False
        if self.run_id and self.run_id != run_id:
            return False
        if sequence <= self.permit_sequence:
            return False
        if not self.run_id:
            self.run_id = run_id
            self.velocity = None
            self.command_sequence = -1
            # A new grant needs a NEW command within the command timeout.
            self.command_until = now + self.limits.command_timeout
        self.permit_sequence = sequence
        self.permit_until = now + min(ttl, self.limits.permit_timeout)
        return True

    def command(self, epoch, run_id, sequence, velocity, ttl, now):
        self.expire(now)
        if (self.stop_latched or not self.connected or not self.ready
                or not self.run_id or epoch != self.epoch or run_id != self.run_id
                or sequence <= self.command_sequence):
            return False
        if (len(velocity) != 3 or not all(math.isfinite(v) for v in velocity)
                or not math.isfinite(ttl) or ttl <= 0):
            self.trip('invalid_command')
            return False
        # Reject out-of-range commands instead of silently changing their shape.
        if any(abs(v) > bound for v, bound in zip(
                velocity, (self.limits.vx, self.limits.vy, self.limits.wz))):
            self.trip('velocity_limit')
            return False
        self.velocity = tuple(velocity)
        self.command_sequence = sequence
        self.command_until = now + min(ttl, self.limits.command_timeout)
        return True

    def output(self, now):
        self.expire(now)
        if (self.motion_enabled and self.connected and self.ready
                and self.run_id and not self.stop_latched):
            return self.velocity
        return None

    def reset_stop(self):
        if not self.stop_latched or not self.connected or not self.ready:
            return False
        self._clear()
        self.stop_latched = False
        self.reason = 'waiting_for_new_permit'
        return True

    def snapshot(self, now):
        self.expire(now)
        if not self.connected:
            state = 'DISCONNECTED'
        elif self.stop_latched:
            state = 'STOPPING'
        elif not self.ready:
            state = 'NOT_READY'
        elif not self.motion_enabled:
            state = 'READ_ONLY'
        elif self.run_id:
            state = 'ENABLED'
        else:
            state = 'READY'
        reason = self.reason if self.stop_latched else {
            'DISCONNECTED': 'disconnected', 'NOT_READY': 'telemetry_or_mode_not_validated',
            'READ_ONLY': 'motion_enabled_false', 'READY': 'waiting_for_permit',
            'ENABLED': 'permit_valid',
        }.get(state, '')
        return dict(state=state, reason=reason, bridge_epoch=self.epoch,
                    run_id=self.run_id, connected=self.connected,
                    robot_ready=self.ready, motion_enabled=self.motion_enabled,
                    permit_valid=bool(self.run_id and not self.stop_latched),
                    stop_requested=self.stop_latched)
