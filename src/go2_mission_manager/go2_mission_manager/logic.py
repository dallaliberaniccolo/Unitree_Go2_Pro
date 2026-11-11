"""Mission state machine and direct open-field waypoint controller.

Readiness failures keep a requested mission in a safe hold.  The mission resumes
automatically when fresh localization, heading and bridge state recover.
"""
import math
import time


def clip(val, low, high):
    return max(low, min(high, val))


class MissionLogic:
    ACTIVE_STATES = ('NAVIGATING_ACTIVE', 'GPS_GRACE_HOLD')
    REQUESTED_STATES = ('WAIT_LOCALIZATION', 'NAVIGATING_DRY_RUN',
                        'NAVIGATING_ACTIVE', 'GPS_GRACE_HOLD')

    def __init__(self, loss_timeout=20.0, arrival_samples=2):
        if not math.isfinite(loss_timeout) or loss_timeout <= 0 or arrival_samples < 1:
            raise ValueError('Bad mission parameters')
        self.loss_timeout = float(loss_timeout)  # retained for config compatibility
        self.arrival_samples_needed = int(arrival_samples)
        self.state = 'IDLE'
        self.reason = 'no_goal'
        self.goal = None
        self.arrival_samples = 0
        self.last_gps_update = -1
        self.aligning_in_place = False
        self.ready_since = None
        self.ever_active = False
        self.arrival_distance_m = None
        self.arrival_stamp = None
        self.wolf_detected = False

    def load_goal(self, goal):
        self.wolf_detected = False
        self.goal = [float(goal[0]), float(goal[1])]
        self.state = 'LOADED'
        self.reason = 'goal_loaded'
        self.arrival_samples = 0
        self.last_gps_update = -1
        self.aligning_in_place = False
        self.ready_since = None
        self.ever_active = False
        self.arrival_distance_m = None
        self.arrival_stamp = None

    def start(self):
        if self.wolf_detected:
            return False, 'Wolf confirmed; cancel or submit a new waypoint to leave this phase'
        if self.goal is None:
            return False, 'No waypoint loaded'
        if self.state in self.REQUESTED_STATES:
            return False, 'Mission already started or waiting for readiness'
        self.state = 'WAIT_LOCALIZATION'
        self.reason = 'waiting_for_readiness'
        self.ready_since = None
        self.arrival_samples = 0
        self.aligning_in_place = True
        self.ever_active = False
        return True, 'Mission queued; waiting for automatic preflight'

    def pause(self, reason='user_paused'):
        self.state, self.reason = 'PAUSED', reason
        self.ready_since = None
        self.aligning_in_place = False

    def cancel(self):
        self.wolf_detected = False
        self.state = 'IDLE'
        self.reason = 'mission_cancelled'
        self.goal = None
        self.ready_since = None
        self.arrival_samples = 0
        self.last_gps_update = -1
        self.aligning_in_place = False
        self.ever_active = False
        self.arrival_distance_m = None
        self.arrival_stamp = None

    def confirm_wolf(self):
        if (self.goal is None or self.wolf_detected
                or self.state not in self.REQUESTED_STATES + ('ARRIVED_WAIT_IDENTIFICATION',)):
            return False
        self.wolf_detected = True
        self.state, self.reason = 'WOLF_DETECTED', 'wolf_confirmed_stop'
        self.ready_since = None
        self.aligning_in_place = False
        return True

    def update(self, gps_ok, nav_ready, bridge_connected, within_goal, gps_updates,
               enable_motion=False, now=None, readiness_stable_sec=1.5,
               bridge_ready=None, distance_m=None):
        t = time.monotonic() if now is None else now
        if self.state not in self.REQUESTED_STATES:
            return

        if bridge_ready is None:
            bridge_ready = bridge_connected
        ready = bool(gps_ok and nav_ready and bridge_connected and bridge_ready)
        if not ready:
            self.ready_since = None
            if self.ever_active:
                self.state = 'GPS_GRACE_HOLD'
                if not bridge_connected:
                    self.reason = 'waiting_for_bridge_reconnect'
                elif not bridge_ready:
                    self.reason = 'waiting_for_bridge_ready_or_stop_reset'
                elif not gps_ok:
                    self.reason = 'localization_invalid_motion_hold'
                else:
                    self.reason = 'sensor_preflight_invalid_motion_hold'
            else:
                self.state = 'WAIT_LOCALIZATION'
                if not bridge_connected:
                    self.reason = 'waiting_for_bridge'
                elif not bridge_ready:
                    self.reason = 'waiting_for_bridge_ready_or_stop_reset'
                elif not gps_ok:
                    self.reason = 'waiting_for_valid_gps'
                else:
                    self.reason = 'waiting_for_valid_imu_heading_and_navigation'
            return

        if self.ready_since is None:
            self.ready_since = t
            self.state = 'GPS_GRACE_HOLD' if self.ever_active else 'WAIT_LOCALIZATION'
            self.reason = 'preflight_stabilizing'
            return
        if t - self.ready_since < readiness_stable_sec:
            self.state = 'GPS_GRACE_HOLD' if self.ever_active else 'WAIT_LOCALIZATION'
            self.reason = 'preflight_stabilizing'
            return

        active_state = 'NAVIGATING_ACTIVE' if enable_motion else 'NAVIGATING_DRY_RUN'
        if self.state in ('WAIT_LOCALIZATION', 'GPS_GRACE_HOLD'):
            self.state = active_state
            self.reason = 'direct_waypoint_active' if enable_motion else 'no_actuators_connected'
            self.ever_active = True
            self.arrival_samples = 0
            self.last_gps_update = -1

        if isinstance(gps_updates, int) and not isinstance(gps_updates, bool):
            if gps_updates > self.last_gps_update:
                self.last_gps_update = gps_updates
                self.arrival_samples = (self.arrival_samples + 1) if within_goal else 0
            if self.arrival_samples >= self.arrival_samples_needed:
                self.state = 'ARRIVED_WAIT_IDENTIFICATION'
                self.reason = 'waypoint_reached'
                if isinstance(distance_m, (int, float)) and math.isfinite(distance_m):
                    self.arrival_distance_m = float(distance_m)
                self.arrival_stamp = time.time()
                self.aligning_in_place = False

    def compute_velocity(self, distance_m, heading_error_deg, cfg):
        if self.state != 'NAVIGATING_ACTIVE' or distance_m is None or heading_error_deg is None:
            return (0.0, 0.0, 0.0)
        if not all(isinstance(v, (int, float)) and math.isfinite(v)
                   for v in (distance_m, heading_error_deg)):
            return (0.0, 0.0, 0.0)
        if distance_m <= cfg['arrival_radius_m']:
            return (0.0, 0.0, 0.0)

        err = float(heading_error_deg)
        abs_err = abs(err)
        if self.aligning_in_place:
            if abs_err <= cfg['align_exit_deg']:
                self.aligning_in_place = False
        elif abs_err >= cfg['turn_in_place_deg']:
            self.aligning_in_place = True

        wz_raw = -cfg['yaw_cmd_sign'] * cfg['kp_yaw'] * math.radians(err)
        if self.aligning_in_place and not cfg['lock_straight_line']:
            wz = clip(wz_raw, -cfg['max_wz'], cfg['max_wz'])
            if 0.0 < abs(wz) < cfg['min_wz']:
                wz = math.copysign(cfg['min_wz'], wz)
            return (0.0, 0.0, round(wz, 4))

        cruise = cfg.get('cruise_speed_mps', 0.0) or cfg['max_vx']
        target_speed = min(cruise, cfg['max_vx'])
        aligned_speed = target_speed * max(0.5, math.cos(math.radians(abs_err)))
        if distance_m < cfg['slow_down_radius_m']:
            span = cfg['slow_down_radius_m'] - cfg['arrival_radius_m']
            progress = clip((distance_m - cfg['arrival_radius_m']) / span, 0.0, 1.0)
            approach_speed = min(cfg['approach_speed_mps'], aligned_speed)
            vx = approach_speed + (aligned_speed - approach_speed) * progress
        else:
            vx = aligned_speed
        vx = clip(vx, cfg['min_vx'], cfg['max_vx'])
        if cfg['lock_straight_line'] or abs_err <= cfg['yaw_deadband_deg']:
            wz = 0.0
        else:
            wz = clip(wz_raw, -cfg['max_wz_walking'], cfg['max_wz_walking'])
        return (round(vx, 4), 0.0, round(wz, 4))
