"""Estimate gravity direction from external gyro and reliable accelerometer samples."""
import math

class GravityFilter:
    def __init__(self, tau_sec=0.5, max_coast_sec=2.0, accel_smoothing_sec=0.2):
        if not all(math.isfinite(v) and v > 0 for v in (tau_sec, max_coast_sec)):
            raise ValueError('Invalid gravity filter timing')
        if not math.isfinite(accel_smoothing_sec) or accel_smoothing_sec < 0:
            raise ValueError('Invalid acceleration smoothing time')
        self.accel_tau = accel_smoothing_sec
        self.acc_lp = None
        self.tau = tau_sec
        self.max_coast = max_coast_sec
        self.up = None
        self.last_time = None
        self.last_accel = None
        self.recovery_required = False
        self.recovery_samples = []

    def update(self, acc, gyro_rad, now):
        out = dict(valid=False, tilt_deg=None, accel_used=False, accel_age_sec=None)
        if not (math.isfinite(now) and len(acc) == len(gyro_rad) == 3
                and all(math.isfinite(v) for v in (*acc, *gyro_rad))):
            return out
        dt = 0.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        if dt < 0 or dt > 0.5:
            self.up = None
            self.last_accel = None
            dt = 0.0
        if max(abs(v) for v in gyro_rad) > math.radians(400):
            self.up = None
            self.last_accel = None
            self.acc_lp = None
            self.recovery_required = True
            self.recovery_samples = []
            return out
        def normalize(v):
            length = math.sqrt(sum(x*x for x in v))
            return tuple(x/length for x in v) if length > 1e-9 else None
        if self.up is not None:
            ux, uy, uz = self.up
            gx, gy, gz = gyro_rad
            # Passive body-frame rotation of the world-up vector: -omega cross up.
            self.up = normalize((ux + dt*(gz*uy-gy*uz),
                                 uy + dt*(gx*uz-gz*ux),
                                 uz + dt*(gy*ux-gx*uy)))
        if self.up is not None and self.last_accel is not None and now-self.last_accel > self.max_coast:
            self.up = None
            self.last_accel = None
            self.recovery_required = True
            self.recovery_samples = []
            self.acc_lp = None
        # Filter the acceleration vector, never Euler angles. Transport its
        # previous value into the current body frame before averaging.
        if dt == 0.0:
            self.acc_lp = None
        if self.acc_lp is not None:
            x,y,z = self.acc_lp
            gx,gy,gz = gyro_rad
            self.acc_lp = (x+dt*(gz*y-gy*z),y+dt*(gx*z-gz*x),z+dt*(gy*x-gx*y))
        raw_acc = tuple(acc)
        raw_magnitude = math.sqrt(sum(v*v for v in acc))
        if 0.5 <= raw_magnitude <= 1.5:
            alpha = 1.0 if self.acc_lp is None or self.accel_tau == 0 else dt/(self.accel_tau+dt)
            self.acc_lp = tuple(acc) if self.acc_lp is None else tuple((1-alpha)*a+alpha*b for a,b in zip(self.acc_lp,acc))
        # An outlier must not count as a new gravity observation.
        acc = self.acc_lp if self.acc_lp is not None and 0.5 <= raw_magnitude <= 1.5 else (0.,0.,0.)
        magnitude = math.sqrt(sum(v*v for v in acc))
        if 0.85 <= magnitude <= 1.15:
            measured = normalize(acc)
            if self.up is None and self.recovery_required:
                if max(abs(v) for v in gyro_rad) <= math.radians(10):
                    self.recovery_samples.append((now, normalize(raw_acc)))
                    self.recovery_samples = [(t,v) for t,v in self.recovery_samples if now-t <= 1.0]
                    values = self.recovery_samples
                    if len(values) >= 6 and now-values[0][0] >= 0.5:
                        average = normalize(tuple(sum(v[i] for t,v in values)/len(values) for i in range(3)))
                        consistent = average is not None and all(
                            sum(a*b for a,b in zip(v,average)) >= math.cos(math.radians(8)) for t,v in values)
                        if consistent:
                            self.up = average
                            self.last_accel = now
                            self.recovery_required = False
                            self.recovery_samples = []
                            out['accel_used'] = True
                else:
                    self.recovery_samples = []
            else:
                agreement = 1.0 if self.up is None else sum(a*b for a,b in zip(self.up, measured))
                if self.up is None or agreement >= math.cos(math.radians(20)):
                    alpha = 1.0 if self.up is None else dt/(self.tau+dt)
                    self.up = measured if self.up is None else normalize(tuple(
                        (1-alpha)*a+alpha*b for a,b in zip(self.up, measured)))
                    self.last_accel = now
                    out['accel_used'] = True
        elif self.recovery_required:
            self.recovery_samples = []
        if self.up is not None and self.last_accel is not None:
            age = now-self.last_accel
            out.update(valid=0 <= age <= self.max_coast,
                       tilt_deg=math.degrees(math.acos(max(-1.0,min(1.0,self.up[2])))),
                       accel_age_sec=age)
        return out
