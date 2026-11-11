"""Bound linear acceleration; a requested stop is always immediate."""
import math

class SpeedRamp:
    def __init__(self, acceleration=0.3, deceleration=0.5):
        if not all(math.isfinite(v) and v > 0 for v in (acceleration, deceleration)):
            raise ValueError('Acceleration bounds must be finite and positive')
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.speed = 0.0
        self.last_time = None

    def update(self, requested, now):
        if not math.isfinite(requested) or not math.isfinite(now) or requested < 0:
            self.speed = 0.0
            self.last_time = None
            return 0.0
        dt = 0.0 if self.last_time is None else max(0.0,min(0.1,now-self.last_time))
        self.last_time = now
        if requested == 0:
            self.speed = 0.0
        elif requested >= self.speed:
            self.speed = min(requested,self.speed+self.acceleration*dt)
        else:
            self.speed = max(requested,self.speed-self.deceleration*dt)
        return round(self.speed,4)
