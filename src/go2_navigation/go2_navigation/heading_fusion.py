"""North from external magnetometer; bounded relative-yaw propagation on magnetic anomalies."""
import math


def angle_error(target, current):
    return (target-current+180.0)%360.0-180.0


class HeadingFusion:
    def __init__(self, max_hold_sec=10.0, innovation_limit_deg=12.0,
                 reacquire_sec=1.0, correction_rate_dps=2.0, yaw_ccw=True):
        values=(max_hold_sec,innovation_limit_deg,reacquire_sec,correction_rate_dps)
        if not all(math.isfinite(v) and v>0 for v in values) or innovation_limit_deg>=90:
            raise ValueError('Invalid heading fusion bounds')
        self.max_hold=max_hold_sec
        self.innovation_limit=innovation_limit_deg
        self.reacquire=reacquire_sec
        self.correction_rate=correction_rate_dps
        self.sign=-1.0 if yaw_ccw else 1.0
        self.heading=None
        self.last_yaw=None
        self.last_time=None
        self.last_mag_time=None
        self.last_mag_id=None
        self.trusted=False
        self.recovering=False
        self.pending=[]
        self.last_reason='waiting_for_magnetic_north'

    def reset_reference(self, reason):
        self.heading=None
        self.last_yaw=None
        self.last_mag_time=None
        self.trusted=False
        self.recovering=False
        self.pending=[]
        self.last_reason=reason

    def _append_consistent(self, now, value):
        self.pending.append((now, value))
        self.pending=[p for p in self.pending if now-p[0]<=self.reacquire+0.5]
        if len(self.pending)<5 or now-self.pending[0][0]<self.reacquire:
            return False
        x=sum(math.cos(math.radians(v)) for t,v in self.pending)
        y=sum(math.sin(math.radians(v)) for t,v in self.pending)
        average=math.degrees(math.atan2(y,x))%360.0
        return all(abs(angle_error(v,average))<=5.0 for t,v in self.pending)

    def update(self, now, yaw_deg, yaw_fresh, mag_deg, mag_valid, mag_sample_id):
        if not math.isfinite(now):
            self.reset_reference('invalid_time')
            return dict(valid=False,heading_deg=None,source='none',mode='WAITING',
                        reason=self.last_reason,mag_age_sec=None,innovation_deg=None,mag_accepted=False)
        dt=0.0 if self.last_time is None else now-self.last_time
        self.last_time=now
        mag_valid=bool(mag_valid and isinstance(mag_deg,(float,int)) and not isinstance(mag_deg,bool)
                       and math.isfinite(mag_deg) and 0<=mag_deg<360)
        yaw_fresh=bool(yaw_fresh and isinstance(yaw_deg,(float,int)) and math.isfinite(yaw_deg))
        if not yaw_fresh:
            self.reset_reference('relative_imu_not_fresh')
            return dict(valid=mag_valid,heading_deg=float(mag_deg) if mag_valid else None,
                        source='external_icm20948_magnetometer' if mag_valid else 'none',
                        mode='COMPASS_ONLY' if mag_valid else 'WAITING',reason=self.last_reason,
                        mag_age_sec=None,innovation_deg=None,mag_accepted=mag_valid)
        if dt<0 or dt>0.5:
            self.reset_reference('relative_imu_update_gap')
        if self.last_yaw is not None:
            delta=angle_error(yaw_deg,self.last_yaw)
            if abs(delta)>5.0+180.0*max(0.0,dt):
                self.reset_reference('relative_yaw_discontinuity')
            elif self.heading is not None:
                self.heading=(self.heading+self.sign*delta)%360.0
        self.last_yaw=float(yaw_deg)
        innovation=None
        new_mag=mag_sample_id is not None and mag_sample_id!=self.last_mag_id
        accepted=False
        if not mag_valid:
            self.trusted=False
            self.recovering=False
            if self.pending and now-self.pending[-1][0]>0.3:
                self.pending=[]
            self.last_reason='magnetic_measurement_invalid'
        elif new_mag:
            self.last_mag_id=mag_sample_id
            if self.heading is None:
                anchor=(mag_deg-self.sign*yaw_deg)%360.0
                if self._append_consistent(now,anchor):
                    x=sum(math.cos(math.radians(v)) for t,v in self.pending)
                    y=sum(math.sin(math.radians(v)) for t,v in self.pending)
                    average=math.degrees(math.atan2(y,x))%360.0
                    self.heading=(average+self.sign*yaw_deg)%360.0
                    self.last_mag_time=now
                    self.trusted=True
                    self.recovering=False
                    self.pending=[]
                    self.last_reason='magnetic_north_anchored'
                    accepted=True
            else:
                innovation=angle_error(mag_deg,self.heading)
                outlier=abs(innovation)>self.innovation_limit
                if self.recovering:
                    if not self._append_consistent(now,innovation):
                        self.recovering=False
                        self.trusted=False
                        self.pending=[(now,innovation)]
                        self.last_reason='magnetic_direction_rejected'
                    if self.recovering:
                        step=self.correction_rate*min(max(dt,0.0),0.2)
                        self.heading=(self.heading+max(-step,min(step,innovation)))%360.0
                        self.last_mag_time=now
                        accepted=True
                        if abs(innovation)<=self.innovation_limit:
                            self.recovering=False
                            self.pending=[]
                            self.last_reason='magnetic_correction'
                        else:
                            self.last_reason='magnetic_reacquiring'
                elif outlier:
                    if self._append_consistent(now,innovation):
                        self.recovering=True
                        self.trusted=True
                        step=self.correction_rate*min(max(dt,0.0),0.2)
                        self.heading=(self.heading+max(-step,min(step,innovation)))%360.0
                        self.last_mag_time=now
                        accepted=True
                        self.last_reason='magnetic_reacquiring'
                    else:
                        self.trusted=False
                        self.last_reason='magnetic_direction_rejected'
                else:
                    if not self.trusted:
                        if self._append_consistent(now,innovation):
                            self.trusted=True
                        else:
                            self.last_reason='magnetic_reacquiring'
                    if self.trusted:
                        step=self.correction_rate*min(max(dt,0.0),0.2)
                        self.heading=(self.heading+max(-step,min(step,innovation)))%360.0
                        self.last_mag_time=now
                        self.pending=[]
                        self.last_reason='magnetic_correction'
                        accepted=True
        age=None if self.last_mag_time is None else max(0.0,now-self.last_mag_time)
        valid=self.heading is not None and age is not None and age<=self.max_hold
        tracking=valid and self.trusted and mag_valid and age<=0.3
        mode='FUSED' if tracking else 'MAGNETIC_HOLD' if valid else 'WAITING' if self.heading is None else 'HOLD_EXPIRED'
        source=('external_mag_anchored_unitree_imu' if tracking else 'unitree_imu_magnetic_hold') if valid else 'none'
        reason='magnetic_hold_expired' if mode=='HOLD_EXPIRED' else self.last_reason
        return dict(valid=valid,heading_deg=self.heading if valid else None,source=source,mode=mode,
                    reason=reason,mag_age_sec=age,innovation_deg=innovation,mag_accepted=accepted)
