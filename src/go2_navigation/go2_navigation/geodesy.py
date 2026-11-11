"""Short-range WGS84 great-circle geometry; no path planning or obstacle avoidance."""
import math
R = 6371008.8

def valid_coordinates(lat, lon):
    return all(math.isfinite(v) for v in (lat, lon)) and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0

def distance_and_bearing(lat_a, lon_a, lat_b, lon_b):
    if not (valid_coordinates(lat_a, lon_a) and valid_coordinates(lat_b, lon_b)):
        raise ValueError('Invalid WGS84 coordinates')
    la, lb = math.radians(lat_a), math.radians(lat_b)
    dlat, dlon = lb - la, math.radians(lon_b - lon_a)
    h = math.sin(dlat / 2) ** 2 + math.cos(la) * math.cos(lb) * math.sin(dlon / 2) ** 2
    dist = 2 * R * math.asin(min(1.0, math.sqrt(max(0.0, h))))
    y = math.sin(dlon) * math.cos(lb)
    x = math.cos(la) * math.sin(lb) - math.sin(la) * math.cos(lb) * math.cos(dlon)
    bear = (math.degrees(math.atan2(y, x)) + 360) % 360
    return dist, bear

def heading_error_deg(target, current):
    if not (math.isfinite(target) and math.isfinite(current)):
        raise ValueError('non finite heading')
    return (target - current + 180.0) % 360.0 - 180.0

def gps_quality_ok(gga_quality, satellites_used, gga_age_s, min_satellites=7, max_age_s=2.0):
    """Require a valid CURRENT GGA fix, not stale satellite count from old messages."""
    return (isinstance(gga_quality, int) and not isinstance(gga_quality, bool)
            and gga_quality > 0
            and isinstance(satellites_used, int) and not isinstance(satellites_used, bool)
            and satellites_used >= min_satellites
            and isinstance(gga_age_s, (int, float)) and math.isfinite(gga_age_s)
            and 0 <= gga_age_s <= max_age_s)
