# Pure math; body coordinates +X forward, +Y left, +Z up.
import math

def apply_axis_map(values, indices):
    if len(values) != 3 or len(indices) != 3 or sorted(abs(i) for i in indices) != [1, 2, 3]:
        raise ValueError('Axis map must be a signed permutation of (1,2,3)')
    return tuple(float(values[abs(i) - 1]) * (1 if i > 0 else -1) for i in indices)

def norm(v):
    return math.sqrt(sum(x * x for x in v))

def dot(a, b):
    return sum(x * y for x, y in zip(a, b))

def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])

def normalize(v):
    n = norm(v)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError('Vector magnitude too small or invalid')
    return tuple(x / n for x in v)

def compass_2d_deg(mag_ut):
    mx, my = float(mag_ut[0]), float(mag_ut[1])
    if math.hypot(mx, my) < 1e-6:
        raise ValueError('Horizontal magnetic field too small')
    return math.degrees(math.atan2(my, mx)) % 360.0

def tilt_compass_deg(accel_g, mag_ut):
    up = normalize(accel_g)
    forward = (1., 0., 0.)
    projection = dot(forward, up)
    forward_horizontal = normalize(tuple(f - projection * u for f, u in zip(forward, up)))
    left_horizontal = normalize(cross(up, forward_horizontal))
    return math.degrees(math.atan2(dot(mag_ut, left_horizontal),
                                   dot(mag_ut, forward_horizontal))) % 360.0

def calibration_from_json(raw):
    off = tuple(float(x) for x in raw['offset_ut'])
    scale = tuple(float(x) for x in raw['scale_axes'])
    ref = float(raw['intensita_riferimento_ut'])
    spread = float(raw.get('variazione_p10_p90_percento', 10.0))
    if (len(off) != 3 or len(scale) != 3 or not all(math.isfinite(x) for x in off + scale)
            or not all(x > 0 for x in scale) or not 10 <= ref <= 90 or not math.isfinite(spread)
            or spread < 0):
        raise ValueError('Invalid calibration numbers')
    return off, scale, ref, spread

def calibrated_field(raw, off, scale):
    return tuple((float(raw[i]) - off[i]) * scale[i] for i in range(3))
