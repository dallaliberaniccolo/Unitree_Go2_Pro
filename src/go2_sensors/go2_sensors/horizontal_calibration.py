"""Planar magnetometer calibration. No Z calibration or tilt compensation."""
import math

def load_horizontal_calibration(raw):
    if raw.get('kind') != 'horizontal_2d':
        raise ValueError('Expected horizontal_2d calibration')
    off = tuple(float(v) for v in raw['offset_xy_ut'])
    matrix = tuple(tuple(float(v) for v in row) for row in raw['matrix_xy'])
    ref = float(raw['reference_horizontal_ut'])
    spread = float(raw['radial_p10_p90_percent'])
    correction = float(raw['true_north_correction_deg'])
    axes = tuple(raw['mag_axes'])
    if len(off) != 2 or len(matrix) != 2 or any(len(row) != 2 for row in matrix):
        raise ValueError('Invalid horizontal calibration dimensions')
    values = off + matrix[0] + matrix[1] + (ref, spread, correction)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite horizontal calibration')
    a, b = matrix[0]
    c, d = matrix[1]
    if a <= 0 or d <= 0 or a*d-b*c <= 1e-9 or abs(b-c) > 1e-6:
        raise ValueError('Horizontal correction must be symmetric positive definite')
    if not 5 <= ref <= 90 or spread < 0 or not -180 <= correction < 180:
        raise ValueError('Invalid field reference, spread or heading correction')
    if axes != (1, -2, -3) or raw.get('heading_reference') != 'geographic_north_iphone':
        raise ValueError('Calibration axis map or north reference mismatch')
    return dict(offset=off, matrix=matrix, reference=ref, spread=spread,
                correction=correction, axes=axes)

def calibrated_horizontal_field(raw, calibration):
    if len(raw) != 3 or not all(math.isfinite(v) for v in raw):
        raise ValueError('Invalid raw magnetic field')
    x = float(raw[0]) - calibration['offset'][0]
    y = float(raw[1]) - calibration['offset'][1]
    m = calibration['matrix']
    return (m[0][0]*x + m[0][1]*y, m[1][0]*x + m[1][1]*y, float(raw[2]))
