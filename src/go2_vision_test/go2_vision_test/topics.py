ROOT = '/vision_test'

MANAGER_TOPICS = (
    '/mission/goal', '/navigation/status', '/go2/bridge_state',
    '/go2/control_permit', '/go2/cmd_vel', '/mission/status',
    '/identification/enable', '/identification/control', '/identification/result',
    '/identification/status', '/mission/wolf_detected', '/go2/stop',
    '/go2/reset_stop', '/mission/start', '/mission/pause', '/mission/cancel',
)
DETECTOR_TOPICS = (
    '/go2/camera/image_raw', '/identification/control', '/identification/result',
    '/identification/status', '/identification/detected',
)
MANAGER_REMAPS = [(name, ROOT + name) for name in MANAGER_TOPICS]
DETECTOR_REMAPS = [(name, ROOT + name) for name in DETECTOR_TOPICS]
