"""Local video -> ROS Image, with current capture stamps and original playback speed."""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .topics import ROOT
from .video import VideoReader


class VideoCamera(Node):
    def __init__(self, reader_factory=VideoReader):
        super().__init__('video_camera')
        defaults = dict(video_path='', image_topic=ROOT + '/go2/camera/image_raw',
                        publish_rate_hz=1.5, playback_speed=1.0, loop=False,
                        source_fps=0.0, wait_for_detector=True,
                        camera_frame='go2_camera_optical_frame')
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.config = {key: self.get_parameter(key).value for key in defaults}
        cfg = self.config
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Video replay stamps use wall time; use_sim_time must be false')
        if (any(not math.isfinite(cfg[k]) or cfg[k] <= 0 for k in ('publish_rate_hz', 'playback_speed'))
                or not math.isfinite(cfg['source_fps']) or cfg['source_fps'] < 0
                or not cfg['camera_frame'] or not cfg['image_topic']):
            raise ValueError('Invalid video replay parameters')
        self.reader = reader_factory(cfg['video_path'], loop=cfg['loop'], source_fps=cfg['source_fps'])
        self.position = 0.0
        self.last_tick = None
        self.was_playing = False
        self.detector_ready = False
        self.detector_received = None
        self.sent = 0
        self.state, self.error = 'WAITING_FOR_DETECTOR', ''
        self.failed = False
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        control_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.image_pub = self.create_publisher(Image, cfg['image_topic'], image_qos)
        self.status_pub = self.create_publisher(String, ROOT + '/video/status', control_qos)
        self.sample_pub = self.create_publisher(String, ROOT + '/video/sample', control_qos)
        self.create_subscription(String, ROOT + '/identification/status', self.on_detector, control_qos)
        self.create_timer(1.0 / cfg['publish_rate_hz'], self.tick)
        self.create_timer(0.5, self.publish_status)

    def on_detector(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = data['stamp_unix']
            fresh = (isinstance(stamp, (float, int)) and not isinstance(stamp, bool)
                     and math.isfinite(stamp) and -0.05 <= time.time() - stamp <= 2.0)
            self.detector_ready = bool(fresh and data.get('model_ready') is True and data.get('enabled') is True)
            self.detector_received = time.monotonic()
        except (TypeError, ValueError, KeyError):
            self.detector_ready = False

    def tick(self):
        now = time.monotonic()
        permitted = (not self.config['wait_for_detector']
                     or (self.detector_ready and self.detector_received is not None
                         and now - self.detector_received <= 2.0))
        if self.failed or self.reader.finished:
            return
        if not permitted:
            self.was_playing = False
            self.last_tick = now
            self.state = 'PAUSED' if self.sent else 'WAITING_FOR_DETECTOR'
            return
        if self.was_playing and self.last_tick is not None:
            self.position += max(0.0, now - self.last_tick) * self.config['playback_speed']
        self.last_tick, self.was_playing = now, True
        try:
            bgr = self.reader.read_at(self.position)
            if self.reader.finished:
                self.state = 'EOF'
                self.get_logger().info('End of video; image publication stopped')
                return
            self.state = 'PLAYING'
            if bgr is None:
                return
            bgr = np.ascontiguousarray(bgr)
            if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
                raise ValueError('Video decoder must return uint8 BGR frames')
            msg = Image()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
            msg.header.frame_id = self.config['camera_frame']
            msg.height, msg.width = bgr.shape[:2]
            msg.encoding, msg.is_bigendian = 'bgr8', 0
            msg.step = msg.width * 3
            msg.data = bgr.tobytes()
            self.image_pub.publish(msg)
            self.sample_pub.publish(String(data=json.dumps(dict(
                frame_stamp_unix=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                source_frame=self.reader.frame_index,
                video_time_sec=self.reader.frame_index / self.reader.fps,
                loop_count=self.reader.cycle), allow_nan=False)))
            self.sent += 1
        except Exception as exc:
            self.state, self.error, self.failed = 'ERROR', f'{type(exc).__name__}: {exc}', True
            self.reader.close()
            self.get_logger().error(self.error)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.state, error=self.error, source_fps=self.reader.fps,
            source_frame_count=self.reader.frame_count,
            source_duration_sec=(self.reader.frame_count / self.reader.fps
                                 if self.reader.frame_count else None),
            output_fps=self.config['publish_rate_hz'], playback_speed=self.config['playback_speed'],
            playback_sec=round(self.position, 3), frame_index=self.reader.frame_index,
            loop_count=self.reader.cycle, published_frames=self.sent, stamp_unix=time.time()), allow_nan=False)))

    def close(self):
        self.reader.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VideoCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
