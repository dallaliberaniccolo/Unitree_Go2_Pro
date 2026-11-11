"""ROS callbacks stay responsive while one worker performs bounded-rate inference."""
import json
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .backend import YoloBackend
from .core import ControlLease, DetectionWindow, image_to_bgr
from .recording import SampleRecorder


class AnimalDetector(Node):
    def __init__(self, backend_factory=YoloBackend, recorder_factory=SampleRecorder):
        super().__init__('go2_animal_detector')
        defaults = dict(
            model_path='/ws/models/wolf/wolf_detector.pt', image_topic='/go2/camera/image_raw',
            inference_rate_hz=1.5, image_size=448, inference_threads=2,
            target_class_names=['wolf', 'lupo'], target_class_id=-1,
            confidence_threshold=0.2, nms_iou=0.6, max_detections=300,
            confirmation_frames=2, confirmation_window=0,
            confirmation_max_age_sec=3.0, max_frame_age_sec=2.0, control_timeout_sec=1.5,
            record_directory='', record_source='')
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.config = {key: self.get_parameter(key).value for key in defaults}
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Detector expects wall-clock camera stamps; use_sim_time must be false')
        cfg = self.config
        if (any(not math.isfinite(cfg[k]) or cfg[k] <= 0 for k in
                ('inference_rate_hz', 'max_frame_age_sec', 'control_timeout_sec'))
                or cfg['image_size'] < 32 or cfg['image_size'] % 32
                or cfg['inference_threads'] < 1 or cfg['target_class_id'] < -1
                or not math.isfinite(cfg['nms_iou']) or not 0 < cfg['nms_iou'] <= 1
                or cfg['max_detections'] < 1
                or not cfg['image_topic'] or not cfg['model_path']):
            raise ValueError('Invalid detector parameters')
        self.window = DetectionWindow(cfg['confidence_threshold'], cfg['confirmation_frames'],
                                      cfg['confirmation_window'], cfg['confirmation_max_age_sec'])
        self.lease = ControlLease(cfg['control_timeout_sec'])
        self.lock = threading.Lock()
        self.halt = threading.Event()
        self.frame = None
        self.last_image_stamp = None
        self.last_image_received = None
        self.pending_result = None
        self.last_result = None
        self.model_ready = False
        self.warming_up = False
        self.warmup_ms = None
        self.model_error = ''
        self.inference_error = ''
        self.inference_ms = None
        self.completions = deque(maxlen=20)
        self.recording_error = ''
        self.recorder = (recorder_factory(cfg['record_directory'], dict(cfg, mode='ros'))
                         if cfg['record_directory'] else None)
        control_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.result_pub = self.create_publisher(String, '/identification/result', control_qos)
        self.detected_pub = self.create_publisher(Bool, '/identification/detected', control_qos)
        self.status_pub = self.create_publisher(String, '/identification/status', control_qos)
        self.create_subscription(String, '/identification/control', self.on_control, control_qos)
        self.create_subscription(Image, cfg['image_topic'], self.on_image, image_qos)
        self.create_timer(0.05, self.publish_result)
        self.create_timer(0.5, self.publish_status)
        self.worker = threading.Thread(target=self._work, args=(backend_factory,), daemon=False)
        self.worker.start()

    def _clear_frames(self):
        self.frame = self.pending_result = self.last_result = None
        self.last_image_stamp = self.last_image_received = None
        self.completions.clear()
        self.inference_error = ''

    def on_control(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            previous = self.lease.generation
            self.lease.receive(data, time.monotonic(), time.time())
            if self.lease.generation != previous:
                self._clear_frames()

    def on_image(self, msg):
        now, wall = time.monotonic(), time.time()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if not self.model_ready or not self.lease.active(now):
                return
            if (stamp <= 0 or stamp < self.lease.started_wall
                    or not -0.05 <= wall - stamp <= self.config['max_frame_age_sec']
                    or (self.last_image_stamp is not None and stamp <= self.last_image_stamp)):
                return
            self.last_image_stamp, self.last_image_received = stamp, now
            self.frame = (msg, stamp, self.lease.generation)  # overwrite, never queue

    def _work(self, backend_factory):
        try:
            backend = backend_factory(self.config)
            warmup = getattr(backend, 'warmup', None)
            if warmup is not None:
                with self.lock:
                    self.warming_up = True
                started = time.monotonic()
                warmup()
                with self.lock:
                    self.warmup_ms = (time.monotonic() - started) * 1000.0
        except Exception as exc:
            with self.lock:
                self.warming_up = False
                self.model_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(f'Model unavailable: {self.model_error}')
            if self.recorder:
                self.recorder.summary('error', self.model_error)
            return
        with self.lock:
            self.warming_up = False
            if self.halt.is_set():
                return
            self.model_ready = True
        generation = None
        next_due = 0.0
        period = 1.0 / self.config['inference_rate_hz']
        while not self.halt.is_set():
            now = time.monotonic()
            if now < next_due:
                self.halt.wait(min(0.02, next_due - now))
                continue
            with self.lock:
                identity = self.lease.active(now)
                item, self.frame = self.frame, None
                current_generation = self.lease.generation
            if not identity or item is None or item[2] != current_generation:
                self.halt.wait(0.02)
                continue
            msg, frame_stamp, item_generation = item
            if generation != item_generation:
                self.window.clear()
                generation = item_generation
            if time.time() - frame_stamp > self.config['max_frame_age_sec']:
                self.window.clear()
                continue
            started = time.monotonic()
            next_due = started + period
            bgr, boxes = None, []
            try:
                bgr = image_to_bgr(msg)
                if hasattr(backend, 'predict_details'):
                    prediction = backend.predict_details(bgr)
                    score, boxes = prediction['confidence'], prediction['boxes']
                else:
                    score = backend.predict(bgr)
                error = ''
            except Exception as exc:
                score = None
                error = f'{type(exc).__name__}: {exc}'
            finished, wall = time.monotonic(), time.time()
            result = dict(
                mission_id=identity[0], session_id=identity[1], valid=False,
                confidence=score if score is not None and math.isfinite(score) else None,
                frame_stamp_unix=frame_stamp, stamp_unix=wall,
                inference_ms=(finished - started) * 1000.0, detected=None,
                frame_positive=(score >= self.config['confidence_threshold']
                                if score is not None and math.isfinite(score) else None))
            with self.lock:
                if (not self.lease.active(finished) or item_generation != self.lease.generation
                        or self.halt.is_set()):
                    result['error'] = 'Discarded: session changed, lease expired or node stopped'
                elif error or not -0.05 <= wall - frame_stamp <= self.config['max_frame_age_sec']:
                    self.inference_ms = result['inference_ms']
                    self.window.clear()
                    self.last_result = self.pending_result = None
                    self.inference_error = error or 'Inference result exceeded max_frame_age_sec'
                    result['error'] = self.inference_error
                else:
                    self.inference_ms = result['inference_ms']
                    confirmation = self.window.add(frame_stamp, score, now=max(wall, frame_stamp))
                    if confirmation is None:
                        self.inference_error = 'Invalid confidence or repeated frame'
                        self.window.clear()
                        self.last_result = self.pending_result = None
                        result['error'] = self.inference_error
                    else:
                        self.inference_error = ''
                        self.completions.append(finished)
                        result.update(valid=True, error='', **confirmation)
                        self.last_result = result
                        # Preserve the positive event until the ROS executor sends it.
                        if not self.pending_result or not self.pending_result['detected']:
                            self.pending_result = result
            # Disk work never holds the ROS control lock. Recording failures are visible
            # but cannot suppress a valid stop event or turn a model error into false.
            if self.recorder:
                try:
                    self.recorder.save(bgr, result, boxes)
                except Exception as exc:
                    with self.lock:
                        self.recording_error = f'{type(exc).__name__}: {exc}'
                    self.get_logger().error('Sample recording failed: ' + self.recording_error)
        if self.recorder:
            try:
                self.recorder.summary('error' if self.recording_error else 'stopped',
                                      self.recording_error)
            except Exception as exc:
                self.get_logger().error('Cannot finalize sample report: ' + str(exc))

    def publish_result(self):
        with self.lock:
            identity = self.lease.active(time.monotonic())
            result, self.pending_result = self.pending_result, None
            if (not result or identity != (result['mission_id'], result['session_id'])
                    or time.time() - result['frame_stamp_unix'] > self.config['max_frame_age_sec']):
                return
        self.result_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        self.detected_pub.publish(Bool(data=result['detected']))

    def publish_status(self):
        now, wall = time.monotonic(), time.time()
        with self.lock:
            identity = self.lease.active(now)
            valid = bool(identity and self.last_result and not self.inference_error
                         and (self.last_result['mission_id'], self.last_result['session_id']) == identity
                         and -0.05 <= wall - self.last_result['frame_stamp_unix']
                         <= self.config['max_frame_age_sec'])
            if self.model_error:
                state = 'MODEL_ERROR'
            elif self.warming_up:
                state = 'WARMING_MODEL'
            elif not self.model_ready:
                state = 'LOADING_MODEL'
            elif not identity:
                state = 'DISABLED'
            elif self.inference_error:
                state = 'INFERENCE_ERROR'
            elif self.last_image_received is None:
                state = 'WAITING_FOR_IMAGE'
            elif now - self.last_image_received > self.config['max_frame_age_sec']:
                state = 'CAMERA_STALE'
            else:
                state = 'RUNNING' if valid else 'WAITING_FOR_RESULT'
            fps = 0.0
            if len(self.completions) >= 2 and now - self.completions[-1] <= 2.0:
                fps = (len(self.completions) - 1) / (self.completions[-1] - self.completions[0])
            status = dict(
                state=state, enabled=bool(identity), valid=valid,
                mission_id=identity[0] if identity else '',
                session_id=identity[1] if identity else '',
                detected=self.last_result['detected'] if valid else None,
                model_ready=self.model_ready, error=self.model_error or self.inference_error,
                recording_directory=str(self.recorder.path) if self.recorder else '',
                recording_error=self.recording_error,
                warmup_ms=self.warmup_ms,
                inference_ms=self.inference_ms, effective_fps=round(fps, 2),
                requested_fps=self.config['inference_rate_hz'], stamp_unix=wall)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def close(self):
        self.halt.set()
        self.worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AnimalDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
