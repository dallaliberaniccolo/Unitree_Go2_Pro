"""Load a local custom model once; never download a fallback pretrained model."""
import math
import os
from pathlib import Path

from .core import resolve_target_classes


class YoloBackend:
    def __init__(self, config):
        path = Path(config['model_path']).expanduser()
        if not path.exists():
            raise FileNotFoundError(f'Custom wolf model missing: {path}')
        threads = int(config['inference_threads'])
        for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
            os.environ[name] = str(threads)
        import torch
        torch.set_num_threads(threads)
        if torch.get_num_interop_threads() != 1:
            torch.set_num_interop_threads(1)
        from ultralytics import YOLO
        self.model = YOLO(str(path))
        if self.model.task != 'detect':
            raise ValueError(f'Expected an object detection model, got {self.model.task}')
        self.classes = resolve_target_classes(
            self.model.names, config['target_class_names'], config['target_class_id'])
        self.config = config

    def warmup(self):
        """Initialize lazy predictor/fusion using a discarded synthetic image."""
        import numpy as np
        size = self.config['image_size']
        self.predict(np.zeros((size, size, 3), dtype=np.uint8))

    def predict(self, bgr):
        return self.predict_details(bgr)['confidence']

    def predict_details(self, bgr):
        results = self.model.predict(
            source=bgr, imgsz=self.config['image_size'],
            conf=self.config['confidence_threshold'], classes=self.classes,
            iou=self.config['nms_iou'],
            device='cpu', verbose=False, save=False, stream=False,
            rect=False, max_det=self.config['max_detections'])
        if len(results) != 1 or results[0].boxes is None:
            raise ValueError('Detector returned no valid boxes result')
        boxes = results[0].boxes
        confidence = 0.0
        detections = []
        for cls, score, xyxy in zip(boxes.cls.cpu().tolist(), boxes.conf.cpu().tolist(),
                                    boxes.xyxy.cpu().tolist()):
            if not math.isfinite(score) or not 0 <= score <= 1:
                raise ValueError('Invalid confidence returned by model')
            if int(cls) in self.classes:
                confidence = max(confidence, float(score))
                if len(xyxy) != 4 or not all(math.isfinite(v) for v in xyxy):
                    raise ValueError('Invalid bounding box returned by model')
                detections.append(dict(class_id=int(cls), label=str(self.model.names[int(cls)]),
                                       confidence=float(score), xyxy=list(map(float, xyxy))))
        return dict(confidence=confidence, boxes=detections)
