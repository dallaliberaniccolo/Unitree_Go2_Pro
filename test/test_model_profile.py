import hashlib
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from go2_animal_detector.backend import YoloBackend
from go2_animal_detector.node import AnimalDetector


class ModelProfileTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        self.folder = self.root / 'models/wolf'
        self.profile = json.loads((self.folder / 'inference_config.json').read_text())

    def test_checkpoint_matches_supplied_export_hash(self):
        actual = hashlib.sha256((self.folder / self.profile['model']).read_bytes()).hexdigest()
        self.assertEqual(actual, self.profile['checkpoint_sha256'])

    def test_node_and_yaml_use_exported_inference_settings(self):
        node = AnimalDetector(lambda cfg: NS(predict=lambda image: 0.0))
        self.addCleanup(node.close)
        yaml_cfg = yaml.safe_load((self.root / 'config/go2_waypoint_diretto.yaml').read_text())['/go2_animal_detector']['ros__parameters']
        for cfg in (node.config, yaml_cfg):
            self.assertEqual(cfg['inference_rate_hz'], 1.5)
            self.assertEqual(cfg['confirmation_max_age_sec'], 3.0)
            self.assertEqual(cfg['max_frame_age_sec'], 2.0)
            self.assertEqual(cfg['confirmation_frames'], 2)
            self.assertEqual(cfg['confirmation_window'], 0)
            for runtime, exported in (('image_size', 'imgsz'), ('confidence_threshold', 'conf'),
                                      ('nms_iou', 'nms_iou'), ('max_detections', 'max_det')):
                self.assertEqual(cfg[runtime], self.profile[exported])

    def test_backend_does_not_treat_high_confidence_fox_as_wolf(self):
        tensor = lambda values: NS(cpu=lambda: NS(tolist=lambda: values))
        boxes = NS(cls=tensor([1]), conf=tensor([.99]), xyxy=tensor([[0, 0, 1, 1]]))
        calls = []

        def predict(**kwargs):
            calls.append(kwargs)
            return [NS(boxes=boxes)]

        model = NS(task='detect', names={0: 'wolf', 1: 'fox'}, predict=predict)
        torch = NS(get_num_interop_threads=lambda: 2, set_num_threads=lambda value: None, set_num_interop_threads=lambda value: None)
        cfg = dict(model_path=str(self.folder / self.profile['model']), inference_threads=2,
                   target_class_names=['wolf'], target_class_id=-1, image_size=448,
                   confidence_threshold=.2, nms_iou=.6, max_detections=300)
        with patch.dict(sys.modules, {'torch': torch, 'ultralytics': NS(YOLO=lambda path: model)}), patch.dict(os.environ):
            backend = YoloBackend(cfg)
            self.assertEqual(backend.predict(np.zeros((2, 2, 3), dtype=np.uint8)), 0.0)
            self.assertEqual(calls[-1]['classes'], [0])
            boxes.cls, boxes.conf = tensor([1, 0]), tensor([.99, .7])
            boxes.xyxy = tensor([[0, 0, 1, 1], [0, 0, 2, 2]])
            self.assertEqual(backend.predict(np.zeros((2, 2, 3), dtype=np.uint8)), .7)
            backend.warmup()
            self.assertEqual(calls[-1]['source'].shape, (448, 448, 3))
            self.assertEqual(calls[-1]['source'].max(), 0)
