import json
from pathlib import Path
from ultralytics import YOLO
BASE = Path(__file__).resolve().parent
cfg = json.loads((BASE / 'inference_config.json').read_text())
model = YOLO(str(BASE / cfg['model']))
def detect_wolf(frame):
    result = model.predict(frame, imgsz=cfg['imgsz'], conf=cfg['conf'],
                           iou=cfg['nms_iou'], rect=cfg['rect'], max_det=cfg['max_det'],
                           verbose=False)[0]
    boxes = result.boxes
    return boxes[boxes.cls == cfg['wolf_id']]
