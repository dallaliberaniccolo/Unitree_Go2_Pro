"""Append-only evidence for actual analyzed frames; usable without ROS or a display."""
import csv
import datetime
import hashlib
import html
import json
from pathlib import Path
import uuid


class SampleRecorder:
    FIELDS = ('sample', 'source_frame', 'video_time_sec', 'frame_stamp_unix',
              'stamp_unix', 'confidence', 'frame_positive', 'detected', 'positive_frames',
              'window_frames', 'valid', 'error', 'inference_ms', 'mission_id', 'session_id',
              'image', 'annotated')

    def __init__(self, directory, metadata, cv=None):
        if cv is None:
            import cv2 as cv
        self.cv = cv
        run = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S')
        self.path = Path(directory).expanduser().resolve() / (run + '_' + uuid.uuid4().hex[:8])
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / 'images').mkdir()
        (self.path / 'annotated').mkdir()
        self.count = 0
        self.positives = 0
        self.confirmations = 0
        model = Path(metadata.get('model_path', ''))
        metadata = dict(metadata)
        if model.is_file():
            metadata['model_sha256'] = hashlib.sha256(model.read_bytes()).hexdigest()
        (self.path / 'metadata.json').write_text(
            json.dumps(metadata, indent=2, allow_nan=False) + '\n')
        with (self.path / 'results.csv').open('w', newline='') as out:
            csv.DictWriter(out, fieldnames=self.FIELDS).writeheader()
        (self.path / 'results.jsonl').touch()
        (self.path / 'index.html').write_text(
            '<!doctype html><html lang="it"><meta charset="utf-8">'
            '<title>Campioni analizzati — lupo</title>'
            '<style>body{font:16px system-ui;max-width:1100px;margin:2em auto;padding:1em}'
            'article{border:1px solid #aaa;padding:1em;margin:1em 0}img{max-width:100%}'
            'pre{white-space:pre-wrap}a{margin-right:1em}</style>'
            '<h1>Campioni realmente analizzati</h1>'
            '<p>Positivo = lupo nel singolo fotogramma. Confermato = almeno due positivi '
            'negli ultimi 3 secondi. Gli esiti sono del modello, non etichette verificate '
            'da una persona. Confidence 0 significa nessun lupo sopra soglia, non certezza '
            'di assenza. Aggiornare la pagina durante la prova.</p>'
            '<p><a href="results.csv">CSV</a><a href="results.jsonl">JSON completo</a>'
            '<a href="metadata.json">Configurazione</a><a href="summary.json">Riepilogo</a></p>\n')
        self.summary('running')

    def summary(self, state, error=''):
        data = dict(state=state, analyzed_samples=self.count,
                    positive_samples=self.positives, confirmed_samples=self.confirmations,
                    error=error)
        temp = self.path / 'summary.tmp'
        temp.write_text(json.dumps(data, indent=2) + '\n')
        temp.replace(self.path / 'summary.json')

    def save(self, bgr, result, boxes):
        number = self.count + 1
        row = dict(result, sample=number, boxes=boxes, image='', annotated='')
        if bgr is not None:
            row['image'] = f'images/{number:06d}.png'
            row['annotated'] = f'annotated/{number:06d}.jpg'
            overlay = bgr.copy()
            for box in boxes:
                x1, y1, x2, y2 = map(lambda x: int(round(x)), box['xyxy'])
                self.cv.rectangle(overlay, (x1, y1), (x2, y2), (0, 200, 0), 2)
                self.cv.putText(overlay, f"{box['label']} {box['confidence']:.2f}",
                                (max(0, x1), max(18, y1)), self.cv.FONT_HERSHEY_SIMPLEX,
                                .6, (0, 200, 0), 2)
            for key, pixels in (('image', bgr), ('annotated', overlay)):
                if not self.cv.imwrite(str(self.path / row[key]), pixels):
                    raise OSError(f'Cannot write {self.path / row[key]}')
        # JSON preserves all boxes and invalid/discarded inference reasons.
        with (self.path / 'results.jsonl').open('a') as out:
            out.write(json.dumps(row, allow_nan=False) + '\n')
        with (self.path / 'results.csv').open('a', newline='') as out:
            csv.DictWriter(out, fieldnames=self.FIELDS, extrasaction='ignore').writerow(row)
        label = (f"Campione {number} — tempo video: {row.get('video_time_sec', 'ROS')} s"
                 f" — positivo: {row.get('frame_positive')} — confermato: {row.get('detected')}")
        description = {k: v for k, v in row.items() if k not in ('image', 'annotated', 'boxes')}
        card = '<article><h2>' + html.escape(label) + '</h2>'
        if row['image']:
            card += (f'<a href="{row["image"]}">Originale PNG</a>'
                     f'<a href="{row["annotated"]}">Immagine con riquadri</a>'
                     f'<p><img loading="lazy" src="{row["annotated"]}"></p>')
        card += '<pre>' + html.escape(json.dumps(description, indent=2)) + '</pre></article>\n'
        with (self.path / 'index.html').open('a') as out:
            out.write(card)
        self.count = number
        self.positives += int(bool(row.get('frame_positive')))
        self.confirmations += int(bool(row.get('detected')) and bool(row.get('valid')))
        self.summary('running')
