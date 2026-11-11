"""Analyze a complete video at source-time 1 FPS without ROS, motion or frame drops."""
import argparse
import json
import math
from pathlib import Path
import time

from go2_animal_detector.backend import YoloBackend
from go2_animal_detector.core import DetectionWindow
from go2_animal_detector.recording import SampleRecorder
from .video import VideoReader


def analyze(path, model, output, fps=1.0, source_fps=0.0, backend_factory=YoloBackend,
            reader_factory=VideoReader, recorder_factory=SampleRecorder):
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('FPS must be positive and finite')
    profile = json.loads((Path(model).parent / 'inference_config.json').read_text())
    config = dict(model_path=str(model), inference_threads=2, image_size=profile['imgsz'],
                  target_class_names=['wolf', 'lupo'], target_class_id=profile['wolf_id'],
                  confidence_threshold=profile['conf'], nms_iou=profile['nms_iou'],
                  max_detections=profile['max_det'])
    recorder = recorder_factory(output, dict(
        config, mode='offline_video', video=str(Path(path).resolve()), sample_fps=fps,
        confirmation_frames=2, confirmation_window=0, confirmation_max_age_sec=3.0,
        clock='video source time; processing latency excluded; no ROS or motion'))
    print(f'Report: {recorder.path / "index.html"}', flush=True)
    reader = None
    try:
        reader = reader_factory(path, source_fps=source_fps)
        backend = backend_factory(config)
        backend.warmup()
        window = DetectionWindow(threshold=config['confidence_threshold'])
        tick = 0
        while not reader.finished:
            bgr = reader.read_at(tick / fps)
            tick += 1
            if bgr is None:
                continue
            # Use actual source-frame time, never processing time or repeated frames.
            stamp = reader.frame_index / reader.fps
            started = time.monotonic()
            try:
                prediction = backend.predict_details(bgr)
                confirmation = window.add(stamp, prediction['confidence'])
                if confirmation is None:
                    raise ValueError('Invalid or duplicate model result')
            except Exception as exc:
                recorder.save(bgr, dict(valid=False, error=str(exc),
                              source_frame=reader.frame_index, video_time_sec=stamp,
                              inference_ms=(time.monotonic() - started) * 1000), [])
                raise
            result = dict(valid=True, error='', source_frame=reader.frame_index,
                          video_time_sec=stamp, confidence=prediction['confidence'],
                          inference_ms=(time.monotonic() - started) * 1000, **confirmation)
            recorder.save(bgr, result, prediction['boxes'])
            print(f'{Path(path).name} t={stamp:.2f}s '
                  f'wolf={confirmation["frame_positive"]} '
                  f'confirmed={confirmation["detected"]} '
                  f'confidence={prediction["confidence"]:.3f}', flush=True)
        recorder.summary('complete')
    except BaseException as exc:
        recorder.summary('interrupted' if isinstance(exc, KeyboardInterrupt) else 'error', str(exc))
        raise
    finally:
        if reader is not None:
            reader.close()
    return recorder.path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path, help='Video file or directory')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=float, default=1.0)
    parser.add_argument('--source-fps', type=float, default=0.0)
    args = parser.parse_args()
    videos = (sorted(p for p in args.video.iterdir() if p.is_file() and
                     p.suffix.lower() in ('.mp4', '.mov', '.avi', '.mkv', '.webm', '.m4v'))
              if args.video.is_dir() else [args.video])
    if not videos:
        parser.error('No video files found')
    reports, failures = [], []
    for video in videos:
        try:
            report = analyze(video, args.model, args.output, args.fps, args.source_fps)
            reports.append(dict(video=str(video), report=str(report)))
        except Exception as exc:
            failures.append(dict(video=str(video), error=str(exc)))
            print(f'ERROR {video}: {exc}', flush=True)
    print(json.dumps(dict(reports=reports, failures=failures), indent=2))
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
