"""Sample a local constant-frame-rate clip against elapsed playback time."""
import math
from pathlib import Path


class VideoReader:
    def __init__(self, path, loop=False, source_fps=0.0, cv_module=None):
        path = Path(path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Video file missing: {path}')
        if cv_module is None:
            import cv2 as cv_module
        self.cv = cv_module
        self.capture = self.cv.VideoCapture(str(path))
        try:
            if not self.capture.isOpened():
                raise ValueError(f'Cannot decode video: {path}')
            metadata_fps = float(self.capture.get(self.cv.CAP_PROP_FPS))
            self.fps = source_fps if source_fps > 0 else metadata_fps
            if not math.isfinite(self.fps) or self.fps <= 0:
                raise ValueError('Video has no usable FPS metadata; set source_fps explicitly')
            count = float(self.capture.get(self.cv.CAP_PROP_FRAME_COUNT))
            self.frame_count = int(count) if math.isfinite(count) and count >= 1 else None
            if loop and self.frame_count is None:
                raise ValueError('Loop playback requires frame-count metadata')
        except Exception:
            self.capture.release()
            raise
        self.loop = loop
        self.frame_index = -1
        self.cycle = 0
        self.finished = False

    def read_at(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Playback time must be finite and non-negative')
        if self.finished:
            return None
        absolute_index = int(elapsed * self.fps)
        if self.frame_count and self.loop:
            cycle, target = divmod(absolute_index, self.frame_count)
        else:
            cycle, target = 0, absolute_index
            if self.frame_count and target >= self.frame_count:
                self.finished = True
                return None
        if cycle != self.cycle or target < self.frame_index:
            if not self.capture.set(self.cv.CAP_PROP_POS_FRAMES, 0):
                raise ValueError('Video backend cannot rewind this clip')
            self.frame_index = -1
            self.cycle = cycle
        if target == self.frame_index:
            return None  # source FPS below output FPS: do not count a frame twice
        while self.frame_index < target:
            if not self.capture.grab():
                if self.frame_count and self.frame_index + 1 < self.frame_count:
                    raise ValueError('Video ended before its declared frame count')
                self.finished = True
                return None
            self.frame_index += 1
        ok, bgr = self.capture.retrieve()
        if not ok or bgr is None:
            raise ValueError(f'Failed to decode video frame {self.frame_index}')
        return bgr

    def close(self):
        self.capture.release()
