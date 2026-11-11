"""Run the actual ROS video pipeline and collect mission evidence, without motion."""
import argparse
import csv
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def stop_launch(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)  # launch forwards it to its children
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def add_source_positions(output):
    """Join ROS image timestamps to exact video positions after recording closes."""
    from go2_animal_detector.recording import SampleRecorder
    positions = {}
    with (output / 'events.jsonl').open() as events:
        for line in events:
            event = json.loads(line)
            if event['topic'] == 'sample':
                data = event['data']
                positions[round(data['frame_stamp_unix'], 6)] = data
    for results in output.rglob('results.jsonl'):
        rows = [json.loads(line) for line in results.read_text().splitlines()]
        page = results.parent / 'index.html'
        html = page.read_text()
        for row in rows:
            position = positions.get(round(row['frame_stamp_unix'], 6))
            if position:
                row.update(source_frame=position['source_frame'], video_time_sec=position['video_time_sec'])
                html = html.replace(f"Campione {row['sample']} — tempo video: ROS s",
                                    f"Campione {row['sample']} — tempo video: {row['video_time_sec']:.3f} s")
        results.write_text(''.join(json.dumps(row, allow_nan=False) + '\n' for row in rows))
        with (results.parent / 'results.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=SampleRecorder.FIELDS, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
        page.write_text(html)


class ReviewState:
    """Decision logic for real ROS observations; never injects detections."""
    def __init__(self, full_video=False):
        self.full_video = full_video
        self.confirmed = False
        self.first_detection_playback_sec = None
        self.latest = {}
        self.counts = Counter()
        self.eof_since = None
        self.last_result_at = None
        self.violation = ''

    def receive(self, key, data, now):
        self.latest[key] = (data, now)
        self.counts[key] += 1
        if key == 'result':
            self.last_result_at = now
            if data.get('valid'):
                self.counts['valid_results'] += 1

    def outcome(self, now):
        if self.violation:
            raise RuntimeError(self.violation)
        for key in ('mission', 'detector', 'video'):
            data = self.latest.get(key, ({}, 0))[0]
            if key == 'mission' and data and data.get('mode') != 'DRY_RUN_ONLY':
                raise RuntimeError('Test mission manager must have motion disabled')
            if key == 'detector' and (data.get('state') in ('MODEL_ERROR', 'INFERENCE_ERROR')
                                      or data.get('recording_error')):
                raise RuntimeError('Detector: ' + str(data.get('recording_error') or data.get('error')))
            if key == 'video' and data.get('state') == 'ERROR':
                raise RuntimeError('Video: ' + str(data.get('error')))
        required = ('mission', 'detector', 'video', 'control', 'flag')
        if any(k not in self.latest for k in required):
            return None
        # The flag is transient-local and only published on change; other status
        # messages must stay fresh throughout the test.
        if any(now - self.latest[k][1] > 3 for k in required if k != 'flag'):
            return None
        mission = self.latest['mission'][0]
        control = self.latest['control'][0]
        if (mission.get('state') == 'WOLF_DETECTED' and self.latest['flag'][0] is True
                and not control.get('enabled') and self.counts['valid_results']):
            if not self.confirmed:
                self.first_detection_playback_sec = self.latest['video'][0].get('playback_sec')
            self.confirmed = True
            if not self.full_video:
                return 'WOLF_DETECTED'
        if self.latest['video'][0].get('state') == 'EOF':
            if self.eof_since is None:
                self.eof_since = now
            # Drain the last image through inference and the ROS result callbacks.
            if now - max(self.eof_since, self.last_result_at or 0) >= 3:
                if not self.counts['valid_results']:
                    raise RuntimeError('Video ended without a valid inference result')
                return 'EOF_AFTER_WOLF_DETECTED' if self.confirmed else 'EOF_WITHOUT_CONFIRMATION'
        return None


class DiagnosticControl:
    """Test-only control: start with the manager, then analyze through video EOF."""
    def __init__(self):
        self.identity = None

    def command(self, manager_control, wall):
        if manager_control.get('enabled') is True:
            identity = (manager_control.get('mission_id'), manager_control.get('session_id'))
            if all(isinstance(v, str) and v for v in identity):
                self.identity = identity
        return dict(enabled=bool(self.identity),
                    mission_id=self.identity[0] if self.identity else '',
                    session_id=self.identity[1] if self.identity else '', stamp_unix=wall)


def review(video, output, timeout=1800, full_video=False):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, Bool
    from go2_interfaces.msg import MotionPermit, MotionCommand
    if os.environ.get('ROS_DOMAIN_ID') != '84':
        raise RuntimeError('Video review requires isolated ROS_DOMAIN_ID=84')
    output.mkdir(parents=True, exist_ok=False)
    state = ReviewState(full_video=full_video)
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    flag_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init()
    node = rclpy.create_node('wolf_video_review_observer')
    process = None
    report = dict(video=str(video), mode='ros_video_full' if full_video else 'ros_video',
                  full_video=full_video, diagnostic_control_override=full_video,
                  domain=84, physical_stop_tested=False)
    with (output / 'events.jsonl').open('w') as events, (output / 'ros_launch.log').open('w') as log:
        def receive(key, boolean=False):
            def callback(msg):
                data = msg.data if boolean else json.loads(msg.data)
                state.receive(key, data, time.monotonic())
                events.write(json.dumps(dict(topic=key, received_unix=time.time(), data=data)) + '\n')
                events.flush()
            return callback
        def permit(msg):
            if msg.enabled:
                state.violation = 'Unexpected motion permit in isolated test'
        def command(msg):
            t = msg.twist
            if any(v != 0 for v in (t.linear.x, t.linear.y, t.linear.z,
                                    t.angular.x, t.angular.y, t.angular.z)):
                state.violation = 'Unexpected nonzero velocity in isolated test'
        for key, topic in (('mission', '/mission/status'), ('detector', '/identification/status'),
                           ('control', '/identification/control'), ('result', '/identification/result'),
                           ('sample', '/video/sample'),
                           ('video', '/video/status')):
            node.create_subscription(String, '/vision_test' + topic, receive(key), qos)
        node.create_subscription(Bool, '/vision_test/mission/wolf_detected', receive('flag', True), flag_qos)
        node.create_subscription(MotionPermit, '/vision_test/go2/control_permit', permit, qos)
        node.create_subscription(MotionCommand, '/vision_test/go2/cmd_vel', command, qos)
        if full_video:
            diagnostic = DiagnosticControl()
            control_pub = node.create_publisher(String, '/vision_test/diagnostics/control', qos)
            def keep_analyzing():
                command = diagnostic.command(state.latest.get('control', ({}, 0))[0], time.time())
                control_pub.publish(String(data=json.dumps(command)))
            node.create_timer(.05, keep_analyzing)
        try:
            existing = [n for n, _ in node.get_node_names_and_namespaces()
                        if n.startswith('video_test_')]
            if existing:
                raise RuntimeError('Another video test is running in domain 84: ' + ', '.join(existing))
            launch_command = ['ros2', 'launch', 'go2_vision_test', 'video_test.launch.py',
                 'video_path:=' + str(video), 'fps:=1.5', 'loop:=false',
                 'record_directory:=' + str(output / 'samples')]
            if full_video:
                launch_command.append('detector_control_topic:=/vision_test/diagnostics/control')
            process = subprocess.Popen(launch_command,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline, last_print = time.monotonic() + timeout, 0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if process.poll() is not None:
                    raise RuntimeError('ROS launch exited unexpectedly; see ros_launch.log')
                now = time.monotonic()
                outcome = state.outcome(now)
                if now - last_print >= 5:
                    mission = state.latest.get('mission', ({}, 0))[0]
                    detector = state.latest.get('detector', ({}, 0))[0]
                    print(f'{video.name}: manager={mission.get("state")} '
                          f'detector={detector.get("state")} '
                          f'analisi={state.counts["valid_results"]} '
                          f'video_sec={state.latest.get("video", ({}, 0))[0].get("playback_sec")}', flush=True)
                    last_print = now
                if outcome:
                    report.update(outcome=outcome, counts=dict(state.counts),
                                  first_detection_playback_sec=state.first_detection_playback_sec,
                                  final_states={k: v[0] for k, v in state.latest.items()})
                    print(f'{video.name}: {outcome}', flush=True)
                    return report
            raise RuntimeError('ROS video test timed out; inspect events.jsonl and ros_launch.log')
        except BaseException as exc:
            report.update(outcome='ERROR', error=str(exc), counts=dict(state.counts))
            raise
        finally:
            if process is not None:
                stop_launch(process)
            events.flush()
            try:
                add_source_positions(output)
            except Exception as exc:
                report['source_mapping_error'] = str(exc)
            (output / 'ros_report.json').write_text(json.dumps(report, indent=2) + '\n')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--full-video', action='store_true',
                        help='Continue diagnostic ROS analysis through EOF after the manager stops.')
    args = parser.parse_args()
    videos = (sorted(p for p in args.video.iterdir() if p.is_file() and
                     p.suffix.lower() in ('.mp4', '.mov', '.mkv', '.avi', '.webm', '.m4v'))
              if args.video.is_dir() else [args.video])
    if not videos or any(not p.is_file() for p in videos):
        parser.error('No readable videos found')
    args.output.mkdir(parents=True, exist_ok=True)
    failures = []
    with open('/tmp/go2_ros_video_review.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for i, video in enumerate(videos):
            try:
                review(video, args.output / f'{i + 1:03d}_{video.stem}', full_video=args.full_video)
            except Exception as exc:
                failures.append(str(exc))
                print(f'ERROR {video.name}: {exc}', flush=True)
    if failures:
        raise SystemExit('Some ROS video tests failed; inspect reports.')


if __name__ == '__main__':
    main()
