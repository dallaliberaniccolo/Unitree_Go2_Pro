#!/usr/bin/env python3
"""Real CPU/ROS installation checks; no physical robot or wolf footage required."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import time


def model_check():
    import numpy as np
    import yaml
    import torch
    import torchvision
    import ultralytics
    from go2_animal_detector.backend import YoloBackend
    cfg = yaml.safe_load(Path('/ws/config/go2_waypoint_diretto.yaml').read_text())
    cfg = cfg['/go2_animal_detector']['ros__parameters']
    profile = json.loads(Path('/ws/models/wolf/inference_config.json').read_text())
    if ultralytics.__version__ != profile['ultralytics']:
        raise RuntimeError('Ultralytics version differs from supplied model profile')
    if torch.version.cuda is not None:
        raise RuntimeError('Expected a CPU PyTorch build on Raspberry')
    backend = YoloBackend(cfg)
    # Confirm the torchvision extension matches the installed PyTorch wheel.
    assert torchvision.ops.nms(torch.tensor([[0., 0., 1., 1.]]), torch.tensor([.9]), .5).tolist() == [0]
    if backend.classes != [int(profile['wolf_id'])]:
        raise RuntimeError(f'Unexpected wolf class mapping: {backend.classes}')
    blank = np.zeros((cfg['image_size'], cfg['image_size'], 3), dtype=np.uint8)
    started = time.monotonic()
    backend.warmup()  # same initialization used by the ROS worker; no real frame consumed
    warmup_ms = (time.monotonic() - started) * 1000
    timings = []
    scores = []
    for _ in range(3):
        start = time.monotonic()
        scores.append(backend.predict(blank))
        timings.append(time.monotonic() - start)
    return dict(check='model_cpu', torch=torch.__version__, ultralytics=ultralytics.__version__,
                classes=backend.model.names, wolf_classes=backend.classes,
                input_size=cfg['image_size'], blank_scores=scores,
                warmup_ms=round(warmup_ms, 1),
                inference_ms=[round(v * 1000, 1) for v in timings],
                approximate_serial_fps=round(len(timings) / sum(timings), 2),
                max_rss_mib=round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1),
                measured_on='blank images, model process only', physical_stop_tested=False)


def stop_process_group(process):
    if process.poll() is not None:
        return
    # ROS launch forwards SIGINT to children. Signalling the whole group also
    # interrupts their finally blocks a second time during cleanup.
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def ros_check(kind, output):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool, String
    from sensor_msgs.msg import Image
    from go2_interfaces.msg import MotionPermit, MotionCommand
    isolated = kind == 'isolated'
    if isolated and os.environ.get('ROS_DOMAIN_ID') != '84':
        raise RuntimeError('Isolated checks require ROS_DOMAIN_ID=84')
    prefix = '/vision_test' if isolated else ''
    reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    flag_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latest = {}
    counts = Counter()
    violations = []
    process = None
    log = None
    rclpy.init()
    node = rclpy.create_node('go2_install_verifier')

    def receive(key):
        def callback(msg):
            latest[key] = json.loads(msg.data)
            counts[key] += 1
        return callback

    def permit(msg):
        if msg.enabled:
            violations.append('enabled motion permit')

    def command(msg):
        t = msg.twist
        if any(v != 0 for v in (t.linear.x, t.linear.y, t.linear.z,
                                t.angular.x, t.angular.y, t.angular.z)):
            violations.append('nonzero velocity')

    def result(msg):
        data = json.loads(msg.data)
        if data.get('valid') is True and data.get('detected') is False:
            counts['real_negative_results'] += 1

    node.create_subscription(String, prefix + '/mission/status', receive('mission'), reliable)
    node.create_subscription(String, prefix + '/identification/status', receive('detector'), reliable)
    node.create_subscription(String, prefix + '/identification/control', receive('control'), reliable)
    node.create_subscription(Bool, prefix + '/mission/wolf_detected',
                             lambda m: latest.update(flag=m.data), flag_qos)
    node.create_subscription(Image, prefix + '/go2/camera/image_raw',
                             lambda m: counts.update(images=1), image_qos)
    node.create_subscription(MotionPermit, prefix + '/go2/control_permit', permit, reliable)
    node.create_subscription(MotionCommand, prefix + '/go2/cmd_vel', command, reliable)
    node.create_subscription(String, prefix + '/identification/result', result, reliable)
    injection_pub = None
    try:
        if isolated:
            import cv2
            import numpy as np
            clip = output.parent / 'blank_install_check.avi'
            writer = cv2.VideoWriter(str(clip), cv2.VideoWriter_fourcc(*'MJPG'), 5., (256, 192))
            if not writer.isOpened():
                raise RuntimeError('OpenCV cannot write MJPG test video')
            try:
                frame = np.zeros((192, 256, 3), dtype=np.uint8)
                for _ in range(150):
                    writer.write(frame)
            finally:
                writer.release()
            log = (output.parent / 'isolated_ros.log').open('w')
            process = subprocess.Popen(['ros2', 'launch', 'go2_vision_test', 'video_test.launch.py',
                                        'video_path:=' + str(clip), 'loop:=true'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            injection_pub = node.create_publisher(String, prefix + '/identification/result', reliable)
        deadline = time.monotonic() + (180 if isolated else 90)
        last_print = 0.
        injected = False
        injection_started = None
        stable_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            wall = time.time()
            mission = latest.get('mission', {})
            detector = latest.get('detector', {})
            control = latest.get('control', {})
            if process is not None and process.poll() is not None:
                raise RuntimeError('Video test launch exited; see isolated_ros.log')
            if violations:
                raise RuntimeError('Unexpected motion output: ' + ', '.join(violations))
            if detector.get('state') in ('MODEL_ERROR', 'INFERENCE_ERROR'):
                raise RuntimeError('Detector: ' + str(detector.get('error'))
                                   + f', inference_ms={detector.get("inference_ms")}'
                                   + f', warmup_ms={detector.get("warmup_ms")}')
            if now - last_print >= 10:
                print(f'ROS check {kind}: manager={mission.get("state")}, '
                      f'detector={detector.get("state")}, images={counts["images"]}, '
                      f'negative_results={counts["real_negative_results"]}', flush=True)
                last_print = now
            fresh = (0 <= wall - mission.get('stamp_unix', 0) < 2
                     and 0 <= wall - detector.get('stamp_unix', 0) < 2)
            if not fresh:
                continue
            if isolated:
                if mission.get('mode') != 'DRY_RUN_ONLY':
                    raise RuntimeError('Test manager has motion enabled')
                if not injected and latest.get('flag') is True:
                    raise RuntimeError('Blank video produced an unexpected wolf confirmation')
                if (not injected and mission.get('state') == 'NAVIGATING_DRY_RUN'
                        and counts['real_negative_results'] >= 3 and counts['images'] >= 3
                        and detector.get('model_ready') and control.get('enabled')):
                    injected = True
                    injection_started = now
                    print('Real ROS/model negative path passed. Injecting a SIMULATED true event '
                          'on the isolated test topic to verify the manager flag.', flush=True)
                if injected and control.get('enabled'):
                    injection_pub.publish(String(data=json.dumps(dict(
                        mission_id=control['mission_id'], session_id=control['session_id'],
                        valid=True, detected=True, confidence=1., frame_stamp_unix=wall,
                        stamp_unix=wall, install_check_injected=True))))
                if (injected and latest.get('flag') is True
                        and mission.get('state') == 'WOLF_DETECTED'
                        and mission.get('wolf_detected') is True and not control.get('enabled')):
                    return dict(check='isolated_ros', counts=dict(counts),
                                detector_model_ready=True, manager_state=mission['state'],
                                positive_event='synthetically injected; no wolf footage used',
                                physical_stop_tested=False)
                if injection_started is not None and now - injection_started > 10:
                    raise RuntimeError('Manager did not latch the injected positive event')
            else:
                if mission.get('state') != 'IDLE' or mission.get('goal') is not None:
                    raise RuntimeError('Production manager has an unexpected active mission')
                if mission.get('identification_enable') or latest.get('flag') is True:
                    raise RuntimeError('Unexpected detection activation without a mission')
                names = Counter(name for name, _ in node.get_node_names_and_namespaces())
                expected = ('go2_gps_sensor', 'go2_imu_mag_sensor', 'go2_navigation',
                            'go2_bridge', 'go2_mission_manager', 'go2_animal_detector')
                good = detector.get('model_ready') and all(names[name] == 1 for name in expected)
                stable_since = now if good and stable_since is None else stable_since if good else None
                if stable_since is not None and now - stable_since >= 5:
                    return dict(check='production_idle', manager_state='IDLE', goal=None,
                                model_ready=True, nodes={n: names[n] for n in expected},
                                preflight_errors=mission.get('preflight_errors'),
                                robot_required=False, physical_stop_tested=False)
        raise RuntimeError(f'{kind} ROS check timed out: {json.dumps(latest)}')
    finally:
        if process is not None:
            stop_process_group(process)
        if log is not None:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('check', choices=['model', 'isolated', 'observe'])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = model_check() if args.check == 'model' else ros_check(args.check, args.output)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
