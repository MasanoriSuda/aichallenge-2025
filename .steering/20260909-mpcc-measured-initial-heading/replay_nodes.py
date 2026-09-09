"""Actual GNSS and initializer nodes, original sensor messages, isolated ROS domain.

Preserves recorded arrival order but accelerates wall time. It verifies source
joins, transforms and initialization, not the original executor or MPCC motion.
"""
from pathlib import Path
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import Imu, NavSatFix
from std_srvs.srv import SetBool, Trigger
from tf2_ros import StaticTransformBroadcaster


def stamp(message):
    s = message['header']['stamp']
    return s['sec']*1000000000+s['nanosec']


def quaternion_product(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return [aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz]


def angular_error(a, b):
    an = math.sqrt(sum(x*x for x in a)); bn = math.sqrt(sum(x*x for x in b))
    aa = [x/an for x in a]; bb = [x/bn for x in b]
    # Chord-distance form remains well conditioned at tiny rotation error.
    chord = min(math.sqrt(sum((x-y)**2 for x, y in zip(aa, bb))),
                math.sqrt(sum((x+y)**2 for x, y in zip(aa, bb))))
    return 4*math.asin(min(1, chord/2))


source, out = map(Path, sys.argv[1:3]); out.mkdir(parents=True, exist_ok=False)
rows = json.loads(source.read_text())
inputs = [r for r in rows if r['topic'] in ['/sensing/gnss/nav_sat_fix', '/sensing/imu/imu_raw']]
originals = {stamp(r['message']): r['message'] for r in rows
             if r['topic'] == '/sensing/gnss/pose_with_covariance'}
fix_stamps = {stamp(r['message']) for r in inputs if r['topic'].endswith('nav_sat_fix')}
imu_stamps = {stamp(r['message']) for r in inputs if r['topic'].endswith('imu_raw')}
paired = fix_stamps & imu_stamps
assert fix_stamps == paired, 'recording lacks same-source IMU for a GNSS fix'
rclpy.init()
peer = rclpy.create_node('recorded_heading_peer')
poses = {}; initials = []; fused_stamps = set(); processes = []; logs = []


def spin_until(predicate, seconds=3):
    deadline = time.monotonic()+seconds
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(peer, timeout_sec=.01)
    assert predicate(), 'replay condition did not become ready'


try:
    reliable = QoSProfile(depth=10)
    transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pose_sub = peer.create_subscription(PoseStamped, '/replay/gnss_pose',
        lambda m: poses.update({stamp(message_to_ordereddict(m)): message_to_ordereddict(m)}), reliable)
    initial_sub = peer.create_subscription(PoseWithCovarianceStamped, '/localization/initial_pose3d',
        lambda m: initials.append(message_to_ordereddict(m)), transient)
    fused_sub = peer.create_subscription(PoseWithCovarianceStamped, '/localization/imu_gnss_poser/pose_with_covariance',
        lambda m: fused_stamps.add(stamp(message_to_ordereddict(m))), reliable)
    trigger_service = peer.create_service(SetBool, '/localization/trigger_node',
        lambda request, response: setattr(response, 'success', request.data) or response)
    fixes = peer.create_publisher(NavSatFix, '/replay/fix', reliable)
    imus = peer.create_publisher(Imu, '/replay/imu', reliable)
    broadcaster = StaticTransformBroadcaster(peer)
    gnss = TransformStamped(); gnss.header.frame_id = 'base_link'; gnss.child_frame_id = 'gnss_link'
    gnss.transform.translation.x = -.26; gnss.transform.rotation.w = 1.
    imu = TransformStamped(); imu.header.frame_id = 'base_link'; imu.child_frame_id = 'imu_link'
    imu.transform.translation.x = .85
    imu.transform.rotation.z = math.sin(math.pi/4); imu.transform.rotation.w = math.cos(math.pi/4)
    broadcaster.sendTransform([gnss, imu])
    commands = [
        ['ros2', 'run', 'racing_kart_gnss_poser', 'gnss_poser', '--ros-args',
         '-p', 'use_imu_orientation:=true', '-p', 'use_gnss_ins_orientation:=false',
         '-p', 'base_frame:=base_link', '-p', 'gnss_frame:=gnss_link',
         '-p', 'gnss_base_frame:=replay_gnss_output', '-p', 'gnss_change_threshold:=0.2',
         '-p', 'unknown_position_covariance:=0.1', '-r', 'fix:=/replay/fix', '-r', 'imu:=/replay/imu',
         '-r', 'gnss_pose:=/replay/gnss_pose', '-r', 'gnss_pose_cov:=/sensing/gnss/pose_with_covariance'],
        ['ros2', 'run', 'imu_gnss_poser', 'imu_gnss_poser_node', '--ros-args',
         '-p', 'initial_pose_heading_source:=measurement', '-p', 'initial_pose_service:=/replay/set_initial_pose']]
    for index, command in enumerate(commands):
        log = (out/(str(index)+'-node.log')).open('w'); logs.append(log)
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    service = peer.create_client(Trigger, '/replay/set_initial_pose')
    spin_until(lambda: fixes.get_subscription_count() and imus.get_subscription_count() and service.service_is_ready(), 15)
    spin_until(lambda: peer.count_subscribers('/sensing/gnss/pose_with_covariance') > 0, 5)
    # DDS graph discovery precedes delivery of transient-local TF. Allow the
    # test's static fixture to reach the independent listener before input one.
    # Missing TF is separately required to fail closed by the native test.
    fixture_deadline = time.monotonic()+.5
    while time.monotonic() < fixture_deadline:
        rclpy.spin_once(peer, timeout_sec=.01)
    # Service before the first measurement must fail without fabricating a pose.
    early = service.call_async(Trigger.Request()); spin_until(early.done)
    assert not early.result().success and not initials
    sent_fix = set(); sent_imu = set(); imu_by_stamp = {}
    for row in inputs:
        key = stamp(row['message'])
        if row['topic'].endswith('nav_sat_fix'):
            message = NavSatFix(); set_message_fields(message, row['message'])
            fixes.publish(message); sent_fix.add(key)
        else:
            message = Imu(); set_message_fields(message, row['message'])
            imus.publish(message); sent_imu.add(key); imu_by_stamp[key] = row['message']
        # Let this exact arrival reach the node before the next recorded arrival.
        # Checking each pair prevents faster replay from creating QoS-depth loss.
        if key in sent_fix & sent_imu:
            spin_until(lambda: key in poses, 3)
        else:
            for _ in range(3):
                rclpy.spin_once(peer, timeout_sec=.003)
    spin_until(lambda: bool(initials) and max(paired) in fused_stamps)
    automatic = initials[0]
    before_service = len(initials)
    result = service.call_async(Trigger.Request()); spin_until(result.done)
    assert result.result().success
    spin_until(lambda: len(initials) > before_service)
    explicit = initials[-1]
    assert stamp(automatic) == min(paired)
    assert stamp(explicit) == max(paired)
    for initial in [automatic, explicit]:
        expected = poses[stamp(initial)]['pose']; actual = initial['pose']['pose']
        assert actual['position'] == expected['position']
        assert angular_error([actual['orientation'][k] for k in 'xyzw'],
                             [expected['orientation'][k] for k in 'xyzw']) < 1e-12
    comparisons = []
    for key in sorted(paired):
        q = [imu_by_stamp[key]['orientation'][k] for k in 'xyzw']
        expected = quaternion_product(q, [0, 0, -math.sin(math.pi/4), math.cos(math.pi/4)])
        actual = [poses[key]['pose']['orientation'][k] for k in 'xyzw']
        error = angular_error(expected, actual)
        assert error < 1e-10
        comparisons.append(dict(stamp_ns=key, attitude_error_rad=error,
                                output=poses[key], original=originals.get(key)))
    report = dict(authority=False, limits=__doc__, source=str(source),
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), commands=commands,
                  paired_inputs=len(paired), outputs=len(poses), automatic_initial=automatic,
                  service_initial=explicit, early_service_success=early.result().success,
                  maximum_attitude_error_rad=max(r['attitude_error_rad'] for r in comparisons),
                  comparisons=comparisons)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ['comparisons', 'automatic_initial', 'service_initial', 'commands']}), flush=True)
finally:
    if not (out/'report.json').exists():
        (out/'incomplete.json').write_text(json.dumps(dict(source=str(source),
            poses=list(poses), initial_count=len(initials),
            last_input_stamp_ns=locals().get('key'), authority=False), indent=2)+'\n')
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5)
    for log in logs:
        log.close()
    peer.destroy_node(); rclpy.shutdown()
