"""Paired EKF input intervention; recorded trajectory is not re-simulated.

Both instances start from the same recorded pose/covariance at 240 s.
The original EKF's hidden history is unavailable, so this is a controlled
consumer comparison, not an exact replay of its internal state.
"""
import copy
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool

root = Path('/output/20260908-mpcc-stop-reference-single-r3')
pose_topic = '/localization/imu_gnss_poser/pose_with_covariance'
twist_topic = '/localization/twist_estimator/twist_with_covariance'
odom_topic = '/localization/kinematic_state'
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(
    uri=str(root / 'first-failure-partial-bag/failure-prefix.mcap'), storage_id='mcap'),
    rosbag2_py.ConverterOptions('', ''))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
reader.set_filter(rosbag2_py.StorageFilter(topics=[pose_topic, twist_topic, odom_topic, '/clock']))


def seconds(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


events = []
seed = None
while reader.has_next():
    topic, data, received = reader.read_next()
    msg = deserialize_message(data, types[topic])
    stamp = seconds(msg.clock if topic == '/clock' else msg.header.stamp)
    if stamp > 251.1:
        break
    if stamp < 240:
        continue
    if topic == odom_topic:
        if seed is None:
            seed = msg
        continue
    events.append((received, topic, msg))
assert seed is not None and len(events) > 1000

processes, log_files = [], []
rclpy.init()
node = rclpy.create_node('sealed_covariance_comparison')
clock_pub = node.create_publisher(Clock, '/clock', 100)
arms = {}
params = dict(use_sim_time='true', enable_yaw_bias_estimation='false',
              predict_frequency='50.0', tf_rate='30.0', extend_state_step='100',
              pose_smoothing_steps='1', twist_smoothing_steps='1',
              proc_stddev_vx_c='10.0', proc_stddev_wz_c='5.0',
              pose_additional_delay='0.0', pose_frame_id='map')
try:
    for name, covariance in [('recorded100', None), ('calibrated01', .1)]:
        command = ['/autoware/install/ekf_localizer/lib/ekf_localizer/ekf_localizer',
                   '--ros-args', '-r', f'__ns:=/{name}']
        for key, value in params.items():
            command += ['-p', f'{key}:={value}']
        log = (root / f'covariance-replay-{name}.log').open('w')
        log_files.append(log)
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        rows = []

        def observe(msg, rows=rows):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            rows.append([seconds(msg.header.stamp), p.x, p.y, yaw])

        arms[name] = dict(
            covariance=covariance, rows=rows,
            pose=node.create_publisher(PoseWithCovarianceStamped, f'/{name}/in_pose_with_covariance', 100),
            twist=node.create_publisher(TwistWithCovarianceStamped, f'/{name}/in_twist_with_covariance', 100),
            initial=node.create_publisher(PoseWithCovarianceStamped, f'/{name}/initialpose', 10),
            client=node.create_client(SetBool, f'/{name}/trigger_node_srv'),
            subscription=node.create_subscription(Odometry, f'/{name}/ekf_odom', observe, 100))
    for arm in arms.values():
        assert arm['client'].wait_for_service(timeout_sec=10)
    first_clock = next(msg for _, topic, msg in events if topic == '/clock')
    initial = PoseWithCovarianceStamped()
    initial.header = seed.header
    initial.pose = seed.pose
    for _ in range(20):
        clock_pub.publish(first_clock)
        for arm in arms.values():
            arm['initial'].publish(initial)
        rclpy.spin_once(node, timeout_sec=.05)
    for arm in arms.values():
        request = SetBool.Request(data=True)
        future = arm['client'].call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.result() is not None and future.result().success
    start = time.monotonic()
    origin = events[0][0]
    for received, topic, msg in events:
        target = start + (received - origin) * 1e-9
        while time.monotonic() < target:
            rclpy.spin_once(node, timeout_sec=min(.002, max(0, target-time.monotonic())))
        if topic == '/clock':
            clock_pub.publish(msg)
        else:
            for arm in arms.values():
                value = copy.deepcopy(msg)
                if topic == pose_topic:
                    if arm['covariance'] is not None:
                        for index in (0, 7, 14):
                            value.pose.covariance[index] = arm['covariance']
                    arm['pose'].publish(value)
                else:
                    arm['twist'].publish(value)
        rclpy.spin_once(node, timeout_sec=0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=.01)
    result = dict(method=__doc__, parameters=params, seed_source=seconds(seed.header.stamp),
                  rows={name: arm['rows'] for name, arm in arms.items()})
    (root / 'covariance-consumer-comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    for name, arm in arms.items():
        print(name, 'samples', len(arm['rows']),
              'near250.6', min(arm['rows'], key=lambda r: abs(r[0]-250.6)), flush=True)
finally:
    for process in processes:
        process.terminate()
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    for log in log_files:
        log.close()
    node.destroy_node()
    rclpy.shutdown()
