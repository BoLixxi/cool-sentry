"""MID360 + Point-LIO reception/odometry bench. Keep radar stationary. No cmd_vel."""
import math
import os
import pathlib
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    ns = '/lio_bench_' + str(os.getpid())
    rclpy.init()
    node = rclpy.create_node('odometry_check', namespace=ns)
    poses, clouds = [], []

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in vals):
            raise RuntimeError('Non-finite odometry')
        poses.append((time.monotonic(), vals))

    node.create_subscription(Odometry, ns + '/aft_mapped_to_init', odom, 20)
    node.create_subscription(PointCloud2, ns + '/cloud_registered',
                             lambda m: clouds.append(m.width * m.height), qos_profile_sensor_data)
    children = []
    try:
        children.append(subprocess.Popen([
            'ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node', '--ros-args',
            '-r', '__ns:=' + ns, '-p', 'user_config_path:=' + str(root/'deploy/mid360_bench.json'),
            '-p', 'xfer_format:=1', '-p', 'publish_freq:=10.0'], start_new_session=True))
        children.append(subprocess.Popen([
            'ros2', 'run', 'point_lio', 'pointlio_mapping', '--ros-args',
            '-r', '__ns:=' + ns, '--params-file', str(root/'deploy/pointlio_mid360_bench.yaml')],
            start_new_session=True))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('LiDAR or Point-LIO exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.05)
        if len(poses) < 20 or not clouds or not max(clouds):
            raise RuntimeError(f'Insufficient output: odometry={len(poses)}, clouds={len(clouds)}')
        # Exclude first 5 seconds of output while map initializes.
        settled = [v for t, v in poses if t >= poses[0][0] + 5]
        origin = settled[0] if settled else poses[0][1]
        drift = max(math.dist(v[:3], origin[:3]) for v in (settled or [p[1] for p in poses]))
        print(f'Odometry={len(poses)}, registered clouds={len(clouds)}, last pose={poses[-1][1]}', flush=True)
        print(f'Stationary positional excursion after initialization={drift:.4f}m', flush=True)
        print('PASS output continuity; calibration and moving localization not validated', flush=True)
    finally:
        codes = []
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
            codes.append(child.returncode)
        node.destroy_node()
        rclpy.shutdown()
        print('Shutdown exit codes (Point-LIO, LiDAR):', codes, flush=True)
        if any(codes):
            raise SystemExit('Shutdown failed')


if __name__ == '__main__':
    main()
