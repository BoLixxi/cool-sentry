"""Read actual MID360 PointCloud2 and IMU over ROS2; starts no navigation/motors."""
import argparse
import math
import os
import pathlib
import signal
import struct
import subprocess
import time

import rclpy
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.qos import qos_profile_sensor_data


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--seconds', type=float, default=12)
    args = cli.parse_args()
    if args.seconds < 5:
        cli.error('--seconds must be at least 5')
    config = pathlib.Path(__file__).resolve().parents[1] / 'deploy/mid360_bench.json'
    namespace = '/mid360_bench_' + str(os.getpid())
    rclpy.init()
    node = rclpy.create_node('stream_check', namespace=namespace)
    clouds, imus, sizes, samples = [], [], [], []

    def cloud(msg):
        count = msg.width * msg.height
        if not count or len(msg.data) < msg.row_step * msg.height:
            return
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields and fields[k].datatype == 7 for k in ('x', 'y', 'z')):
            return
        endian = '>' if msg.is_bigendian else '<'
        xyz = tuple(struct.unpack_from(endian + 'f', msg.data, fields[k].offset)[0]
                    for k in ('x', 'y', 'z'))
        if not all(math.isfinite(v) for v in xyz):
            return
        clouds.append(time.monotonic())
        sizes.append(count)
        samples.append(xyz)

    def imu(msg):
        values = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        if all(math.isfinite(v) for v in values):
            imus.append(time.monotonic())

    node.create_subscription(PointCloud2, namespace + '/livox/lidar', cloud, qos_profile_sensor_data)
    node.create_subscription(Imu, namespace + '/livox/imu', imu, qos_profile_sensor_data)
    proc = None
    stop_code = 0
    try:
        proc = subprocess.Popen([
            'ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node', '--ros-args',
            '-r', '__ns:=' + namespace, '-p', 'user_config_path:=' + str(config),
            '-p', 'xfer_format:=0', '-p', 'multi_topic:=0', '-p', 'data_src:=0',
            '-p', 'publish_freq:=10.0', '-p', 'output_data_type:=0',
            '-p', 'frame_id:=livox_frame'], start_new_session=True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError('LiDAR driver exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.05)
        if len(clouds) < 10 or len(imus) < 10:
            raise RuntimeError(f'Missing sensor stream: clouds={len(clouds)}, IMU={len(imus)}')
        for name, times in (('PointCloud2', clouds), ('IMU', imus)):
            hz = (len(times) - 1) / (times[-1] - times[0])
            gap = max(b-a for a, b in zip(times, times[1:]))
            print(f'{name}: {len(times)} messages, {hz:.2f} Hz, maximum gap {gap:.3f}s', flush=True)
        print(f'Points/frame: {min(sizes)}..{max(sizes)}, sample XYZ={samples[-1]}', flush=True)
        print('PASS MID360 real pointcloud + IMU reception (not navigation acceptance)', flush=True)
    finally:
        if proc and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                stop_code = proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                stop_code = proc.wait()
        node.destroy_node()
        rclpy.shutdown()
        if stop_code:
            raise SystemExit(f'Sensor reception completed, but driver shutdown failed: exit {stop_code}')


if __name__ == '__main__':
    main()
