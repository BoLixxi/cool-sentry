"""Interactive LiDAR walk test: RViz + odometry CSV, no robot commands.

Write 'finish' to the stop file to end, or close RViz. Starts baseline after
8 seconds of odometry output; keep LiDAR still until status says ready.
"""
import argparse
import csv
import json
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
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--output', required=True)
    cli.add_argument('--seconds', type=float, default=600)
    args = cli.parse_args()
    output = pathlib.Path(args.output)
    if args.seconds <= 0:
        cli.error('--seconds must be positive')
    if (output/'odometry.csv').exists() or (output/'finish').exists():
        cli.error('Choose a new output directory to preserve previous recordings')
    output.mkdir(parents=True, exist_ok=True)
    root = pathlib.Path(__file__).resolve().parents[1]
    rclpy.init()
    node = rclpy.create_node('pointlio_walk_recorder')
    poses, cloud_times = [], []
    stopped = False

    def stop(*unused):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with (output/'odometry.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['elapsed_s', 'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'])
        start = time.monotonic()

        def odom(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                raise RuntimeError('Non-finite odometry')
            row = (time.monotonic()-start, *values)
            poses.append(row)
            writer.writerow(row)

        node.create_subscription(Odometry, '/aft_mapped_to_init', odom, 20)
        node.create_subscription(PointCloud2, '/cloud_registered',
                                 lambda m: cloud_times.append(time.monotonic()), qos_profile_sensor_data)
        children, logs = [], []
        codes = []
        try:
            commands = [
                ['ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node', '--ros-args',
                 '-p', 'user_config_path:=' + str(root/'deploy/mid360_bench.json'),
                 '-p', 'xfer_format:=1', '-p', 'publish_freq:=10.0'],
                ['ros2', 'run', 'point_lio', 'pointlio_mapping', '--ros-args',
                 '--params-file', str(root/'deploy/pointlio_mid360_bench.yaml')],
                ['rviz2', '-d', str(root/'deploy/pointlio_bench.rviz')]]
            for name, command in zip(('livox', 'pointlio', 'rviz'), commands):
                log = (output/(name+'.log')).open('w')
                logs.append(log)
                children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                 start_new_session=True))
            last_status = 0
            while not stopped and time.monotonic()-start < args.seconds:
                if (output/'finish').exists() or children[-1].poll() is not None:
                    break
                if any(p.poll() is not None for p in children[:2]):
                    raise RuntimeError('Sensor or localization node exited')
                rclpy.spin_once(node, timeout_sec=.05)
                now = time.monotonic()
                if now-last_status > 1:
                    ready = bool(poses and poses[-1][0]-poses[0][0] >= 8)
                    status = {'ready': ready, 'odometry_count': len(poses),
                              'cloud_count': len(cloud_times), 'elapsed_s': now-start,
                              'latest': poses[-1] if poses else None,
                              'odometry_age_s': now-start-poses[-1][0] if poses else None}
                    (output/'status.json').write_text(json.dumps(status, indent=2))
                    stream.flush()
                    last_status = now
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
                codes.append(child.returncode)
            for log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()
            result = {'odometry_count': len(poses), 'cloud_count': len(cloud_times),
                      'exit_codes_rviz_lio_lidar': codes}
            if poses:
                # Average 2 seconds after the initial settling period; average last 2 seconds.
                base = [p for p in poses if poses[0][0]+6 <= p[0] <= poses[0][0]+8]
                end = [p for p in poses if p[0] >= poses[-1][0]-2]
                if base:
                    a = [sum(p[k] for p in base)/len(base) for k in range(1, 4)]
                    b = [sum(p[k] for p in end)/len(end) for k in range(1, 4)]
                    result.update(start_position_m=a, end_position_m=b,
                                  return_distance_m=math.dist(a, b),
                                  max_excursion_m=max(math.dist(a, p[1:4]) for p in poses))
                    q1, q2 = base[-1][4:8], end[-1][4:8]
                    dot = abs(sum(a*b for a, b in zip(q1, q2)))
                    norm = math.sqrt(sum(v*v for v in q1)*sum(v*v for v in q2))
                    if norm:
                        result['return_orientation_deg'] = math.degrees(2*math.acos(min(1, dot/norm)))
            (output/'result.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
