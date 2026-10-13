"""Bare-board ROS2 + USB acceptance; motors disconnected, no fire requests.

Run after sourcing ROS2 and the bridge install. Stop other USB users first.
"""
import argparse
import math
import os
import pathlib
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import yaml


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--usb', required=True)
    cli.add_argument('--vision-config', help='Also launch build_clean/standard with auto_fire=false')
    cli.add_argument('--send-test', action='store_true', required=True,
                     help='Confirm bare-board bench mode / disconnected motors')
    args = cli.parse_args()
    root = pathlib.Path(__file__).resolve().parents[2]
    config = None
    if args.vision_config:
        config = pathlib.Path(args.vision_config).resolve()
        with config.open() as stream:
            if yaml.safe_load(stream).get('auto_fire') is not False:
                cli.error('Vision config must explicitly set auto_fire: false')
    rclpy.init()
    namespace = '/bench_' + str(os.getpid())
    node = rclpy.create_node('board_acceptance', namespace=namespace)
    pub = node.create_publisher(Twist, namespace + '/cmd_vel', 1)
    snapshots = []
    node.create_subscription(Float32MultiArray, namespace + '/serial/link_snapshot',
                             lambda msg: snapshots.append(list(msg.data)), 10)
    proc = None
    vision = None
    try:
        if config:
            vision = subprocess.Popen([str(root / 'build_clean/standard'), str(config)],
                                      cwd=root, start_new_session=True)
        proc = subprocess.Popen([
            'ros2', 'run', 'sentry_cboard_bridge', 'bridge', '--ros-args',
            '-r', '__ns:=' + namespace, '-p', 'device:=' + args.usb,
            '-p', 'cmd_vel_topic:=' + namespace + '/cmd_vel'], start_new_session=True)
        deadline = time.monotonic() + 10
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if pub.get_subscription_count() != 1:
            raise RuntimeError('Expected exactly one bench bridge subscriber')

        deadline = time.monotonic() + (10 if vision else 3)
        while time.monotonic() < deadline:
            if proc.poll() is not None or (vision and vision.poll() is not None):
                raise RuntimeError('Bridge or vision exited before acceptance completed')
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = .2, -.1, .3
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)
        moving = [s for s in snapshots if len(s) == 9 and s[5] == 1 and
                  all(math.isclose(a, b, abs_tol=1e-5)
                      for a, b in zip(s[:3], (.2, -.1, .3)))]
        if not moving:
            raise RuntimeError('CBoard did not echo ROS velocity through USB')
        print('PASS ROS_COMMAND: CBoard snapshot', moving[-1], flush=True)
        if vision:
            both = [s for s in moving if s[6] == 1 and s[8] == 0]
            if not both:
                raise RuntimeError('No simultaneous USB navigation and real vision commands')
            print('PASS BOTH_VALID: CBoard snapshot', both[-1], flush=True)
            tracked = [s for s in both if s[7] == 1]
            print('TRACKING snapshots:', len(tracked), flush=True)
            if tracked:
                print('TRACKING sample:', tracked[-1], flush=True)
            else:
                print('Target tracking NOT verified; check lit armor and enemy_color', flush=True)

        # The bridge keeps USB alive with zeros when cmd_vel expires.
        snapshots.clear()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not snapshots or snapshots[-1][:3] != [0., 0., 0.] or snapshots[-1][5] != 1:
            raise RuntimeError('cmd_vel timeout did not clear speed with USB still valid')
        if any(s[8] != 0 for s in snapshots if len(s) == 9):
            raise RuntimeError('Unexpected shoot request; stop other vision senders')
        print('PASS TOPIC_TIMEOUT: CBoard snapshot', snapshots[-1], flush=True)
        print('PASS ROS2 + real CBoard USB', flush=True)
    finally:
        pub.publish(Twist())
        for child in (vision, proc):
            if child and child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
