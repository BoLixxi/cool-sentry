import math
import struct
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from .wire import Parser, VelocityGate, command


class Bridge(Node):
    def __init__(self):
        super().__init__('sentry_cboard_bridge')
        defaults = {'device': '/dev/rm_nav', 'cmd_vel_topic': '/cmd_vel',
                    'cmd_timeout_ms': 100, 'max_linear': 2.5, 'max_angular': 3.0,
                    'imu_frame': 'cboard_imu'}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.device = p('device')
        self.imu_frame = p('imu_frame')
        self.gate = VelocityGate(p('cmd_timeout_ms') / 1000.0, p('max_linear'), p('max_angular'))
        self.port = None
        self.retry_at = 0.0
        self.start = time.monotonic()
        self.parser = Parser()
        self.create_subscription(Twist, p('cmd_vel_topic'), self.on_velocity, 1)
        self.motion_pub = self.create_publisher(Twist, 'serial/robot_motion', 10)
        # Raw board IMU only: mounting TF must be supplied by robot configuration.
        self.imu_pub = self.create_publisher(Imu, 'serial/cboard_imu', 10)
        self.link_pub = self.create_publisher(Float32MultiArray, 'serial/link_snapshot', 10)
        self.create_timer(0.01, self.tick)

    def on_velocity(self, msg):
        if not self.gate.update((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic()):
            self.get_logger().error('Non-finite cmd_vel rejected; velocity cleared')

    def disconnect(self):
        if self.port:
            self.port.close()
        self.port = None
        self.gate.reset()
        self.retry_at = time.monotonic() + 1.0

    def tick(self):
        now = time.monotonic()
        if not self.port:
            if now < self.retry_at:
                return
            try:
                self.port = serial.Serial(self.device, 115200, timeout=0, write_timeout=0.02, exclusive=True)
                self.port.reset_input_buffer()
                self.parser = Parser()
                self.gate.reset()  # Require a new cmd_vel after reconnect.
                self.get_logger().info(f'Connected {self.device}; USB navigation only')
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f'USB not ready: {exc}')
                self.disconnect()
                return
        try:
            frame = command(int((now - self.start) * 1000), *self.gate.get(now))
            if self.port.write(frame) != len(frame):
                raise serial.SerialException('short USB write')
            for kind, tick, payload in self.parser.feed(self.port.read(min(self.port.in_waiting, 4096))):
                self.publish_frame(kind, tick, payload)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'USB disconnected: {exc}')
            self.disconnect()

    def publish_frame(self, kind, tick, payload):
        if kind == 0x1e and len(payload) == 12:
            values = struct.unpack('<3f', payload)
            if not all(math.isfinite(v) for v in values):
                return
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = values
            self.motion_pub.publish(msg)
        elif kind == 2 and len(payload) == 24:
            yaw, pitch, roll, wyaw, wpitch, wroll = struct.unpack('<6f', payload)
            if not all(math.isfinite(v) for v in (yaw, pitch, roll, wyaw, wpitch, wroll)):
                return
            cy, sy = math.cos(yaw/2), math.sin(yaw/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.imu_frame
            msg.orientation.w = cr*cp*cy + sr*sp*sy
            msg.orientation.x = sr*cp*cy - cr*sp*sy
            msg.orientation.y = cr*sp*cy + sr*cp*sy
            msg.orientation.z = cr*cp*sy - sr*sp*cy
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = wroll, wpitch, wyaw
            msg.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(msg)
        elif kind == 0xe0 and len(payload) == 24:
            msg = Float32MultiArray()
            msg.data = list(struct.unpack('<5f', payload[:20])) + [float(v) for v in payload[20:]]
            self.link_pub.publish(msg)

    def stop(self):
        if self.port:
            try:
                self.port.write(command(0, 0, 0, 0))
            except (serial.SerialException, OSError):
                pass
        self.disconnect()


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
