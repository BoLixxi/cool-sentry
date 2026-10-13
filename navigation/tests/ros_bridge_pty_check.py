"""Run after colcon build + source install; real ROS2 node over a virtual USB port."""
import os
import pathlib
import pty
import select
import signal
import struct
import subprocess
import sys
import time
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'ros2/sentry_cboard_bridge'))
from sentry_cboard_bridge.wire import Parser,pack
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

master,slave=pty.openpty()
proc=None
rclpy.init()
node=rclpy.create_node('bridge_pty_test')
pub=node.create_publisher(Twist,'/test_nav_cmd',1)
feedback=[]
node.create_subscription(Float32MultiArray,'/serial/link_snapshot',lambda m:feedback.append(list(m.data)),10)
parser=Parser()
try:
    proc=subprocess.Popen(['ros2','run','sentry_cboard_bridge','bridge','--ros-args',
                           '-p','device:='+os.ttyname(slave),'-p','cmd_vel_topic:=/test_nav_cmd'],
                          stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+8
    while pub.get_subscription_count()==0 and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    assert pub.get_subscription_count()==1,'bridge subscription missing'
    frames=[]; deadline=time.monotonic()+1
    while time.monotonic()<deadline:
        msg=Twist(); msg.linear.x=.2; msg.linear.y=-.1; msg.angular.z=.3; pub.publish(msg)
        if select.select([master],[],[],.01)[0]: frames.extend(parser.feed(os.read(master,4096)))
        diagnostic=struct.pack('<5f4B',.2,-.1,.3,.25,-.5,1,1,1,0)
        os.write(master,pack(0xe0,123,diagnostic))
        rclpy.spin_once(node,timeout_sec=.01)
    assert any(abs(struct.unpack('<3f',f[2][:12])[0]-.2)<1e-5 for f in frames),'nonzero cmd missing'
    assert all(f[0]==1 and len(f[2])==39 and f[2][12:]==bytes(27) for f in frames)
    assert feedback and feedback[-1][5:]==[1,1,1,0],'diagnostic ROS feedback missing'
    frames=[]; deadline=time.monotonic()+.35
    while time.monotonic()<deadline:
        if select.select([master],[],[],.01)[0]: frames.extend(parser.feed(os.read(master,4096)))
        rclpy.spin_once(node,timeout_sec=.01)
    assert frames and struct.unpack('<3f',frames[-1][2][:12])==(0,0,0),'stale cmd not zeroed'
    os.killpg(proc.pid,signal.SIGINT); output=proc.communicate(timeout=5)[0].decode()
    assert proc.returncode==0,output
    print('PASS ROS2 + PTY: cmd_vel -> 49-byte USB frame, feedback -> ROS, topic timeout, clean shutdown')
finally:
    if proc and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGKILL); proc.wait()
    node.destroy_node(); rclpy.shutdown(); os.close(master); os.close(slave)
