"""Send an operator command only to the isolated simulation, never motor output."""
import os
import sys
import time
import rclpy
from std_msgs.msg import String

if os.environ.get('ROS_DOMAIN_ID') != '227':
    raise RuntimeError('Isolated simulation domain required')
topic, value = sys.argv[1:]
if topic not in ('/estop/cmd', '/wander/cmd', '/calibration/cmd', '/goal/cmd'):
    raise ValueError('Unsupported operator topic')
rclpy.init()
node = rclpy.create_node('rig_operator')
publisher = node.create_publisher(String, topic, 10)
deadline = time.monotonic()+10.
while not publisher.get_subscription_count() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
if not publisher.get_subscription_count():
    raise RuntimeError('No simulation subscriber')
publisher.publish(String(data=value))
rclpy.spin_once(node, timeout_sec=.5)
node.destroy_node()
rclpy.shutdown()
