"""Drive only the isolated dashboard test fixture, never production topics."""
import os
import sys
import time
assert os.environ.get('ROS_DOMAIN_ID') == '231'
import rclpy
from std_msgs.msg import String
rclpy.init()
node = rclpy.create_node('dashboard_scenario')
pub = node.create_publisher(String, '/dashboard_test/scenario', 10)
deadline = time.monotonic()+5
while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
assert pub.get_subscription_count() > 0
for _ in range(3):
    pub.publish(String(data=sys.argv[1]))
    rclpy.spin_once(node, timeout_sec=.1)
node.destroy_node()
rclpy.shutdown()
