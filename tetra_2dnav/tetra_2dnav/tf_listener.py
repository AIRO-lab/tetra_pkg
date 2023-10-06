#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry

rclpy.init()
node = Node('tf_listener_py')

def tf_broad(msg):
  br = TransformBroadcaster()
  # xyz, quaternion?? ?? sendTransform
  # map - base_link
  br.sendTransform((msg.pose.pose.position.x, 
                    msg.pose.pose.position.y, 
                    msg.pose.pose.position.z),
                    [msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w],
                    node.get_clock().now().to_msg(),
                    "","/map")

def main(args=None):
  node.create_subscription(Odometry, 'tf_listener', tf_broad, 10)
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  rclpy.shutdown()
  
if __name__ == '__main__':
  main()