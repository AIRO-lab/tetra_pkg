#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped

def main(args=None):
  try:
    rclpy.init()
    node = Node("rob_glob_pose")
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    gpose_publisher = node.create_publisher(PoseStamped, "robot_global_pose", 1000)
    rate = node.create_rate(100)

    while rclpy.ok():
      try:
        rclpy.spin_once(node)
        t = node.get_clock().now()
        #listener.waitForTransform('/map', '/imu_frame', t, rospy.Duration(4.0))
        (trans,rot) = tf_buffer.lookup_transform('/map', '/imu_frame', 0)
        
        gpose = PoseStamped()

        gpose.header.stamp = t
        gpose.header.frame_id = "map"

        gpose.pose.position.x = trans[0]
        gpose.pose.position.y = trans[1]
        gpose.pose.position.z = trans[2]

        gpose.pose.orientation.x = rot[0]
        gpose.pose.orientation.y = rot[1]
        gpose.pose.orientation.z = rot[2]
        gpose.pose.orientation.w = rot[3]
        gpose_publisher.publish(gpose)
          
      except TransformException:
        continue

      rate.sleep()

  except Exception:
    pass
    
  rclpy.shutdown()
if __name__ == '__main__':
  main()