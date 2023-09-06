#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose

from detection_clustering import DetectionClustering

from yaml import dump

class DetectionCollector(Node):
    def __init__(self):
      super.__init__('detection_collector')
      self.declare_parameter('/darknet_ros/yolo_model/detection_classes/names', ["", ])
      self.detected = {}
      self.detection_names = self.get_parameter('/darknet_ros/yolo_model/detection_classes/names').get_parameter_value().string_array_value
      self.create_subscription(PoseArray, '/cluster_decomposer/centroid_pose_array', self.collect, 10)
      print('Searching for objects...')

    def __del__(self):
      print("Writing to detections_raw.db...")
      with open('detections_raw.db', 'w') as outfile:
        dump(self.detected, outfile)

      print("Writing to detections_dbscan.db...")
      dc = DetectionClustering(self.detected)
      with open('detections_dbscan.db', 'w') as outfile:
        dump(dc.clusters, outfile)

    def update_key(self, key, val):
      if self.detected.has_key(key):
        self.detected[key].append(val)
      else:
        self.detected[key] = [val]

    def collect(self, msg):
      for i, pose in enumerate(msg.poses):
        if pose != Pose():
          pos = pose.position
          val = [pos.x, pos.y, pos.z]
          key = self.detection_names[i]
          self.update_key(key, val)

if __name__ == '__main__':
  rclpy.init()
  node = DetectionCollector()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  rclpy.shutdown()
