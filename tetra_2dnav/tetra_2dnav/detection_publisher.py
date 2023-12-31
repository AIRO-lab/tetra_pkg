#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tetra_msgs.srv import GetObjectLocation, GetObjectLocationResponse
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Point

import argparse
from yaml import load
from os.path import expanduser

class DetectionPublisher(Node):
  def __init__(self, filename):
    super.__init__("detection_publisher")
    with open(filename, 'r') as infile:
      self.detections = load(infile)
    self.marker_id = 0
    self.rate = self.create_rate(1)
    self.map_objects = self.make_marker_array(self.detections)
    self.map_publisher = self.create_publisher(MarkerArray, '~map_objects', 10)
    self.map_server = self.create_subscription(GetObjectLocation, '~object_location', self.object_location, 10)
    self.process()

  def make_marker(self, name, x, y, z):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = self.get_clock().now()

    marker.ns = "basic_shapes"
    marker.type = 9 # text
    marker.action = Marker.ADD
    marker.id = self.marker_id
    self.marker_id += 1

    marker.text = name
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z+0.3 # Text appears slightly above the object
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.scale.z = 0.3

    return marker

  def make_marker_array(self, detections):
    markers = []
    for name, pos in detections.iteritems():
      for p in pos:
        marker = self.make_marker(name, *p)
        markers.append(marker)
    return MarkerArray(markers)

  def object_location(self, req):
    pose_array = PoseArray()
    pose_array.header.frame_id = "map"
    pose_array.header.stamp = self.get_clock().now()

    if self.detections.has_key(req.object_name):
      poses = []
      for point in self.detections[req.object_name]:
        pose = Pose()
        pose.position = Point(*point)
        poses.append(pose)
      pose_array.poses = poses
    return GetObjectLocationResponse(pose_array)

  def process(self):
    print("Starting detection publisher")
    while rclpy.ok():
      rclpy.spin_once(self)
      self.map_publisher.publish(self.map_objects)
      self.rate.sleep()

def main(args=None):
  # Argument Parser
  parser = argparse.ArgumentParser()
  parser.add_argument('-i', '--input_file', type=str, default='~/.ros/detections_dbscan.db')
  args, _ = parser.parse_known_args()

  # Init
  rclpy.init()
  DetectionPublisher(expanduser(args.input_file))
  
  rclpy.shutdown()



if __name__ == '__main__':
  main()