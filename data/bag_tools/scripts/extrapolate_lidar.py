# -*- coding=utf-8 -*-
# !/usr/bin/env python

'''
1. https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/scripts/pcl_helper.py
2. 
'''

import sys

import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2

import pcl

class Extrapolate(object):
  def __init__(self):
    self.scan_id = rospy.get_param("scan_id")
    self.topics = rospy.get_param("topics")
    self.topic_lidar = rospy.get_param("topic_lidar")
    self.bag_name_i = rospy.get_param("bag_name_i")
    self.bag_name_o = rospy.get_param("bag_name_o")

  def extrapolate(self):
    with rosbag.Bag(self.bag_name_i, 'r') as bag_in:
      with rosbag.Bag(self.bag_name_o, 'w') as bag_out:
        for topic, msg, t in bag_in.read_messages():
          if topic == self.topic_lidar:
            points_list = []
            for data in pc2.read_points(msg, skip_nans=True):
              points_list.append([data[0], data[1], data[2], data[3]])

            pcl_data = pcl.PointCloud_PointXYZ()
            pcl_data.from_list(points_list)
            pass
          else:
            bag_out.write(topic, msg, t)

def main():
  pass

if __name__ == "__main__":
  rospy.init_node("extrapolate_lidar")
  main()
  rospy.spin()