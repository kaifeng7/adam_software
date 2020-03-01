#!/usr/bin/env python
# -*- coding=utf-8 -*-

import codecs, json
import sys
import os
import pcl
import numpy as np


def main():
  '''
  usage: python pcd2json.py pcd_path [json_path]\n
  desc: converting pcd file to json file which only containing points array
  '''
  if len(sys.argv) < 2:
    print 'usage: python pcd2json.py pcd_path [json_path]'
    return
    # pcd_path = '/home/zh/workspace/catkin_ws/src/bag_file/1217map/5_full.pcd'
    # json_path = '/home/zh/workspace/catkin_ws/src/bag_file/1217map/5_full.json'
  elif len(sys.argv) == 2:
    try:
      pcd_path = str(sys.argv[1])
      json_path = pcd_path.replace('.pcd', '.json')
    except:
      print 'error'
      return
  elif len(sys.argv == 3):
    pcd_path = str(sys.argv[1])
    json_path = str(sys.argv[2])
  else:
    print 'too many args.'
    return
  pcd = pcl.PointCloud()
  pcd = pcl.load(pcd_path)
  print 'points num: ', pcd.height * pcd.width
  a_pcd = np.asarray(pcd)
  # print a_pcd[:10, :]
  # print a_pcd.shape
  print 'converting ', pcd_path, ' to ', json_path
  json.dump(
      a_pcd.tolist(),
      codecs.open(json_path, 'w', encoding='utf-8'),
      separators=(',', ':'),
      sort_keys=True,
      indent=4)
  pass


if __name__ == "__main__":
  main()
