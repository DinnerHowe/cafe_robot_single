#!/usr/bin/env python
#coding=utf-8
"""
rviz 地图编辑软件测试算法用

Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 
"""
import numpy as np
import rospy,yaml,numpy,Image
from nav_msgs.msg import OccupancyGrid 

class grid_map():
 def __init__(self):
  self.define()
  Map=self.ReadPGMMap()
  while not rospy.is_shutdown():
   self.test_map.publish(Map)
   
 def define(self):
  self.filename='/home/howe/cafe_robot_single/src/nav_staff/map/'
  self.test_map=rospy.Publisher("test_map", OccupancyGrid ,queue_size=1)
  
 def ReadPGMMap(self):
  (image,resolution,origin)=  self.ReadYaml()
   
  f=Image.open(self.filename+image)
  (width,height)=f.size
  GridMap=numpy.array(f)
  
  Map=OccupancyGrid()
  Map.header.frame_id='map'
  Map.info.width=width
  Map.info.height=height
  Map.info.resolution=resolution
  Map.info.origin.position.x=origin[0]
  Map.info.origin.position.y=origin[1]
  Map.info.origin.position.z=origin[2]
  Map.info.origin.orientation.w=1
  #print 'height',height,'width',width
  for i in range(height):
   data=GridMap[height-i-1]
   for j in data:
    res=j==[0,0,0]
    if res.all():
     Map.data.append(100)
    else:
     Map.data.append(0)
  #print Map.info
  return Map
   
 def ReadYaml(self):
  with open(self.filename+'office_map_manual.yaml', 'rb') as f:
   data=yaml.load(f)
  image=data['image']
  resolution=data['resolution']
  origin=data['origin']
  return (image,resolution,origin)
  
   
if __name__=='__main__':
 rospy.init_node('grid_map')
 try:
  rospy.loginfo ("initialization system")
  grid_map()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
