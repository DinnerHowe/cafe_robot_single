#!/usr/bin/env python
#coding=utf-8
"""
rviz 地图编辑软件测试算法用

Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 
"""
import numpy as np
import rospy,yaml


class grid_map():
 def __init__(self):
  self.define()
  (width,height,maxval,GridMap)=self.ReadPGMMap()


 def define(self):
  self.filename='/home/howe/cafe_robot_single/src/nav_staff/map/'
  
 def ReadPGMMap(self):
  (image,resolution,origin)=  self.ReadYaml()
  with open(self.filename+image, 'rb') as f:
   MapData=f.read()
  MapData=MapData.split('\n')
  header=MapData[0]
  width=MapData[1]
  height=MapData[2]
  maxval=MapData[3]
  GridMap=MapData[4]
  return (width,height,maxval,GridMap)
   
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
