#!/usr/bin/env python
#coding=utf-8
"""
用来测试代码片段的文件
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import copy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

class test():
 def __init__(self):
  position1 = Point()
  position1.x = 1
  position2 = Point()
  position2.x = 2
  a = [position1, position2]
  position1 = Point()
  position1.x = 1
  b = [position1]

  print set(a).intersection(set(b))
  

class NewPoint(Point):
 #def __hash__(self):
 def __eq__(self, other):
  return self.x == other.x and self.y == other.y and self.z == other.z

class map_check():
 def __init__(self):
  rospy.Subscriber('/map', OccupancyGrid, self.MapCB)
  rospy.spin()
  
 def MapCB(self, data):
  print data.info
 
if __name__=='__main__':
 rospy.init_node('planner_test_node')
 rospy.loginfo ("initialization system")
 #test()
 map_check()
 rospy.loginfo ("process done and quit")

