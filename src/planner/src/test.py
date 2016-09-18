#!/usr/bin/env python
#coding=utf-8
"""
用来测试代码片段的文件
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy,copy
from geometry_msgs.msg import Point


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
   
if __name__=='__main__':
 try:
  rospy.loginfo ("initialization system")
  test()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
