#!/usr/bin/env python
#coding=utf-8
""" 
用来测试plan的程序

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
from nav_msgs.msg import Path,Odometry 
 
class planmoniter():
 def __init__(self):
  rospy.Subscriber('/move_base/DWAPlannerROS/global_plan',Path, self.path_callback)
  rospy.spin()
 def path_callback(self, path):
  print len(path.poses)
  
if __name__ == '__main__':
 rospy.init_node('planmoniter')
 try:
  rospy.loginfo( "initialization system")
  planmoniter()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")
