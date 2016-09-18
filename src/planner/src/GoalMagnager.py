#!/usr/bin/env python
#coding=utf-8
""" 
goal manager

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
from move_base_msgs.msg import MoveBaseGoal

class GoalManage():
 def __init__(self):
  self.define()
  rospy.Subscriber('/turtlebot_position_in_map', Pose, self.OdomCB)
  
  
 def define(self):
  Goals = []
  Goal = MoveBaseGoal
  
if __name__ == '__main__':
 rospy.init_node('GoalManage')
 try:
  rospy.loginfo( "initialization system")
  GoalManage()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")
