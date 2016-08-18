#!/usr/bin/env python
#coding=utf-8
"""
可移动的可制定plan并且根据plan行走的mover

Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import copy
import IMlib
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import maplib
import numpy

class mover_marker():
 def __init__(self):
  self.define()
  if arg=='go':
   self.go_forward()
  rospy.spin()
  
    
 def define(self):
  self.IM_marker = IMlib.obstacles()
  
 def go_forward(self): #testing for go forward
  self.twist = Twist()
  self.cmd = InteractiveMarkerFeedback()
  
  Goal = rospy.wait_for_message('/move_base_simple/goal', PoseStamped)
  self.Gposition = Goal.pose.position
  self.Gangle = maplib.quat_to_angle(Goal.pose.orientation)
  
  self.IM_marker.obstacles()
  rospy.Subscriber(self.root_topic+'/feedback', InteractiveMarkerFeedback, self.RvizFeedback, queue_size=10)

 def RvizFeedback(self, feedback):
  self.Current = feedback.pose
  Cposition = self.Current.position
  self.Cangel = maplib.quat_to_angle(Goal.pose.orientation)
  
  Deviation_x = round(abs(Cposition.x - self.Gposition.x),2)
  Deviation_y = round(abs(Cposition.y - self.Gposition.y),2)
  Deviation_g = round(abs(Cangel - self.Gangle),2)
  
  if Deviation_x < 0.05 and Deviation_y < 0.05:
   self.twist.linear.x = numpy.sqrt(Deviation_x)
   self.twist.linear.y = numpy.sqrt(Deviation_y)
   self.twist.angular.z = numpy.sqrt(Deviation_g)
   
   self.FackRobot(self.twist)#实物用twist代替

 def FackRobot(self, TwistCmd):
  Deviation_q = Quaternion()
  if self.Cangel > 0
  self.Current.position.x += 


 def MoveNE(self):
 
 def MoveNW(self):
 
 def MoveSE(self):
 
 def MoveSW(self):
 
 def MoveN(self):
 
 def MoveS(self):
 
 def MoveE(self):
 
 def MoveW(self):
  
  
  
 
if __name__=='__main__':
 rospy.init_node('test_obstacles')
 try:
  rospy.loginfo ("initialization system")
  mover_marker()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
