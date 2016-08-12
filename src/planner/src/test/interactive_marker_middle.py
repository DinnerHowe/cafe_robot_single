#!/usr/bin/env python
#coding=utf-8
"""
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server
"""
import rospy,copy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarkerControl

from geometry_msgs.msg import PointStamped
from interactive_marker_server import *

class obstacles():
 def __init__(self):
  root_topic = self.define()
  server = InteractiveMarkerServer(root_topic)  ########## server 
  position =  rospy.wait_for_message('/clicked_point', PointStamped)
  self.marker(root_topic,position.point,server)
  print 'loading object'  
  print 'applyChanges'
  server.applyChanges()  ########## server ####################
  rospy.spin()
  
 #initial define 
 def define(self):
  self.obstacle=InteractiveMarker()
  self.unit=Marker()
  self.control=InteractiveMarkerControl()
  root_topic = 'test_obstacles'
  return root_topic

 #the moving object  
 def marker(self,root_topic,position,server):
  print '3'
  self.obstacle.header.frame_id='map'
  self.obstacle.name='obstacles'
  self.obstacle.description='test_marker'
  self.obstacle.pose.position=position
  self.obstacle.pose.position.z = 0.0
  self.obstacle.scale=1/2
  
  self.unit.type = Marker.CUBE
  self.unit.scale.x = 0.45
  self.unit.scale.y = 0.45
  self.unit.scale.z = 0.45
  self.unit.color.r = 1.0
  self.unit.color.g = 1.0
  self.unit.color.b = 0.5
  self.unit.color.a = 1.0
  
  self.obstacle.pose.position.z += self.unit.scale.z/2
  
  self.control.orientation.w = 1
  self.control.orientation.y = 1
  self.control.interaction_mode= InteractiveMarkerControl.MOVE_PLANE
  self.control.always_visible=True
  
  self.control.markers.append(copy.deepcopy(self.unit))
  
  self.obstacle.controls.append(copy.deepcopy(self.control))

  server.insert(self.obstacle, self.processFeedback)   ########## server ##

   
  
  ########## server ####################
 def processFeedback(self,data):
  #print '######################feedback data\n',data,'\n'
  pass
  ########## server ####################
 
if __name__=='__main__':
 rospy.init_node('test_obstacles')
 try:
  rospy.loginfo ("initialization system")
  obstacles()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
