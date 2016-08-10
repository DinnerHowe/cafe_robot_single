#!/usr/bin/env python
#coding=utf-8
"""
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server
"""
import rospy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerInit
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate
from std_msgs.msg import Header
  
class obstacles():
 def __init__(self):
  self.define()
  self.marker()
  
  self.Subscriber(self.root_topic+'/feedback', InteractiveMarkerFeedback, queue_size=1)
  rospy.timer(rospy.Duration(0.5), self.timercb)
  
  
 def marker(self):
  self.obstacle.header.frame_id='map'
  self.obstacle.name='obstacles'
  self.obstacle.description('test_marker')
  
  self.unit.type=Marker.CUBE
  self.unit.scale.x = 0.45
  self.unit.scale.y = 0.45
  self.unit.scale.z = 0.45
  self.unit.color.r = 0.5
  self.unit.color.r = 1.0
  
  self.control.orientation.w = 1
  self.control.orientation.y = 1
  self.control.always_visible=True
  self.control.interaction_mode= InteractiveMarkerControl.MOVE_PLANE
  self.control.markers.append(self.unit)
  
  self.obstacle.controls.append(self.control)
  self.
  
 def define(self):
 
  self.obstacle=InteractiveMarker()
  self.unit=Marker()
  self.control=InteractiveMarkerControl()
  self.update_=rospy.Publisher(self.root_topic+"/Update", InteractiveMarkerUpdate ,queue_size=1)
  self.init_=rospy.Publisher(self.root_topic+'/Init', InteractiveMarkerInit, queue_size=1)
  self.seq_num=0
  self.server_id = self.root_topic + '%s'%(rospy.Timer.now) 
  
  

  
 def timercb(self, event):
  standby=InteractiveMarkerUpdate()
  standby.type = InteractiveMarkerUpdate.KEEP_ALIVE
  standby.server_id  = self.server_id
  standby.seq_num = self.seq_num
  self.update_.publish(standby)
  
if __name__=='__main__':
 rospy.init_node('test_obstacles')
 try:
  rospy.loginfo ("initialization system")
  obstacles()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
