#!/usr/bin/env python
#coding:utf-8
"""
提示报警字幕，触发停止功能
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""

import rospy,actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker


class stopmove():

 def __init__(self):
  self.define()
  rospy.loginfo("waiting fro move_base action server")
  self.stop_base.wait_for_server(rospy.Duration(60))
  rospy.Subscriber("%s"%self.stop_flag_topic, String, self.stop_cb)
  #rospy.Subscriber("turtlebot_position_in_map", Pose, self.pose_cb)
  rospy.spin()

 def stop_cb(self, data):
  if self.info!=data.data:
   self.info=data.data
   self.stop_count=0
  else:
   self.stop_count+=1
  #print data.data, self.stop_count
  if self.info=="stop" and self.stop_count>5:
   self.stopmoveit()
   self.addFlag()
  else:
   pass
  
 #def self.pose_cb(self,data):
  #self.pose=data

 def define(self):
  rospy.init_node("stopmove", anonymous=True)
  
  #params
  self.stop_count=0
  self.info=''
  self.pose=Pose()
  self.marker=Marker()
  self.stop=Twist()
  
  self.stop_flag_topic=''
  self.warning_marker_topic=''
  self.StopMess_topic=''
  self.action_topic=''
  self.turtlebot_position_topic=''
  
  if not rospy.has_param('~stop_flag_topic'):
   rospy.set_param('~stop_flag_topic','/stop_flag')
  self.stop_flag_topic = rospy.get_param('~stop_flag_topic')
   
  if not rospy.has_param('~warning_marker_topic'):
   rospy.set_param('~warning_marker_topic','/warning_marker')
  self.warning_marker_topic = rospy.get_param('~warning_marker_topic')
  
  if not rospy.has_param('~StopMoving_topic'):
   rospy.set_param('~StopMoving_topic','/cmd_vel_mux/input/teleop')
  self.StopMess_topic=rospy.get_param('~StopMoving_topic')
  
  if not rospy.has_param('~action_topic'):
   rospy.set_param('~action_topic','move_base')
  self.action_topic=rospy.get_param('~action_topic')
  
  if not rospy.has_param('~turtlebot_position_topic'):
   rospy.set('~turtlebot_position_topic','turtlebot_position_in_map')
  self.turtlebot_position_topic=rospy.get_param('~turtlebot_position_topic')
  
  #functions
  self.marker_pub = rospy.Publisher("%s"%self.warning_marker_topic, Marker ,queue_size=1)
  self.pubStopMess = rospy.Publisher('%s'%self.StopMess_topic, Twist, queue_size=1)
  self.stop_base = actionlib.SimpleActionClient("%s"%self.action_topic, MoveBaseAction)
  self.stop_server= actionlib.SimpleActionServer('%s'%self.action_topic, MoveBaseAction, False)
  self.recieve_pub=rospy.Publisher("detector_recieved", String ,queue_size=1)
  
  
  #details 
  self.marker.type = Marker.TEXT_VIEW_FACING
  self.marker.header.frame_id='map'
  self.marker.color.r, self.marker.color.b, self.marker.color.g, self.marker.color.a= 1.0, 0.0, 0.0, 1.0
  self.marker.text = "WARNING!!!"
  self.marker.ns="WarningFlag"
  #self.marker.id = 1024
  
 def addFlag(self):
  self.pose = rospy.wait_for_message("%s"%self.turtlebot_position_topic, Pose)
  self.marker.pose = self.pose
  self.marker.pose.position.z= self.pose.position.z +0.5
  self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = 0.01, 0.01, 0.2
  self.marker.lifetime = rospy.Duration(0.5)
  self.marker_pub.publish(self.marker)


 def stopmoveit(self):   
  self.recieve_pub.publish('recieved')
  rospy.loginfo("cancelling goal") 
  if self.stop_server.is_active():
   self.stop_server.set_aborted()
   self.stop_base.cancle_goal()
  else:
   pass
  self.pubStopMess.publish(self.stop)

if __name__ == '__main__':
 try:
  rospy.loginfo ("initialization system")
  stopmove()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")
