#!/usr/bin/env python
#coding=utf-8
"""
用来现实记录path和plan
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy,copy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA

###############################
######### Path View ###########
###############################

class PathRecording():
 def __init__(self):
  root_topic = 'test_obstacles'
  
  self.define(root_topic)
  
  rospy.Subscriber(self.root_topic+'/feedback', InteractiveMarkerFeedback, self.RvizFeedback, queue_size=10)
  
  rospy.Timer(rospy.Duration(0.5), self.Reset)
  
  rospy.spin()
  
 def define(self,root_topic):
  #self.locker = Lock()
  self.pose = PoseStamped()
  self.seq_num = 0
  self.frame_id = '/map'
  self.root_topic = '/' + root_topic
  
  self.path_marker = rospy.Publisher(self.root_topic+"/path", Marker, queue_size=1)
  self.path_pub = rospy.Publisher(self.root_topic+"/plan", Path, queue_size=1)

  self.path = Path()
  #self.path.header.seq = self.seq_num
  self.path.header.frame_id = self.frame_id
 
  self.path_mark = Marker()
  #self.path_mark.header.seq = self.seq_num
  self.path_mark.header.frame_id = self.frame_id
  self.path_mark.ns = self.root_topic
  self.path_mark.type = Marker.POINTS
  self.path_mark.action = Marker.ADD
  self.path_mark.lifetime = rospy.Duration(1)
  self.path_mark.frame_locked = 1
  self.path_mark.scale.x=0.1
  self.path_mark.scale.y=0.1
  self.path_mark.scale.z=0.1
  
  self.color = ColorRGBA()
  self.color.r = 1
  self.color.a = 1
  
 def InsertPath(self, pose):
  #with self.locker:
   self.pose.pose = pose
   self.pose.header.frame_id = self.frame_id
   self.pose.header.stamp = rospy.Time.now()
   self.pose.header.seq = self.seq_num
  
   self.path.header.stamp = rospy.Time.now()
   self.path.header.seq = self.seq_num
   self.path.poses.append(copy.deepcopy(self.pose))

   self.path_mark.header.stamp = rospy.Time.now()
   self.path_mark.header.seq = self.seq_num  
   self.path_mark.points.append(copy.deepcopy(pose.position))
   self.path_mark.colors.append(copy.deepcopy(self.color))
  
 def start(self):
  #with self.locker:
   self.path_marker.publish(self.path_mark)
   self.path_pub.publish(self.path)
   self.seq_num += 1
  
 def Reset(self, event):
  #with self.locker:
   dur = rospy.Duration(6)
   rospy.sleep(dur)
   InitData = Path()
   InitData.header.seq = self.seq_num
   InitData.header.stamp = rospy.Time.now()
   InitData.header.frame_id = self.frame_id
   self.path.poses=[]
      
   Initmark = Marker()
   Initmark.header.frame_id = self.frame_id
   Initmark.ns = self.root_topic
   Initmark.type = Marker.POINTS
   Initmark.action = Marker.ADD
   Initmark.lifetime = rospy.Duration(1)
   Initmark.frame_locked = 1
   self.path_mark.points=[]
   self.path_mark.colors=[]
  
   self.path_pub.publish(InitData)
   self.path_marker.publish(Initmark)
  
 def RvizFeedback(self, feedback):
  #with self.locker: 
   pose = Pose()
   pose=feedback.pose
   rospy.loginfo('\ncurrent position: ' + '%s'%pose)
   self.InsertPath(pose)
   self.start()
   
###############################
#########    main    ##########
###############################
   
if __name__=='__main__':
 rospy.init_node('test_obstacles')
 try:
  rospy.loginfo ("initialization system")
  PathRecording()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
