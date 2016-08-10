#!/usr/bin/env python
#coding=utf-8
"""
分拆path

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""
import rospy,os,sys
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class path_splitter():
 def __init__(self):
  self.define()
  path=rospy.wait_for_message('%s'%self.plan_topic, Path)
  self.analyser(path)

 def define(self):
  self.path_data=[]
  
  if not rospy.has_param('~plan_topic'):
   rospy.set_param('~plan_topic','/move_base/TrajectoryPlannerROS/global_plan')
  self.plan_topic = rospy.get_param('~plan_topic')

  
 def analyser(self, data):
  self.path_data=data.poses
  self.store(self.path_data)
  path_point=Point
  point_list=[]
  for i in self.path_data:
   path_point=i.pose.position
   point_list.append(path_point)
  self.marker(point_list)
  marker_pub=rospy.Publisher("detector_marker", Marker ,queue_size=1)
  while not rospy.is_shutdown():
   marker_pub.publish(self.point_marker)
 
 def store(self, data):
  print len(data)
  print self.cf_dir()  
  f=open('%s/path.txt'%self.cf_dir(), 'w')
  f.writelines('%s'%data)
  f.close()

   
 def cf_dir(self,c_path=''):
  c_path=sys.path[0]
  if os.path.isdir(c_path):
   return c_path
  elif os.path.isfile(c_path):
   return os.path.dirname(c_path)
 
 def marker(self,data):
  self.point_marker=Marker()
  color=ColorRGBA()
  
  color.r=1.0
  color.a=1
  
  self.point_marker.header.frame_id='/map'
  self.point_marker.header.stamp=rospy.Time.now()
  self.point_marker.ns='path_test'
  self.point_marker.action=Marker.ADD
  self.point_marker.id=0
  self.point_marker.type=Marker.POINTS
  self.point_marker.scale.x=0.05
  self.point_marker.scale.y=0.05
  
  self.point_marker.points=data
  
  for i in data:
   self.point_marker.colors.append(color)

  self.point_marker.lifetime=rospy.Duration(0.2)
   
 
if __name__=='__main__':
 try:
  rospy.init_node('path_splitter')
  rospy.loginfo ("initialization system")
  path_splitter()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
