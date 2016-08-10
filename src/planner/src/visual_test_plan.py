#!/usr/bin/env python
#coding=utf-8
"""
视觉显示路径

Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 
"""
import rospy,time
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose

class VisualTestPlan():
 def __init__(self):
  self.define()
  rospy.Subscriber('%s'%self.robot_position_topic,Pose,self.pose_cb)
  rospy.Timer(rospy.Duration(2), self.timer_cb)
  rospy.spin()
  
 def define(self):
  self.CurrentPose=Pose()
 
  if not rospy.has_param('~robot_position_topic'):
   rospy.set_param('~robot_position_topic','turtlebot_position_in_map')
  self.robot_position_topic = rospy.get_param('~robot_position_topic')
  
 def self.pose_cb(self, data):
  self.self.CurrentPose=data
 
 def timer_cb(self, even):
  self.new_goal=rospy.wait_for_message('/clicked_point',PointStamped)
  rospy.loginfo('Planning a new path')
  self.planner()
  if rospy.is_shutdown():
   rospy.signal_shutdown('shutdown')
 
 
 def planner(self, Type):

  #dijkstra 算法
  if Type='dijkstra':
   TimerStart=time.clock()
   
   PlanResult=self.dijkstra(_map, init, goal)
   
   TimerEnd=time.clock()
   print 'dijkstra planner spend time: ', TimerStart-TimerEnd
  
  #Astar 算法
  if Type='Astar':
   TimerStart=time.clock()
   
   PlanResult=self.Astar(_map, init, goal)
   
   TimerEnd=time.clock()
   print 'dijkstra planner spend time: ', TimerStart-TimerEnd

  #Jump Point Search 算法
  if Type='JPS'
   TimerStart=time.clock()
   
   PlanResult=self.JPS(_map, init, goal)
   
   TimerEnd=time.clock()
   print 'dijkstra planner spend time: ', TimerStart-TimerEnd
   

 ################### 算法 ##############################
   #dijkstra 算法
 def dijkstra(self, _map, init, goal):
 
  return Result
  
  
   #dijkstra 算法
 def Astar(self, _map, init, goal):
 
  return Result


   #dijkstra 算法 
 def JPS(self, _map, init, goal):

  return Result
   
 ###################visual_test#########################
 def visual_test(self,data,Type,color,scale):#data=[point1,point2,point3...]

  #plot POINTS
  #print len(data),data[0],data[1]
  if Type==Marker.POINTS:
   #print 'pub POINTS Marker'
   point_marker=Marker()
   point_marker.header.frame_id='/map'
   point_marker.header.stamp=rospy.Time.now()
   point_marker.ns='KeyPoint'
   point_marker.action=Marker.ADD
   
   point_marker.id=0
   point_marker.type=Type
   point_marker.scale.x=scale.x#0.1
   point_marker.scale.y=scale.y#0.1
   
   point_marker.points=data
   for i in data:
    point_marker.colors.append(color)

   point_marker.lifetime=rospy.Duration(0.2)
   
   return point_marker
   
   
  if Type==Marker.LINE_LIST:
   #print 'pub LINE_LIST Marker'
   line_marker=Marker()
   line_marker.header.frame_id='/map'
   line_marker.header.stamp=rospy.Time.now()
   line_marker.ns='Pathes'
   line_marker.action=Marker.ADD
   
   line_marker.id=1
   line_marker.type=Type
   line_marker.scale.x=scale.x#0.05
   line_marker.scale.y=scale.y#0.05  
   
   line_marker.points=data
   for i in data:
    line_marker.colors.append(color)

   line_marker.lifetime=rospy.Duration(0.5)
   
   return line_marker
 
if __name__=='__main__':
 rospy.init_node('VisualTestPlan')
 try:
  rospy.loginfo ("initialization system")
  VisualTestPlan()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
