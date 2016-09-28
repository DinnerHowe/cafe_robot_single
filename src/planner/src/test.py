#!/usr/bin/env python
#coding=utf-8
"""
用来测试代码片段的文件
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import copy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class STATUSSPACE():
 PENDING = 0     # 因为任务插入，目标点暂时挂起，等待插入任务完成后再执行
 ACTIVE = 1      # 目标点正在被执行
 SUCCEEDED = 3   # 目标点已到达
 ABORTED = 4     # 因为某些原因，目标点在执行过程中无法到达

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

class map_check():
 def __init__(self):
  rospy.Subscriber('/map', OccupancyGrid, self.MapCB)
  rospy.spin()
  
 def MapCB(self, data):
  print data.info
  
  
  
class GoalManager():
 def __init__(self):
  self.define()
  print self.ControlBase
  rospy.Subscriber('/%s/goal'%self.ControlBase, MoveBaseGoal, self.Goal)
  rospy.Subscriber('/%s/cancle'%self.ControlBase, GoalStatus, self.cancle)
  rospy.Timer(rospy.Duration(0.2), self.GoalUpdate) 
  rospy.spin()
  
 def define(self):
  self.ControlBase = rospy.get_param('/GoalManage/ControlBase')
  self.pubdata = GoalStatus()
  self.pubdata.status = STATUSSPACE.SUCCEEDED
  self.pubdata.goal_id.id = 'planner_test_node/' + '%s'%None
  self.GoalStatus = rospy.Publisher('/%s/goal_status'%self.ControlBase, GoalStatus, queue_size = 1)

  
 def GoalUpdate(self, event):
  self.pubdata.goal_id.stamp = rospy.Time.now()
  self.GoalStatus.publish(self.pubdata)

  
 def cancle(self, data):
  print '\ncancle request: \n', data
  
  
 def Goal(self, data):
  print '\ngoal:\n', data
  status = raw_input('请输入goal状态（1：ACTIVE， 2：SUCCEEDED， 3. ABORTED， 4：PENDING）')
  
  if status == '1':
   self.pubdata.status = STATUSSPACE.ACTIVE
  if status == '2':
   self.pubdata.status = STATUSSPACE.SUCCEEDED
  if status == '3':
   self.pubdata.status = STATUSSPACE.ABORTED
  if status == '4':
   self.pubdata.status = STATUSSPACE.PENDING
   
  self.pubdata.goal_id.id = 'planner_test_node/' + '%s'%data.target_pose.header.seq
  self.pubdata.goal_id.stamp = rospy.Time.now()
   
  self.GoalStatus = rospy.Publisher('/%s/goal_status'%self.ControlBase, GoalStatus, queue_size = 1)
  self.GoalStatus.publish(self.pubdata)
 
if __name__=='__main__':
 rospy.init_node('planner_test_node')
 rospy.loginfo ("initialization system")
 #test()
 #map_check()
 GoalManager()
 rospy.loginfo ("process done and quit")

