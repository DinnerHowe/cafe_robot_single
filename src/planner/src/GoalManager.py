#!/usr/bin/env python
#coding=utf-8
""" 
goal manager:管理goal并且给planner发送goal

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
import copy
import numpy
from move_base_msgs.msg import MoveBaseGoal
from threading import Lock
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose

""" 
goal_id status 
ACTIVE = 0   # 目标点正在被执行
SUCCEEDED = 1   # 目标点已到达
ABORTED = 2   # 因为某些原因，目标点在执行过程中无法到达
PENDING = 3   # 因为任务插入，目标点暂时挂起，等待插入任务完成后再执行

string text

"""

class STATUSSPACE():
 PENDING = 0     # 因为任务插入，目标点暂时挂起，等待插入任务完成后再执行
 ACTIVE = 1      # 目标点正在被执行
 SUCCEEDED = 3   # 目标点已到达
 ABORTED = 4     # 因为某些原因，目标点在执行过程中无法到达


class GoalManage():
 def __init__(self):
  self.define()
  rospy.Subscriber('/%s/goal_status'%self.ControlBase, GoalStatus, self.GoalStatusCB)
  rospy.Subscriber('/clicked_point/single', PointStamped, self.SClickCB)
  rospy.Subscriber('/clicked_point/multi', PointStamped, self.MClickCB)
  rospy.Subscriber('/%s/cancleCMD'%self.ControlBase, GoalStatus, self.CancleCB)
  rospy.Timer(rospy.Duration(0.2), self.GoalUpdate) 
  rospy.spin()

  
 def GoalStatusCB(self, status):
  with self.lock:
   if status.status == STATUSSPACE.SUCCEEDED:
    self.Goal_state = {None:None}
   else:
    Status_Feedback = status.status
    Goal_ID = status.goal_id.id
    self.ID = Goal_ID
    self.Goal_state = {Goal_ID: Status_Feedback}


 def GoalUpdate(self, event):
  with self.lock:
   #print 'self.SGoal', self.SGoal, 'self.MGoal', self.MGoal
   if self.SGoal:
    rospy.loginfo('update single goal')
    if self.SGoalUpdata != self.SGoal:
     self.SGoalUpdata = self.SGoal
     self.SendGoal(self.SGoal)
   elif self.MGoal:
    rospy.loginfo('update multi-goals')
    if not self.mgoals or self.MGoalsUpdata != self.MGoal:
     self.MGoalsUpdata = self.MGoal
     self.mgoals = copy.deepcopy(self.MGoal)
    if self.Goal_state[self.ID] == STATUSSPACE.SUCCEEDED:
     self.SendGoal(self.mgoals[0])
     self.mgoals.remove(self.mgoals[0]) 
   else:
    #rospy.loginfo('No Goal published')
    pass


 def SendGoal(self, data):
  GoalPub = rospy.Publisher('/%s/goal'%self.ControlBase, MoveBaseGoal, queue_size = 1)
  PubGoal = MoveBaseGoal()
  PubGoal.target_pose.pose.position = data
  PubGoal.target_pose.header = self.HeaderGenerater()
  CurrentPosition = rospy.wait_for_message('/turtlebot_position_in_map', Pose)
  PubGoal.target_pose.pose.orientation = self.QuaternionGenerater(PubGoal.target_pose.pose.position ,CurrentPosition.position)
  GoalPub.publish(PubGoal)

  
 def QuaternionGenerater(self, end, start):
  theta = self.angle_generater(end, start)
  
  quat = Quaternion()
  ax = 0
  ay = 0
  az = 1 #绕z轴旋转
  quat.x = ax * numpy.sin(theta/2)
  quat.y = ay * numpy.sin(theta/2)
  quat.z = az * numpy.sin(theta/2)
  quat.w = numpy.cos(theta/2)
  
  #or try this:
  #import maplib
  #(quat.x, quat.y, quat.z, quat.w) = maplib.angle_to_quat(theta)
  return quat


 def angle_generater(self, sub_point, pre_point):
  if sub_point.y - pre_point.y > 0:
   if sub_point.x < pre_point.x:
    #print 'sub_point.y-pre_point.y>0 && sub_point.x<pre_point.x'
    angle = numpy.pi - abs(numpy.arctan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x)))
   if sub_point.x == pre_point.x:
    #print 'sub_point.y-pre_point.y>0 && sub_point.x==pre_point.x'
    angle = numpy.pi / 2
   if sub_point.x > pre_point.x:
    #print 'sub_point.y-pre_point.y>0 && sub_point.x==sub_point.x>pre_point.x'
    angle = numpy.arctan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x))
   
  if sub_point.y - pre_point.y < 0:
   if sub_point.x < pre_point.x:
    #print 'sub_point.y-pre_point.y<0 && sub_point.x<pre_point.x'
    angle =- (numpy.pi - abs(numpy.arctan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x))))
   if sub_point.x == pre_point.x:
    #print 'sub_point.y-pre_point.y<0 && sub_point.x==pre_point.x'
    angle =- numpy.pi / 2
   if sub_point.x > pre_point.x:
    #print 'sub_point.y-pre_point.y<0 && sub_point.x>pre_point.x'
    angle =- abs(numpy.arctan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x)))
  
  if sub_point.y - pre_point.y == 0:
   if sub_point.x > pre_point.x:
    #print 'sub_point.y-pre_point.y==0 && sub_point.x>pre_point.x'
    angle = 0.0
   if sub_point.x < pre_point.x:
    #print 'sub_point.y-pre_point.y==0 && sub_point.x<pre_point.x'
    angle = numpy.pi
  return angle
 
 
 def HeaderGenerater(self):
  self.seq += 1
  header = Header()
  header.seq = self.seq
  header.stamp = rospy.Time.now()
  header.frame_id = self.GoalFrame
  return header


 def MClickCB(self, data):
  with self.lock:
   rospy.loginfo('初始化多点任务...')
   self.SGoal = None
   rospy.loginfo('是否重载目标坐标（Y/N）')
   Newinput = raw_input('')
   if Newinput.lower() == 'y':
    self.seq = 0
    if self.Goal_state[self.ID] == STATUSSPACE.ACTIVE:
     cancle_request = STATUSSPACE.ABORTED
     self.PubCancleRequest(self.ID, cancle_request)
     self.MGoal = self.loading()
  
    
 def loading(self):
  Plist = []
  data = raw_input('请输入巡航的目标点数量： ')
  try:
   num = int(data)
   for i in range(num):
    rospy.loginfo ('预备录入第%s个目标点'%(i+1))
    position = rospy.wait_for_message("clicked_point", PointStamped)
    rospy.loginfo ('已录入第%s个目标点'%(i+1))
    goal = self.MGoalGenerater(position.point)
    Plist.append(goal)
   return Plist
  except:
   self.loading()

  
 def MGoalGenerater(self, data):
  goal = MoveBaseGoal()
  goal.target_pose.pose.position = data
  return goal

  
 def SClickCB(self, data):
  with self.lock:
   rospy.loginfo('初始化单点任务...')
   self.MGoal = []
   #rospy.loginfo('是否输入新的目标（Y/N）')
   #Newinput = raw_input('')
   Newinput = 'y'
   if Newinput.lower() == 'y':
    self.seq = 0
    if self.Goal_state[self.ID] == STATUSSPACE.ACTIVE:
     cancle_request = STATUSSPACE.ABORTED
     self.PubCancleRequest(self.ID, cancle_request)
     self.SGoal = data.point
    else:
     self.SGoal = data.point

  
 def PubCancleRequest(self, ID, data): #pub cancle request to planner
  CanclePub = rospy.Publisher('/%s/cancle'%self.ControlBase, GoalStatus, queue_size = 1)
  PubData = GoalStatus()
  PubData.goal_id = int(ID.split('/')[1])
  PubData.status = data
  CanclePub.publish(PubData)
 

 def CancleCB(self, data): #subscribe cancle request from user
  with self.lock:
   if data == STATUSSPACE.PENDING:
    self.MBackUp = self.MGoal
    self.SBackUp = self.SGoal
    self.Inintial_Param()
   elif data == STATUSSPACE.ACTIVE:
    self.MGoal = self.MBackUp
    self.SGoal = self.SBackUp
    self.MBackUp = None
    self.SBackUp = None
   elif data == STATUSSPACE.ABORTED:
    self.MBackUp = None
    self.SBackUp = None
    self.Inintial_Param() 
   else:
    rospy.loginfo('出问题啦 T—T')
    pass

   
 def define(self):
  if not rospy.has_param('~ControlBase'):
   rospy.set_param('~ControlBase','move_base')
  else:
   self.ControlBase = rospy.get_param('~ControlBase')
   
  if not rospy.has_param('~ControlFrequency'):
   rospy.set_param('~ControlFrequency','10') #10hz for controlling
  else:
   self.ControlFrequency = rospy.get_param('~ControlFrequency')

  if not rospy.has_param('~GoalFrame'):
   rospy.set_param('~GoalFrame','map') #10hz for controlling
  else:
   self.GoalFrame = rospy.get_param('~GoalFrame')

  self.lock = Lock()
  self.MBackUp = None
  self.SBackUp = None
  self.ID = None
  self.Inintial_Param()

  
 def Inintial_Param(self):
  self.Goal_state = {}
  self.MGoal = None
  self.SGoal = None
  self.mgoals = None
  self.MGoalsUpdata = None
  self.SGoalUpdata = None
  self.seq = 0
  
   
if __name__ == '__main__':
 rospy.init_node('GoalManage')
 try:
  rospy.loginfo( "initialization system")
  GoalManage()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")
