#!/usr/bin/env python
#coding=utf-8
""" 
goal manager

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
import copy
from move_base_msgs.msg import MoveBaseGoal
from planner.msg import GoalStatus
from threading import Lock

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
  rospy.Subscriber('/%s/goal_status'%self.ControlBase, GoalStatus, self.GoalManageCB)
  rospy.Subscriber('/clicked_point/single', PointStamped, self.SClickCB)
  rospy.Subscriber('/clicked_point/multi', PointStamped, self.MClickCB)
  rospy.Subscriber('/%s/cancleCMD'%self.ControlBase, GoalStatus, self.CancleCB)
  rospy.Timer(rospy.Duration(0.2), self.GoalUpdate) 
  rospy.spin()
  
 def GoalManageCB(self, status):
  with self.lock:
   if status.status = :
    
   else:
    Status_Feedback = status.status
    Goal_ID = status.status
    self.Goal_state = {Goal_ID: Status_Feedback}



 def GoalUpdate(self, event):
  if self.SGoal:
   self.SendGoal(self.SGoal)
  elif self.MGoal:
   if self.mgoals:
    self.mgoals = copy.deepcopy(self.MGoal)
   if self.Goal_state
  else:
   self.SendGoal()
  
  
   self.SendGoal(self.PositionList, data)
   

 def CancleCB(self, data):
  
  
 def SClickCB(self, data):
  rospy.loginfo('初始化单点任务...')
  self.MGoals = []
  rospy.loginfo('是否输入新的目标（Y/N）')
  Newinput = raw_input('')
  if Newinput.lower() = 'y':
  
  
 def MClickCB(self, data):
  rospy.loginfo('初始化多点任务...')
  self.SGoal = MoveBaseGoal()
  rospy.loginfo('是否输入新的目标（Y/N）')
  Newinput = raw_input('')
  if Newinput.lower() = 'y':
   self.NewDest = data
   self.cancle_request = True
   self.CancelRes.publish(self.cancle_request)

   
 def loading(self):
  Plist = []
  data = raw_input('请输入巡航的目标点数量： ')
  try:
   num = int(data)
   for i in range(num):
    rospy.loginfo ('预备录入第%s个目标点'%(i+1))
    position =  rospy.wait_for_message("clicked_point",PointStamped)
    rospy.loginfo ('已录入第%s个目标点'%(i+1))
    Plist.append(position.point)
   self.loaded = True
   return Plist
  except:
   self.loading()
   
 def define(self):
  if not rospy.has_param('~ControlBase'):
   rospy.set_param('~ControlBase','move_base')
  else:
   self.ControlBase = rospy.get_param('~ControlBase')
 
  self.lock = Lock()
  self.Goal_state = {}
  self.MGoal = None
  self.SGoal = None
  
  self.GoalPub = rospy.Publisher('/%s/goal'%self.ControlBase, MoveBaseGoal, queue_size = 1)
  self.Accuracy = 2

   
if __name__ == '__main__':
 rospy.init_node('GoalManage')
 try:
  rospy.loginfo( "initialization system")
  GoalManage()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")
