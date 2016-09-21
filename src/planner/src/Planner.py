#!/usr/bin/env python
#coding=utf-8
""" 
planer

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
import copy
import numpy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import PlanAlgrithmsLib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
import maplib
from threading import Lock
import tf

class planner():
 def __init__(self):
  self.define()
  rospy.Subscriber('/map', OccupancyGrid, self.MapCB)
  rospy.Subscriber('/clicked_point', PointStamped, self.ClickCB)#用啦来测试plan
  rospy.Subscriber('/turtlebot_position_in_map', Pose, self.OdomCB)
  
  #rospy.Subscriber('/move_base/goal', MoveBaseGoal, self.GoalCB)# this is used to testing crash function
  #rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.testing) # this is used to testing crash function
  rospy.spin()
  
  # this is used to testing crash function   
 """def testing(self, data):
  point = Point()
  self.SubGoal = []
  for i in data.poses:
   point.x = round(i.pose.position.x, 1)
   point.y = round(i.pose.position.y, 1)
   point.z = round(i.pose.position.z, 1)
   self.SubGoal.append(copy.deepcopy(point))"""  
  
 def Crash(self, _map, data):
  for i in data:
   num = maplib.position_num(_map, i)
   if _map.data[num] == 100:
    return True
  
 def MapCB(self, data): 
  with self.locker:
   if self.SubGoal:
    if cmp(self.Map.data, data.data):
     self.Map = data
     if self.Crash(self.Map, self.SubGoal):
      #self.testcount+=1
      #print 'crash', self.testcount
      rospy.loginfo('Detected Crashing risk')
      #makeplan return Path
      PlanPath = self.MakePlan(self.Map, self.CurrenPosition, self.Goal) 
      self.UpdateSubGoalList(PlanPath)

 def OdomCB(self, data):
  with self.locker:
   self.odom = data
   if self.SubGoal:
    self.CurrenPosition = data.position
    cmd = self.DifferCMD(self.CurrenPosition, self.SubGoal[0])
    if self.AchieveSubGoal(cmd):
     self.SubGoal.remove(self.SubGoal[0])
    else:
     self.Move(cmd)
    if sel.AchieveGoal(self.Goal):
     self.GoalState = 1
   
 def AchieveSubGoal(self, cmd):
  if round(cmd.linear.x, self.Angle_accuracy) == 0 and round(cmd.linear.y, self.Angle_accuracy) == 0 and round(cmd.angular.z, self.Angle_accuracy) == 0:
   return True 
   
 def AchieveGoal(self, goal):
  if round((self.odom.position.x - goal.target_pose.pose.position.x), self.Angle_accuracy) == 0 and round((self.odom.position.y - goal.target_pose.pose.position.y), self.Angle_accuracy) == 0 and round((self.odom.orientation.x - goal.target_pose.pose.orientation.x), self.Angle_accuracy) == 0 and round((self.odom.orientation.y - goal.target_pose.pose.orientation.y), self.Angle_accuracy) == 0 and round((self.odom.orientation.z - goal.target_pose.pose.orientation.z), self.Angle_accuracy) == 0 and round((self.odom.orientation.w - goal.target_pose.pose.orientation.w), self.Angle_accuracy) == 0:
   return True
 
 def DifferCMD(self, CurrenPosition, SubGoal):
  trans = [SubGoal.x - CurrenPosition.x, SubGoal.y - CurrenPosition.y]
  if trans > (self.Position_accuracy, self.Position_accuracy):
   linear = 0.5 * ((trans[0])**2 + (trans[1])**2)
   angular = 4 * numpy.actan(trans[1], trans[0])
  else:
   linear = 0.0
   angular = round(Angle(SubGoal, CurrenPosition), self.Angle_accuracy)
   
  cmd = Twist()
  cmd.linear.x = linear
  cmd.angular.z = angular
  return cmd
  
 def Move(self, cmd):
  MovePub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
  MovePub.publish(cmd)

 def ClickCB(self, data):
  goal = MoveBaseGoal()
  goal.target_pose.pose.position = data.point
  goal.target_pose.pose.orientation.w = 1
  goal.target_pose.header = data.header
  self.GoalCB(goal)
 
 def GoalCB(self, data):
  with self.locker:
   if self.seq_num != data.target_pose.header.seq:
    self.seq_num = data.target_pose.header.seq
    self.Goal_Update(data.target_pose)
    print self.Map.info
    if self.Map != OccupancyGrid():
     if self.GoalAchieveable(self.Map, self.Goal):
      PlanPath = self.MakePlan(self.Map.data, self.CurrenPosition, self.Goal)
      self.UpdateSubGoalList(PlanPath)

 def Goal_Update(self, data):
  self.Goal = data.pose
     
 def UpdateSubGoalList(self, data): # self.SubGoal : pose[]
  self.SubGoal = data
  
 def MakePlan(self, data, start, goal):# SubGoal : pose[]
  if not rospy.has_param('~Planner_Algrithm'):
   rospy.set_param('~Planner_Algrithm','JPS')
  else:
   Planner_Algrithm = rospy.get_param('~Planner_Algrithm')
   
  rospy.loginfo('making a plan')
  if Planner_Algrithm == 'JPS':
   rospy.loginfo('making JPS plan')
   Plan = PlanAlgrithmsLib.Simple_JPS_Planner(data, start, goal)
  PublishPlan(Plan)
  print len(plan)
  return Plan
          
 def GoalAchieveable(self, _map, goal):
  print type(_map),_map.info.resolution
  num = maplib.position_num(_map, goal)
  if _map.data[num] != 100:
   for i in range(self.radius):
    for j in range(self.radius):
     if _map.data[_num +i * _map.info.width + j] != 100 and _map.data[_num + i * _map.info.width - j] != 100 and _map.data[_num - i * _map.info.width + j] != 100 and _map.data[_num - i * _map.info.width - j] != 100:
      pass
     else:
      rospy.loginfo('Goal not achieveable')
      return False
   return True
    
 def PublishPlan(data):
  path = Path()
  pose = PoseStamped()
  self.plan_seq +=1
  path.header.seq = self.plan_seq
  path.header.frame_id = self.frame_id
  path.header.stamp = rospy.Time.now()
  pose_seq = 0
  for i in data:
   pose.header.seq = pose_seq
   pose.header.stamp = rospy.Time.now()
   pose.header.frame_id = self.frame_id
   pose.pose.position.x = i[0]
   pose.pose.position.y = i[1] 
   path.poses.append(pose)
  
  self.PlanPub.publish()
    
 def define(self):
  self.PlanPub = rospy.Publisher('/move_base/DWAPlannerROS/global_plan', Path, queue_size = 1)
  
  if not rospy.has_param('~frame_id'):
   rospy.set_param('~frame_id','/map')
  self.frame_id = rospy.get_param('~frame_id')
  
  self.Angle_accuracy = 2
  self.Position_accuracy = 0.01
  self.seq_num = None
  self.plan_seq = 0
  self.Map = rospy.wait_for_message('/map', OccupancyGrid)
  self.SubGoal = []
  self.locker = Lock()
  self.radius = 8
  #self.testcount = 0

  
if __name__ == '__main__':
 rospy.init_node('planner')
 try:
  rospy.loginfo( "initialization system")
  planner()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")
