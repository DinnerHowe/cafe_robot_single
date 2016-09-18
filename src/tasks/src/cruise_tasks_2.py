#!/usr/bin/env python
#coding=utf-8
"""
multi-goal tasks,不在actions_reference中进行检测目标点迭代（有缺陷）

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import getpass
#import actions_reference
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

class cruise_modle():
 def __init__(self):
  Plist=[]
  #Current_position = rospy.wait_for_message("turtlebot_position_in_map", Pose)
  self.initial = True
  self.Glist=self.loading(Plist)
  self.counter = 0
  self.seq = 0

  self.goal=Marker()
  self.goal.color.r=1.0
  self.goal.color.g=0.0
  self.goal.color.b=0.0
  self.goal.color.a=1.0
  self.goal.ns='task_points'
  self.goal.scale.x=0.1
  self.goal.scale.y=0.1
  self.goal.header.frame_id='map'
  self.goal.type=Marker.SPHERE_LIST
  self.goal.action=Marker.ADD
  self.goal.lifetime = rospy.Duration(0)
  
  rospy.loginfo('start cruising task')
  self.pub=rospy.Publisher("multi_marker", Marker, queue_size=1)
  
  rospy.Subscriber("turtlebot_position_in_map", Pose, self.odom_callback)
  
  rospy.spin()
  
 def odom_callback(self, odom):
  self.current_position = odom.position
  #rospy.loginfo('checking if achieve goal')
  
  Errx = self.current_position.x - self.Glist[self.counter].x
  Erry = self.current_position.y - self.Glist[self.counter].y
  
  #print abs(Errx), '=' , self.current_position.x, '-', self.Glist[self.counter-1].x
  #print abs(Erry), '=' , self.current_position.y, '-', self.Glist[self.counter-1].y
  #print self.counter
   
  if self.counter < (len(self.Glist)-1):
  
   if self.initial:
    print 'initial'
    self.initial = False
    self.goal.id = self.seq
    self.goal.header.stamp =rospy.Time.now()
    self.goal.points.append(self.Glist[self.counter])
    self.pub.publish(self.goal)
   
   if abs(Errx) < 0.1 and abs(Erry) < 0.1:
    print self.counter,' times'
    self.counter += 1
    self.seq += 1
    self.goal.id = self.seq
    self.goal.header.stamp =rospy.Time.now()
    self.goal.points.append(self.Glist[self.counter])
    #rospy.sleep(0.5)
    self.pub.publish(self.goal)
    #actions_reference.multitasks(Current_position, self.Glist[self.counter])
   elif:
    #self.pub.publish(self.goal)

  else :
   print 'backward'
   if abs(Errx) < 0.1 and abs(Erry) < 0.1:
    self.Glist.reverse()
    self.counter = 0
    self.seq += 1
    self.goal.id = self.seq
    self.goal.header.stamp =rospy.Time.now()
    self.goal.points.append(self.Glist[self.counter])
    #rospy.sleep(0.5)
    self.pub.publish(self.goal)
   else:
    #self.pub.publish(self.goal)

 def loading(self, Plist):
  data=raw_input('请输入巡航的目标点数量： ')
  try:
   num = int(data)
   for i in range(num):
    rospy.loginfo ('预备录入第%s个目标点'%(i+1))
    position =  rospy.wait_for_message("clicked_point",PointStamped)
    rospy.loginfo ('已录入第%s个目标点'%(i+1))
    Plist.append(position.point)
  except:
   self.loading(Plist)
  return Plist
  

if __name__ == '__main__':
 rospy.init_node('cruise_tasks')
 try:
  rospy.loginfo( "initialization system")
  cruise_modle()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")

