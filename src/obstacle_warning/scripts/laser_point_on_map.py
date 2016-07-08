#!/usr/bin/env python
#coding:utf-8
"""
在地图上标记出laser点
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""
import rospy,numpy,PyKDL
from sensor_msgs.msg import LaserScan
from std_msgs.msg import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Pose
from nav_msgs.msg import Odometry

class laser_point():
 def define(self):
  self.marker=Marker()
  self.pose=Pose()

  self.marker.id = 0
  
  self.color=ColorRGBA()
  self.color.r=0.0
  self.color.g=1.0
  self.color.b=0.0
  self.color.a=1.0
  
  self.oriation_angle=0
  
  self.marker.ns='laser_points'
  self.marker.scale.x=0.01
  self.marker.scale.y=0.01
  
  self.marker.type=Marker.POINTS
  self.marker.action=Marker.ADD
  self.marker.lifetime = rospy.Duration(0.1)
  self.marker_pub=rospy.Publisher("laser_marker",Marker,queue_size=1)


 def __init__(self):
  self.define()
  rospy.Subscriber('/scan', LaserScan, self.laser_cb)
  rospy.Subscriber('turtlebot_position_in_map',Pose,self.pose_cb)
  rospy.Subscriber('/odom', Odometry, self.odom_pose_cb)
  rospy.spin()


 def odom_pose_cb(self, data):
  self.marker.pose.orientation=data.pose.pose.orientation
  self.oriation_angle=self.Q2A([self.marker.pose.orientation.x,self.marker.pose.orientation.y,self.marker.pose.orientation.z,self.marker.pose.orientation.w])
  #print self.marker.pose.orientation
  
 def pose_cb(self,data):
  self.pose=data
  
 def laser_cb(self,data):
  angle_min=data.angle_min
  angle_max=data.angle_max
  angle_inc=data.angle_increment
  
  point=Point()
  self.marker.header.stamp =rospy.Time.now()
  self.marker.header.frame_id='map'
  
  #print 'angle_min:', angle_min,'\n','angle_max:', angle_max,'\n','angle_inc:', angle_inc,'\n'
  LaserData=[]
  count=0
  for i in data.ranges:
   #print 'get total points:', count
   if i < 0.7:
    if type(i) is float:
     #print 'get points:', count
     if angle_min+angle_inc*count<angle_max: 
      #print 'get point'
      (point.y,point.x)=(i*numpy.sin(self.oriation_angle+angle_min+angle_inc*count), i*numpy.cos(self.oriation_angle+angle_min+angle_inc*count))
      self.marker.points.append(point)
      self.marker.colors.append(self.color)
   count+=1
  self.marker_pub.publish(self.marker)  
  #print 'pub marker'
  
  
 def Q2A(self,quat):
  rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
  return rot.GetRPY()[2]
  
  
if __name__ == '__main__':
 rospy.init_node("laser_point", anonymous=True)
 try:
  rospy.loginfo ("initialization system")
  laser_point()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")
