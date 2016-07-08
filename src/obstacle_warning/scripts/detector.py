#!/usr/bin/env python
#coding:utf-8
"""
小于阀值点发出报警
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""
import rospy,numpy,PyKDL,maplib
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String,ColorRGBA
from geometry_msgs.msg import Point,Pose
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import Marker

class laser_point():
 def define(self):
  self.pose=Pose()
  self.oriation_angle=0
  self.map=OccupancyGrid()
  self.coor=[]
  self.static_area=[]
  self.points_marker=Marker()
  self.cut_points=[]
  
  
  if not rospy.has_param('~stop_flag_topic'):
   rospy.set_param('~stop_flag_topic','/stop_flag')
  self.stop_flag_topic = rospy.get_param('~stop_flag_topic')

  self.stop_flag=rospy.Publisher("%s"%self.stop_flag_topic, String ,queue_size=1)
  self.marker_pub=rospy.Publisher("detector_marker", Marker ,queue_size=1)
  
 def __init__(self):
  self.define()
  self.map_data()
  rospy.Subscriber('/scan', LaserScan, self.laser_cb)
  rospy.Subscriber('turtlebot_position_in_map',Pose,self.pose_cb)
  rospy.Subscriber('/odom', Odometry, self.odom_pose_cb)
  rospy.spin()

 def map_data(self):
  self.map=rospy.wait_for_message('map',OccupancyGrid)
  self.statice_area()
  
 def statice_area(self):
  self.static_area=maplib.get_effective_point(self.map)[1]
  self.points_marker=self.visual_test(self.static_area,Marker.POINTS)
  
  #geohash算法区域划分
  width=self.map.info.width
  height=self.map.info.height
  resolution=self.map.info.resolution
  map_origin=self.map.info.origin
  key=['map_']
  map_dict={'%s'%key[0]:self.static_area}
  
  self.geohash(key, map_dict, width, height, resolution, map_origin, 1, 1)
 
 #地图区域划分 
 def geohash(self, key, map_dict, width, height, resolution, map_origin, X, Y):
  #获取中心点坐标
  pose_mean=Point()
  line=[]
  linex_start=Point()
  linex_end=Point()
  liney_start=Point()
  liney_end=Point()
  
  pose_mean.x=(width/2)*resolution*X+map_origin.position.x
  pose_mean.y=(height/2)*resolution*Y+map_origin.position.y
  
  linex_start.x=pose_mean.x
  linex_start.y=map_origin.position.y
  linex_end.x=pose_mean.x
  linex_end.y=map_origin.position.y+resolution*height*Y

  liney_start.y=pose_mean.y
  liney_start.x=map_origin.position.x
  liney_end.y=pose_mean.y  
  liney_end.x=map_origin.position.x+resolution*width*X
  
  line=[linex_start,linex_end,liney_start,liney_end]
  self.cut_points.append(linex_start,linex_end,liney_start,liney_end)
  
  self.line_marker=self.visual_test(line,Marker.LINE_LIST)#标记出第一层区域划分（testing）
  
  width/=2
  height/=2
  map_origin.position=pose_mean  
  
  self._map_divide_store(width,height,data,map_origin)
  
 #地图划分区域存储 
 def _map_divide_store(self,width,height,data,map_origin):

  NWSE=''
  map_Subdict={}
  
  if len(key)==1: 
   static_area=map_dict['%s'%key[0]]
  else:
   for i in range(len(key)):
    if type(map_dict['%s'%key[i]]) is dict:
     pass
    ###怎么调下一级的字典defaultdict方法
    
    
  static_Subarea={'WN':[],'EN':[],'WS':[],'ES':[]}

  for point in static_area:
   # divide to 4 parts
   if linex_start.x<=point.x<pose_mean.x:
    NWSE+='E'
   elif pose_mean.x<=point.x<linex_end.x:
    NWSE+='W'
   else:
    rospy.loginfo('point x coordinate is out of range')
    
   if linex_start.y<=point.y<pose_mean.y:
    NWSE+='S'
   elif pose_mean.y<=point.y<linex_end.y:
    NWSE+='N'
   else:
    rospy.loginfo('point y coordinate is out of range')
   # classify points into it's area
   static_Subarea['%s'%NWSE].append(point)  #计算static_Subarea

  map_dict['%s'%key]=static_Subarea  
  if NWSE=='WN':
   (X,Y)=(X,Y)
   if not (int(width)<=10 or int(height)<=10):
    self.geohash(key, map_dict, width, height, resolution, map_origin, X, Y)
  if NWSE=='EN':
   (X,Y)=(-X,Y)
   if not (int(width)<=10 or int(height)<=10):
    self.geohash(key, map_dict, width, height, resolution, map_origin, X, Y)
  if NWSE=='WS':
   (X,Y)=(X,-Y)
   if not (int(width)<=10 or int(height)<=10):
    self.geohash(key, map_dict, width, height, resolution, map_origin, X, Y)
  if NWSE=='ES':
   (X,Y)=(-X,-Y)
   if not (int(width)<=10 or int(height)<=10):
    self.geohash(key, map_dict, width, height, resolution, map_origin, X, Y)
#############################################################
  
  
 def odom_pose_cb(self, data):
  self.oriation_angle=self.Q2A([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
  self.orientation=data.pose.pose.orientation
  #print self.marker.pose.orientation
  self.marker_pub.publish(self.points_marker) #marker out all static area
  self.marker_pub.publish(self.line_marker) #marker out divie line
    
 def pose_cb(self,data):
  self.pose=data
  
 def laser_cb(self,data):
  angle_min=data.angle_min
  angle_max=data.angle_max
  angle_inc=data.angle_increment
  
  point=Point()

  LaserData=[]
  count=0
  for i in data.ranges:
   #print 'get total points:', count
   if i < 0.7:
    if angle_min+angle_inc*count<angle_max: 
     #print 'get point'
     (point.y,point.x)=(i*numpy.sin(self.oriation_angle+angle_min+angle_inc*count), i*numpy.cos(self.oriation_angle+angle_min+angle_inc*count))
     #if self.check(point): #判断是否在误差许可之内
      #self.stop_flag.publish('stop')
   count+=1
  #print 'pub marker'
  
######################################################  
#检查镭射是否为地图上的静态障碍物（eg，wall door etc.）
 #def check(self,data):
  #CheckPoint=data.x/2
  #n=2
  #for i in range(n):
   #for StaticPoint in self.static_area:
    #if (data.x+0.1*i) - StaticPoint:
     #if (data.y+0.1*i) - self.static_area:
     
     ####
     
     
     
##################################################   

# 视觉显示检测结果
 def visual_test(self,data,Type):#data=[point1,point2,point3...]
#plot POINTS
  #print len(data),data[0],data[1]
  color=ColorRGBA()
  if Type==Marker.POINTS:
   #print 'pub POINTS Marker'
   point_marker=Marker()
   point_marker.header.frame_id='/map'
   point_marker.header.stamp=rospy.Time.now()
   point_marker.ns='detector_visual_test'
   point_marker.action=Marker.ADD
   
   point_marker.id=0
   point_marker.type=Type
   point_marker.scale.x=0.05
   point_marker.scale.y=0.05
   
   color.r=1.0
   color.g=1.0
   color.b=0.0
   color.a=1.0
   
   point_marker.points=data
   for i in data:
    point_marker.colors.append(color)

   point_marker.lifetime=rospy.Duration(0.5)
   
   return point_marker
   
#plot LINE_LIST  
  if Type==Marker.LINE_LIST:
   print 'pub LINE_LIST Marker'
   line_marker=Marker()
   line_marker.header.frame_id='/map'
   line_marker.header.stamp=rospy.Time.now()
   line_marker.ns='detector_visual_test'
   line_marker.action=Marker.ADD
   #line_marker.pose.orientation=self.orientation
   
   line_marker.id=1
   line_marker.type=Type
   line_marker.scale.x=0.05
   line_marker.scale.y=0.05  

   color.r=0.0
   color.g=1.0
   color.b=0.0
   color.a=1.0
   
   line_marker.points=data
   for i in data:
    line_marker.colors.append(color)

   line_marker.lifetime=rospy.Duration(0.1)
   
   return line_marker
   
######################################################   


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
