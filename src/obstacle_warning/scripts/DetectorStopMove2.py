#!/usr/bin/env python
#coding:utf-8
"""
小于阀值点发出报警
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""
import rospy
import numpy
import PyKDL
import maplib
import copy
import tf
import random
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray

class laser_point():
 def define(self):
  self.pose = Pose()
  self.oriation_angle = 0
  self.map = OccupancyGrid()
  self.static_area = []
  self.points_marker = Marker()
  self.cut_points = []#中分线的上下左右终点
  self.centre_points = []
  self.seq = 0

  self.CurrentPose = tf.TransformListener() 

  self.obstacles = LaserScan()
  
  #topic names
  if not rospy.has_param('~warning_marker_topic'):
   rospy.set_param('~warning_marker_topic','/warning_marker')
  self.warning_marker_topic = rospy.get_param('~warning_marker_topic')

  if not rospy.has_param('~stop_flag_topic'):
   rospy.set_param('~stop_flag_topic','/stop_flag')
  self.stop_flag_topic = rospy.get_param('~stop_flag_topic')

  #if not rospy.has_param('~staticarea_marker_topic'):
   #rospy.set_param('~staticarea_marker_topic','/staticarea_marker')
  #self.staticarea_marker_topic = rospy.get_param('~staticarea_marker_topic')

  #if not rospy.has_param('~cleararea_marker_topic'):
   #rospy.set_param('~cleararea_marker_topic','/cleararea_marker')
  #self.cleararea_marker_topic = rospy.get_param('~cleararea_marker_topic')

  #if not rospy.has_param('~Cposition_marker_topic'):
   #rospy.set_param('~Cposition_marker_topic','/Cposition_marker')
  #self.Cposition_marker_topic = rospy.get_param('~Cposition_marker_topic')

  #if not rospy.has_param('~detected_obstacles_topic'):
   #rospy.set_param('~detected_obstacles_topic','/detected_obstacles')
  #self.detected_obstacles_topic = rospy.get_param('~detected_obstacles_topic')
  
  if not rospy.has_param('~Projection_topic'):
   rospy.set_param('~Projection_topic','/test_obstacles/projection')
  self.Projection_topic = rospy.get_param('~Projection_topic') 

  #if not rospy.has_param('~ProjectionSize_topic'):
   #rospy.set_param('~ProjectionSize_topic','/test_obstacles/projection/size')
  #self.ProjectionSize_topic = rospy.get_param('~ProjectionSize_topic') 

  #if not rospy.has_param('~detector_marker_topic'):
   #rospy.set_param('~detector_marker_topic','/detector_marker')
  #self.detector_marker_topic = rospy.get_param('~detector_marker_topic')  

  #最小地图存储模块
  if not rospy.has_param('~detector_resolution'):
   rospy.set_param('~detector_resolution',20)
  self.mim_space = rospy.get_param('~detector_resolution')
  
  #误差允许范围
  if not rospy.has_param('~detector_radius'):
   rospy.set_param('~detector_radius',0.2)
  self.radius = rospy.get_param('~detector_radius')

  #最远警戒范围
  if not rospy.has_param('~Maxdetect'):
   rospy.set_param('~Maxdetect',1.0)
  self.Maxdetect = rospy.get_param('~Maxdetect')
  
  #最近警戒范围
  if not rospy.has_param('~Mindetect'):
   rospy.set_param('~Mindetect',0.0)
  self.Mindetect = rospy.get_param('~Mindetect')
 
  # setting up topics
  self.warning_marker = rospy.Publisher("%s"%self.warning_marker_topic, Marker ,queue_size=1)
  
  #self.staticarea_pub = rospy.Publisher("%s"%self.staticarea_marker_topic, Marker ,queue_size=1)
  
  #self.cleararea_pub = rospy.Publisher("%s"%self.cleararea_marker_topic, Marker ,queue_size=1)

  #self.robotposition_pub = rospy.Publisher("%s"%self.Cposition_marker_topic, Marker ,queue_size=1)

  #self.obstacles_pub = rospy.Publisher("%s"%self.detected_obstacles_topic, LaserScan ,queue_size=1)

  #self.Projection_pub = rospy.Publisher("%s"%self.Projection_topic, PointStamped ,queue_size=1)

  #self.ProjectionSize_pub = rospy.Publisher("%s"%self.ProjectionSize_topic, String ,queue_size=1)
  
  self.Projection_pub = rospy.Publisher("%s"%self.Projection_topic, PoseArray ,queue_size=1)
  
  self.stop_flag = rospy.Publisher("%s"%self.stop_flag_topic, String ,queue_size=1)

  #self.marker_pub = rospy.Publisher("%s"%self.detector_marker_topic, Marker ,queue_size=1)
  
  print 'radiu',self.radius
  
 def __init__(self):
  self.define()
  self.map_data()
  rospy.Subscriber('/scan', LaserScan, self.laser_cb)
  rospy.Subscriber('/odom', Odometry, self.odom_cb)
  rospy.spin()

 def map_data(self):
  self.map = rospy.wait_for_message('map',OccupancyGrid)

  self.statice_area()
   
  #geohash算法区域划分
  width = self.map.info.width
  height = self.map.info.height
  map_origin = self.map.info.origin
  print 'map info', 'width: ', width, '  height: ', height
  self.geohash(self.static_area, width, height, map_origin)
  #line_marker centre_points_marker
  #self.line_centre_markers() 
  
 def statice_area(self): #visual_test
  self.static_area = maplib.get_effective_point(self.map)[1]
  #self.static_area_makers()  
  #self.clear_area = maplib.get_effective_point(self.map)[0]
  #print 'block area:', len(self.static_area), '  clear area:', len(self.clear_area)
  #self.clear_area_makers()
  #print self.static_area[0]
  
 def line_centre_markers(self):
  color = ColorRGBA()
  scale = Point()
  scale.x = 0.05
  scale.y = 0.05
  color.r = 0.0
  color.g = 1.0
  color.b = 0.0
  color.a = 0.5
  self.line_marker = self.visual_test(self.cut_points, Marker.LINE_LIST, color, scale)#标记出区域划分（testing）
  
  color = ColorRGBA()
  scale = Point()
  scale.x = 0.1
  scale.y = 0.1
  color.r = 0.0
  color.g = 0.0
  color.b = 1.0
  color.a = 1.0
  self.centre_points_marker = self.visual_test(self.centre_points,Marker.POINTS, color, scale)
  
 def robot_position_marker(self):
  color = ColorRGBA()
  scale = Point()
  scale.x = 0.1
  scale.y = 0.1
  scale.z = 0.1
  color.r = 1.0
  color.a = 1.0
  self.robot_position = self.visual_test(self.pose, Marker.CUBE, color, scale) 
  
 def static_area_makers(self):
  color = ColorRGBA()
  scale = Point()
  scale.x = 0.05
  scale.y = 0.05
  color.r = 1.0
  color.g = 1.0
  color.b = 0.0
  color.a = 1.0
  self.points_marker = self.visual_test(self.static_area, Marker.POINTS, color, scale)
  #self.static_area=[point1,point2,point3...]
  
 def clear_area_makers(self):
  color = ColorRGBA()
  scale = Point()
  scale.x = 0.05
  scale.y = 0.05
  color.r = 0.0
  color.g = 1.0
  color.b = 0.0
  color.a = 1.0
  self.ClearPoints_marker = self.visual_test(self.clear_area, Marker.POINTS, color, scale)
  print 'finish build markers', len(self.ClearPoints_marker.points)
  
 def LaserDataMarker(self,LaserData): 
  color = ColorRGBA()
  scale = Point()
  scale.x = 0.01
  scale.y = 0.01
  color.r = 0.0
  color.g = 2.0
  color.b = 0.0
  color.a = 1.0
  
  self.laser_points_marker = self.visual_test(LaserData, Marker.POINTS, color, scale)
  
 
 #地图区域划分 
 def geohash(self, data, width, height, map_origin):
  (X,Y) = (1,1)
  pose_mean = Point()
  pose_mean.x = (width/2)*self.map.info.resolution*X+map_origin.position.x
  pose_mean.y = (height/2)*self.map.info.resolution*Y+map_origin.position.y
   
  self._map_divide_store(width, height, data, map_origin.position, pose_mean, (X,Y))
  

 #地图划分区域存储 
 def _map_divide_store(self, width, height, data, map_origin, pose_mean, (X,Y)):
 
  root = self.create_btree(data, width, height, map_origin, pose_mean, (X,Y))

  rospy.loginfo( 'start loading tree' )
  for i in data:
   self.load(root, i)
   
  self.root = root
  rospy.loginfo( 'end building tree' ) 
  
 #创建树 
 def create_btree(self,data, width, height, map_origin, pose_mean, (X,Y)):
  root = {'map':[data], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
  rospy.loginfo( 'start building tree' )
  for i in data:
   self.insert(root, i, width, height, map_origin, (X,Y))
  rospy.loginfo( 'end building tree' )
  return root

 #存入值
 def insert(self, root, data, width, height, map_origin, (X,Y)):

  pose_mean = Point()
  pose_mean.x = abs((width/2)*self.map.info.resolution)*X+map_origin.x
  pose_mean.y = abs((height/2)*self.map.info.resolution)*Y+map_origin.y
  
  linex_start = Point()
  linex_end = Point()
  liney_start = Point()
  liney_end = Point()
  
  linex_start.x = pose_mean.x
  linex_start.y = map_origin.y
  linex_end.x = pose_mean.x
  linex_end.y = map_origin.y+self.map.info.resolution*height*Y

  liney_start.y = pose_mean.y
  liney_start.x = map_origin.x
  liney_end.y = pose_mean.y  
  liney_end.x = map_origin.x+self.map.info.resolution*width*X
  
  line = [linex_start,linex_end,liney_start,liney_end]
  self.cut_points += line
  self.centre_points.append(pose_mean)

  if width <= self.mim_space or height <= self.mim_space:
   pass
   
  else:
   #print 'width, height: ', width, height
   if data.x >= pose_mean.x and data.y >= pose_mean.y:#EN 的条件
    if root['EN'] != None:
     pass
    elif root['EN'] == None:
     root['EN'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree EN errro' )
    (X,Y) = (1,1)
    width /= 2
    height /= 2
    self.insert(root['EN'], data, width, height, pose_mean, (X,Y))
     
   elif data.x >= pose_mean.x and data.y < pose_mean.y:#ES 的条件
    if root['ES'] != None:
     pass
    elif root['ES'] == None:
     root['ES'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree ES errro' )
    (X,Y) = (1,-1)
    width /= 2
    height /= 2
    self.insert(root['ES'], data, width, height, pose_mean, (X,Y))
    
   elif data.x < pose_mean.x and data.y >= pose_mean.y:#WN 的条件
    if root['WN'] != None:
     pass
    elif root['WN'] == None:
     root['WN'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree WN errro' )
    (X,Y) = (-1,1)
    width /= 2
    height /= 2
    self.insert(root['WN'], data, width, height, pose_mean, (X,Y))

   elif data.x < pose_mean.x and data.y < pose_mean.y:#WS 的条件
    if root['WS'] != None:
     pass
    elif root['WS'] == None:
     root['WS'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree WS errro' )
    (X,Y) = (-1,-1)
    width /= 2
    height /= 2
    self.insert(root['WS'], data, width, height, pose_mean, (X,Y))
    
   else:
    rospy.loginfo( 'error during building tree' )

 def load(self,root,i):
  if root['EN'] == None and root['ES'] == None and root['WN'] == None and root['WS'] == None:
  #LEAF
   root['map'].append(i)

#1   
  if root['EN'] != None and root['ES'] == None and root['WN'] == None and root['WS'] == None:
  # EN
   self.load(root['EN'],i)
   
  if root['EN'] == None and root['ES'] != None and root['WN'] == None and root['WS'] == None:
  # ES
   self.load(root['ES'],i)
   
  if root['EN'] == None and root['ES'] == None and root['WN'] != None and root['WS'] == None:
  #WN
   self.load(root['WN'],i)
   
  if root['EN'] == None and root['ES'] == None and root['WN'] == None and root['WS'] != None:
  #WS
   self.load(root['WS'],i)

#2
  if root['EN'] != None and root['ES'] != None and root['WN'] == None and root['WS'] == None:
   #EN ES
   if root['pose_mean'].x <= i.x:
    if root['pose_mean'].y > i.y:
     self.load(root['ES'],i)
    elif root['pose_mean'].y <= i.y:
     self.load(root['EN'],i)
    else:
     rospy.loginfo('EN ES1 error point not in position')

       
  if root['EN'] != None and root['ES'] == None and root['WN'] != None and root['WS'] == None:
  # EN WN
   if root['pose_mean'].y <= i.y:
    if root['pose_mean'].x <= i.x:
     self.load(root['EN'],i)
    elif root['pose_mean'].x > i.x:
     self.load(root['WN'],i)
    else:
     rospy.loginfo('EN WN1 error point not in position')

       
  if root['EN'] != None and root['ES'] == None and root['WN'] == None and root['WS'] != None:
  # EN WS
   if root['pose_mean'].x <= i.x and root['pose_mean'].y < i.y:
    self.load(root['EN'],i)
   elif root['pose_mean'].x > i.x and root['pose_mean'].y > i.y:
    self.load(root['WS'],i)
   else:
    rospy.loginfo('EN WS error point not in position')
  
  if root['EN'] == None and root['ES'] != None and root['WN'] != None and root['WS'] == None:
  # ES WN
   if root['pose_mean'].x <= i.x and root['pose_mean'].y > i.y:
    self.load(root['ES'],i)
   elif root['pose_mean'].x > i.x and root['pose_mean'].y <= i.y:
    self.load(root['WN'],i)
   else:
    rospy.loginfo('ES WN error point not in position')
   
  if root['EN'] == None and root['ES'] != None and root['WN'] == None and root['WS'] != None:
  # ES WS
   if root['pose_mean'].y > i.y:
    if root['pose_mean'].x > i.x:
     self.load(root['WS'],i)
    elif root['pose_mean'].x <= i.x:
     self.load(root['ES'],i)
    else:
     rospy.loginfo('ES WS1 error point not in position')

         
  if root['EN'] == None and root['ES'] == None and root['WN'] != None and root['WS'] != None:
  # WN WS
   if root['pose_mean'].x >= i.x:
    if root['pose_mean'].y <= i.y:
     self.load(root['WN'],i)
    elif root['pose_mean'].y > i.y:
     self.load(root['WS'],i)
    else:
     rospy.loginfo('WN WS1 error point not in position')

   
#3
  if root['EN'] != None and root['ES'] != None and root['WN'] != None and root['WS'] == None:
  #EN ES WN
   if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   else:
    #rospy.loginfo('EN ES WN error point not in position')
    return None
    
  if root['EN'] != None and root['ES'] != None and root['WN'] == None and root['WS'] != None:
  # EN ES WS
   if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
    self.load(root['ES'],i)
    
   else:
    #rospy.loginfo('EN ES WS error point not in position')
    return None

  if root['EN'] != None and root['ES'] == None and root['WN'] != None and root['WS'] != None:
  # EN WN WS
   if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
    self.load(root['WS'],i)
    
   else:
    #rospy.loginfo('EN WN WS error point not in position')
    return None
    
  if root['EN'] == None and root['ES'] != None and root['WN'] != None and root['WS'] != None:
  # ES WN WS
   if i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
    self.load(root['WS'],i)
    
   else:
    #rospy.loginfo('ES WN WS error point not in position')
    return
#4  
  if root['EN'] != None and root['ES'] != None and root['WN'] != None and root['WS'] != None:
   # EN ES WN WS
   if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
    self.load(root['WS'],i)
   
   else:
    rospy.loginfo('EN ES WN WS error point not in position')
    return    
     
 #查询值 
 def read(self, root, i):
  #rospy.loginfo('checking the map')
  if i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
   if root['WN'] == None:
    for j in root['map']:
     conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3))) < (self.radius, self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['WN'], i)
   
  elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
   if root['WS'] == None:
    for j in root['map']:
     conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3))) < (self.radius, self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['WS'], i)
   
  elif i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
   if root['EN'] == None:
    for j in root['map']:
     conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3))) < (self.radius, self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['EN'], i)
   
  elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
   if root['ES'] == None:
    for j in root['map']:
     conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3)))<(self.radius, self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['ES'], i)
   
  else:
   rospy.loginfo('查询值  error point not in position')
 
 
 # get odom 
 def odom_cb(self,data):
  #print '更新坐标'
  self.oriation_angle = -self.Q2A([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
  
  self.CurrentPose.waitForTransform('/map', '/camera_depth_frame', rospy.Time(0), rospy.Duration(0.7))
  (pos,rot) = self.CurrentPose.lookupTransform('/map', '/camera_depth_frame', rospy.Time())
  self.pose.position.x = pos[0]
  self.pose.position.y = pos[1]
  self.pose.position.z = pos[2]
  
  #self.robot_position_marker()
  #self.robotposition_pub.publish(self.robot_position)
  
  #发布visual_test的结果
  try:
   #self.staticarea_pub.publish(self.points_marker) #marker out all static area
   #self.cleararea_pub.publish(self.ClearPoints_marker)
   #self.marker_pub.publish(self.line_marker) #marker out divie line
   #self.marker_pub.publish(self.centre_points_marker)
   pass
  except:
   pass
  
  
 def laser_cb(self,data):
  #self.obstacles.header = data.header
  #self.obstacles.angle_min = data.angle_min
  #self.obstacles.angle_max = data.angle_max
  #self.obstacles.range_min = self.Mindetect
  #self.obstacles.range_max = self.Maxdetect
  #self.obstacles.angle_increment = data.angle_increment
  #self.obstacles.time_increment = data.time_increment
  #self.obstacles.ranges = []
  LaserData = []
  MateData = []
  count = 0
  particles = 65
  """#Mindistance = [numpy.inf, None]
  #Maxdistance = [0.0, None]"""
  CastData = []
  
  for i in data.ranges:
   if self.Mindetect < i < self.Maxdetect:
    #print 'get total points:', count, i
    #self.obstacles.ranges.append(i)
    MateData.append([i, count])
   else:
    #self.obstacles.ranges.append(0.0)
    pass
   count += 1
  
  point = Point() 
  if len(MateData) > particles:
   SampleData = random.sample(MateData, 10)
   for j in SampleData: 
    angle = data.angle_min + data.angle_increment * j[1] - self.oriation_angle
    point.x = j[0] * numpy.cos(angle) + self.pose.position.x
    point.y = j[0] * numpy.sin(angle) + self.pose.position.y
    LaserData.append(copy.deepcopy(point))
  else:
   pass

  self.result = self.check(LaserData)
  #print self.result
  if self.result: 
   #rospy.loginfo('判断:是在误差许可之内为地图上已知点')
   self.addFlag()
   """#发布visual_test的结果
   #self.LaserDataMarker(LaserData)
   #self.marker_pub.publish(self.laser_points_marker)"""
   """#CastData = [Mindistance, Maxdistance, MateData[0], MateData[-1]]"""
   #print 'CastData', len(CastData), '\n', CastData
   CastData = maplib.GradientDes(MateData)

   """ProjectionDataSet = maplib.ProjectionPole(CastData, data, [self.oriation_angle, self.pose.position.x, self.pose.position.y])
   self.Projection_pub.publish(ProjectionDataSet)
   OriginPosition = PointStamped()
   OriginPosition.header = data.header
   OriginPosition.point = ProjectionDataSet[0]
   self.Projection_pub.publish(OriginPosition)
   print ProjectionDataSet[1], numpy.ceil(ProjectionDataSet[1] / self.map.info.resolution)
   self.ProjectionSize_pub.publish('%s'%numpy.ceil(ProjectionDataSet[1] / self.map.info.resolution))"""
   
  ProjectionDataSets = PoseArray()
  Castpose = Pose()  
  point = Point() 
  #MarkCastData = [] 
  for MCdata in CastData:

   #print 'MCdata', MCdata,MCdata[1], data.angle_increment
   angle = data.angle_min + data.angle_increment * MCdata[1] - self.oriation_angle
   point.x = MCdata[0] * numpy.cos(angle) + self.pose.position.x
   point.y = MCdata[0] * numpy.sin(angle) + self.pose.position.y
   Castpose.position = point
   #MarkCastData.append(copy.deepcopy(point))
   ProjectionDataSets.poses.append(copy.deepcopy(Castpose))
   
  #self.LaserDataMarker(MarkCastData)
  #self.marker_pub.publish(self.laser_points_marker)
  
  ProjectionDataSets.header.seq = self.seq
  ProjectionDataSets.header.stamp = rospy.Time.now()
  ProjectionDataSets.header.frame_id = '/map'
  self.Projection_pub.publish(ProjectionDataSets)
  
  #self.obstacles_pub.publish(self.obstacles)
  self.seq += 1
      
#检查镭射是否为地图上的静态障碍物（eg，wall door etc.）
 def check(self, data):
  trigger = 0
  for i in data:
   result = self.read(self.root,i)
   if result is True:
    trigger += 1
   else:
    pass
  if trigger >= 0.8*len(data):
   return False
  else:
   return True


# 视觉显示检测结果
 def visual_test(self, data, Type, color, scale):
#plot POINTS
  #print len(data),data[0],data[1]
  if Type == Marker.POINTS:
   #print 'pub POINTS Marker'
   point_marker = Marker()
   point_marker.header.frame_id = '/map'
   point_marker.header.stamp = rospy.Time.now()
   point_marker.ns = 'detector_visual_test'
   point_marker.action = Marker.ADD
   
   point_marker.id = 0
   point_marker.type = Type
   point_marker.scale.x = scale.x#0.1
   point_marker.scale.y = scale.y#0.1
   point_marker.points = data
   for i in data:
    point_marker.colors.append(color)
   point_marker.lifetime = rospy.Duration(0.1)
   return point_marker
   
#plot LINE_LIST  
  if Type == Marker.LINE_LIST:
   #print 'pub LINE_LIST Marker'
   line_marker = Marker()
   line_marker.header.frame_id = '/map'
   line_marker.header.stamp = rospy.Time.now()
   line_marker.ns = 'detector_visual_test'
   line_marker.action = Marker.ADD
   
   line_marker.id = 1
   line_marker.type = Type
   line_marker.scale.x = scale.x#0.05
   line_marker.scale.y = scale.y#0.05  
   
   line_marker.points = data
   for i in data:
    line_marker.colors.append(color)

   line_marker.lifetime = rospy.Duration(0.5)
   
   return line_marker
   
#plot WORDS
  if Type == Marker.TEXT_VIEW_FACING:
   #details 
   flag_marker = Marker()
   flag_marker.type = Type
   flag_marker.header.frame_id='map'
   flag_marker.text = "     WARNING!!!\n OBSTACLE DETECTED!!"
   flag_marker.ns = "WarningFlag"
   flag_marker.color = color
   flag_marker.scale = scale
   flag_marker.header.stamp = rospy.Time.now()
   flag_marker.lifetime = rospy.Duration(0.3)
   flag_marker.pose = data
   return flag_marker
   
#plot robot position
  if Type == Marker.CUBE:
   #details 
   robot_marker = Marker()
   robot_marker.type = Type
   robot_marker.header.frame_id = 'map'
   robot_marker.scale.x = scale.x#0.05
   robot_marker.scale.y = scale.y#0.05 
   robot_marker.scale.z = scale.z
   robot_marker.header.stamp = rospy.Time.now()
   robot_marker.lifetime = rospy.Duration(0.5)
   robot_marker.pose = data
   robot_marker.color = color
   return robot_marker
   
 def Q2A(self,quat):
  rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
  return rot.GetRPY()[2]
 
  
 def addFlag(self):
  self.flag_makers()
  self.warning_marker.publish(self.flag_marker)
  #print 'add flag', self.flag_marker
  
 def flag_makers(self):
  color = ColorRGBA()
  scale = Point()
  pose = Pose()
  scale.x = 0.01
  scale.y = 0.01
  scale.z = 0.2
  pose = self.pose
  pose.position.z += 0.5
  color.r = 1.0
  color.a = 1.0
  self.flag_marker = self.visual_test(pose, Marker.TEXT_VIEW_FACING, color, scale)


  
if __name__ == '__main__':
 rospy.init_node("DetectorStopMove", anonymous=True)
 try:
  rospy.loginfo ("initialization system")
  laser_point()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")
