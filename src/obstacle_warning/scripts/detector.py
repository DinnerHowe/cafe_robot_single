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
from std_msgs.msg import String,ColorRGBA#,Time
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
  #self.points_marker=Marker()
  self.cut_points=[]#中分线的上下左右终点
  self.centre_points=[]
  self.feedback=''
  #self.now=Time()
  
  if not rospy.has_param('~detector_resolution'):
   rospy.set_param('~detector_resolution',20)
  self.mim_space = rospy.get_param('~detector_resolution')
  
  if not rospy.has_param('~detector_radius'):
   rospy.set_param('~detector_radius',0.2)
  self.radius = rospy.get_param('~detector_radius')
  
  if not rospy.has_param('~stop_flag_topic'):
   rospy.set_param('~stop_flag_topic','/stop_flag')
  self.stop_flag_topic = rospy.get_param('~stop_flag_topic')

  
  self.stop_flag=rospy.Publisher("%s"%self.stop_flag_topic, String ,queue_size=1)
  self.marker_pub=rospy.Publisher("detector_marker", Marker ,queue_size=1)
  
  print 'radiu',self.radius
  
 def __init__(self):
  self.define()
  self.map_data()
  rospy.Subscriber('/scan', LaserScan, self.laser_cb)
  rospy.Subscriber('detector_recieved', String, self.feedback_cb)
  rospy.Subscriber('turtlebot_position_in_map',Pose,self.pose_cb)
  rospy.spin()

 def map_data(self):
  self.map=rospy.wait_for_message('map',OccupancyGrid)
  
  self.statice_area()
   
  #geohash算法区域划分
  width=self.map.info.width
  height=self.map.info.height
  map_origin=self.map.info.origin
  self.geohash(self.static_area, width, height, map_origin)

  #self.visual_markers()####################visual_test
  
 def statice_area(self):
  self.static_area=maplib.get_effective_point(self.map)[1]

  #self.static_area_makers()####################visual_test  
    
 def visual_markers(self):
  color=ColorRGBA()
  scale=Point()
  scale.x=0.05
  scale.y=0.05
  color.r=0.0
  color.g=1.0
  color.b=0.0
  color.a=0.5
  self.line_marker=self.visual_test(self.cut_points, Marker.LINE_LIST, color, scale)#标记出区域划分（testing）
  
  color=ColorRGBA()
  scale=Point()
  scale.x=0.1
  scale.y=0.1
  color.r=0.0
  color.g=0.0
  color.b=1.0
  color.a=1.0
  self.centre_points_marker=self.visual_test(self.centre_points,Marker.POINTS, color, scale)
  
 def static_area_makers(self):
  color=ColorRGBA()
  scale=Point()
  scale.x=0.05
  scale.y=0.05
  color.r=1.0
  color.g=1.0
  color.b=0.0
  color.a=1.0
  
  self.points_marker=self.visual_test(self.static_area,Marker.POINTS, color, scale)
  #self.static_area=[point1,point2,point3...]
 
 def LaserDataMarker(self,LaserData): ####################visual_test
  color=ColorRGBA()
  scale=Point()
  scale.x=0.04
  scale.y=0.04
  color.r=0.0
  color.g=2.0
  color.b=0.0
  color.a=1.0
  
  self.laser_points_marker=self.visual_test(LaserData, Marker.POINTS, color, scale)
  
 
 #地图区域划分 
 def geohash(self, data, width, height, map_origin):
  (X,Y)=(1,1)
  pose_mean=Point()
  pose_mean.x=(width/2)*self.map.info.resolution*X+map_origin.position.x
  pose_mean.y=(height/2)*self.map.info.resolution*Y+map_origin.position.y
   
  self._map_divide_store(width, height, data, map_origin.position, pose_mean, (X,Y))
 
 ###########################################
  #testing 
  #test_point=Point()
  #test_point.x=-1.15
  #test_point.y=5.20
  #test_data=[test_point]
  #self._map_divide_store(width, height, test_data, map_origin.position, pose_mean, (X,Y))
 ############################################
 
 
 #地图划分区域存储 
 def _map_divide_store(self, width, height, data, map_origin, pose_mean, (X,Y)):
 
  root=self.create_btree(data, width, height, map_origin, pose_mean, (X,Y))

  rospy.loginfo( 'start loading tree' )
  for i in data:
   self.load(root, i)
   
  self.root=root
  
  #for k in root['EN']['EN']['EN'].keys():
   #if k=='map':
    #print k,len(root['EN']['EN']['EN'][k])

  #print root['EN']['EN']['EN']['EN']['map']
  
 #创建树 
 def create_btree(self,data, width, height, map_origin, pose_mean, (X,Y)):
  root={'map':[data], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
  rospy.loginfo( 'start building tree' )
  for i in data:
   self.insert(root, i, width, height, map_origin, (X,Y))
  return root

 #存入值
 def insert(self, root, data, width, height, map_origin, (X,Y)):

  pose_mean=Point()
  pose_mean.x=abs((width/2)*self.map.info.resolution)*X+map_origin.x
  pose_mean.y=abs((height/2)*self.map.info.resolution)*Y+map_origin.y
  
  linex_start=Point()
  linex_end=Point()
  liney_start=Point()
  liney_end=Point()
  
  linex_start.x=pose_mean.x
  linex_start.y=map_origin.y
  linex_end.x=pose_mean.x
  linex_end.y=map_origin.y+self.map.info.resolution*height*Y

  liney_start.y=pose_mean.y
  liney_start.x=map_origin.x
  liney_end.y=pose_mean.y  
  liney_end.x=map_origin.x+self.map.info.resolution*width*X
  
  line=[linex_start,linex_end,liney_start,liney_end]
  self.cut_points+=line
  self.centre_points.append(pose_mean)
  
  #print (X,Y),pose_mean.x,pose_mean.y,' data:',data.x,data.y
  
   
  if width<=self.mim_space or height<=self.mim_space:
   pass
   
  else:
   #print 'width, height: ', width, height
   if data.x>=pose_mean.x and data.y>=pose_mean.y:#EN 的条件
    if root['EN'] != None:
     pass
    elif root['EN']==None:
     root['EN']={'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree EN errro' )
    (X,Y)=(1,1)
    width/=2
    height/=2
    self.insert(root['EN'], data, width, height, pose_mean, (X,Y))
     
   elif data.x>=pose_mean.x and data.y<pose_mean.y:#ES 的条件
    if root['ES']!= None:
     pass
    elif root['ES']==None:
     root['ES']={'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree ES errro' )
    (X,Y)=(1,-1)
    width/=2
    height/=2
    self.insert(root['ES'], data, width, height, pose_mean, (X,Y))
    
   elif data.x<pose_mean.x and data.y>=pose_mean.y:#WN 的条件
    if root['WN']!= None:
     pass
    elif root['WN']==None:
     root['WN']={'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree WN errro' )
    (X,Y)=(-1,1)
    width/=2
    height/=2
    self.insert(root['WN'], data, width, height, pose_mean, (X,Y))

   elif data.x<pose_mean.x and data.y<pose_mean.y:#WS 的条件
    if root['WS']!= None:
     pass
    elif root['WS']==None:
     root['WS']={'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
    else:
     rospy.loginfo( 'create tree WS errro' )
    (X,Y)=(-1,-1)
    width/=2
    height/=2
    self.insert(root['WS'], data, width, height, pose_mean, (X,Y))
    
   else:
    rospy.loginfo( 'error during building tree' )

 def load(self,root,i):
  if root['EN']==None and root['ES']==None and root['WN']==None and root['WS']==None:
  #LEAF
   root['map'].append(i)

#1   
  if root['EN']!=None and root['ES']==None and root['WN']==None and root['WS']==None:
  # EN
   self.load(root['EN'],i)
   
  if root['EN']==None and root['ES']!=None and root['WN']==None and root['WS']==None:
  # ES
   self.load(root['ES'],i)
   
  if root['EN']==None and root['ES']==None and root['WN']!=None and root['WS']==None:
  #WN
   self.load(root['WN'],i)
   
  if root['EN']==None and root['ES']==None and root['WN']==None and root['WS']!=None:
  #WS
   self.load(root['WS'],i)

#2
  if root['EN']!=None and root['ES']!=None and root['WN']==None and root['WS']==None:
   #EN ES
   if root['pose_mean'].x<=i.x:
    if root['pose_mean'].y>i.y:
     self.load(root['ES'],i)
    elif root['pose_mean'].y<=i.y:
     self.load(root['EN'],i)
    else:
     rospy.loginfo('EN ES1 error point not in position')

       
  if root['EN']!=None and root['ES']==None and root['WN']!=None and root['WS']==None:
  # EN WN
   if root['pose_mean'].y<=i.y:
    if root['pose_mean'].x<=i.x:
     self.load(root['EN'],i)
    elif root['pose_mean'].x>i.x:
     self.load(root['WN'],i)
    else:
     rospy.loginfo('EN WN1 error point not in position')

       
  if root['EN']!=None and root['ES']==None and root['WN']==None and root['WS']!=None:
  # EN WS
   if root['pose_mean'].x<=i.x and root['pose_mean'].y<i.y:
    self.load(root['EN'],i)
   elif root['pose_mean'].x>i.x and root['pose_mean'].y>i.y:
    self.load(root['WS'],i)
   else:
    rospy.loginfo('EN WS error point not in position')
  
  if root['EN']==None and root['ES']!=None and root['WN']!=None and root['WS']==None:
  # ES WN
   if root['pose_mean'].x<=i.x and root['pose_mean'].y>i.y:
    self.load(root['ES'],i)
   elif root['pose_mean'].x>i.x and root['pose_mean'].y<=i.y:
    self.load(root['WN'],i)
   else:
    rospy.loginfo('ES WN error point not in position')
   
  if root['EN']==None and root['ES']!=None and root['WN']==None and root['WS']!=None:
  # ES WS
   if root['pose_mean'].y>i.y:
    if root['pose_mean'].x>i.x:
     self.load(root['WS'],i)
    elif root['pose_mean'].x<=i.x:
     self.load(root['ES'],i)
    else:
     rospy.loginfo('ES WS1 error point not in position')

         
  if root['EN']==None and root['ES']==None and root['WN']!=None and root['WS']!=None:
  # WN WS
   if root['pose_mean'].x>=i.x:
    if root['pose_mean'].y<=i.y:
     self.load(root['WN'],i)
    elif root['pose_mean'].y>i.y:
     self.load(root['WS'],i)
    else:
     rospy.loginfo('WN WS1 error point not in position')

   
#3
  if root['EN']!=None and root['ES']!=None and root['WN']!=None and root['WS']==None:
  #EN ES WN
   if i.x>=root['pose_mean'].x and i.y>=root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x>=root['pose_mean'].x and i.y<root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x<root['pose_mean'].x and i.y>=root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   else:
    #rospy.loginfo('EN ES WN error point not in position')
    return None
    
  if root['EN']!=None and root['ES']!=None and root['WN']==None and root['WS']!=None:
  # EN ES WS
   if i.x>=root['pose_mean'].x and i.y>=root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x>=root['pose_mean'].x and i.y<root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x<root['pose_mean'].x and i.y<root['pose_mean'].y:#WS 的条件
    self.load(root['ES'],i)
    
   else:
    #rospy.loginfo('EN ES WS error point not in position')
    return None

  if root['EN']!=None and root['ES']==None and root['WN']!=None and root['WS']!=None:
  # EN WN WS
   if i.x>=root['pose_mean'].x and i.y>=root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x<root['pose_mean'].x and i.y>=root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   elif i.x<root['pose_mean'].x and i.y<root['pose_mean'].y:#WS 的条件
    self.load(root['WS'],i)
    
   else:
    #rospy.loginfo('EN WN WS error point not in position')
    return None
    
  if root['EN']==None and root['ES']!=None and root['WN']!=None and root['WS']!=None:
  # ES WN WS
   if i.x>=root['pose_mean'].x and i.y<root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x<root['pose_mean'].x and i.y>=root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   elif i.x<root['pose_mean'].x and i.y<root['pose_mean'].y:#WS 的条件
    self.load(root['WS'],i)
    
   else:
    #rospy.loginfo('ES WN WS error point not in position')
    return
#4  
  if root['EN']!=None and root['ES']!=None and root['WN']!=None and root['WS']!=None:
   # EN ES WN WS
   if i.x>=root['pose_mean'].x and i.y>=root['pose_mean'].y:#EN 的条件
    self.load(root['EN'],i)
    
   elif i.x>=root['pose_mean'].x and i.y<root['pose_mean'].y:#ES 的条件
    self.load(root['ES'],i)
    
   elif i.x<root['pose_mean'].x and i.y>=root['pose_mean'].y:#WN 的条件
    self.load(root['WN'],i)
    
   elif i.x<root['pose_mean'].x and i.y<root['pose_mean'].y:#WS 的条件
    self.load(root['WS'],i)
   
   else:
    rospy.loginfo('EN ES WN WS error point not in position')
    return    
     
 #查询值 
 def read(self, root, i):
  #rospy.loginfo('checking the map')
  if i.x<root['pose_mean'].x and i.y>=root['pose_mean'].y:#WN 的条件
   if root['WN']==None:
    for j in root['map']:
     conditional=(abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))<(self.radius,self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['WN'], i)
   
  elif i.x<root['pose_mean'].x and i.y<root['pose_mean'].y:#WS 的条件
   if root['WS']==None:
    for j in root['map']:
     conditional=(abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))<(self.radius,self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['WS'], i)
   
  elif i.x>=root['pose_mean'].x and i.y>=root['pose_mean'].y:#EN 的条件
   if root['EN']==None:
    for j in root['map']:
     conditional=(abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))<(self.radius,self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['EN'], i)
   
  elif i.x>=root['pose_mean'].x and i.y<root['pose_mean'].y:#ES 的条件
   if root['ES']==None:
    for j in root['map']:
     conditional=(abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))<(self.radius,self.radius)
     if conditional:
      #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
      return True
     else:
      continue
   else:
    return self.read(root['ES'], i)
   
  else:
   rospy.loginfo('查询值  error point not in position')
   

  
 def pose_cb(self,data):
  self.pose=data
  self.oriation_angle=self.Q2A([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
  #发布visual_test的结果
  try:
   #self.marker_pub.publish(self.points_marker) #marker out all static area
   #self.marker_pub.publish(self.line_marker) #marker out divie line
   #self.marker_pub.publish(self.centre_points_marker)
   self.marker_pub.publish(self.laser_points_marker)####################visual_test
  except:
   pass
     
 #def odom_cb(self,data):
   #pass
   
 def feedback_cb(self,data):
  self.feedback=data.data

  
 def laser_cb(self,data):
  self.now=rospy.get_rostime()
  if self.feedback=='recieved':
   self.feedback=''
   #rospy.loginfo('sleeping')
   rospy.sleep(0.5)
  if data.header.stamp.secs==self.now.secs:
   #print 'time qt', data.header.stamp.secs==self.now.secs
   LaserData=[]
   count=0
   for i in data.ranges:
    #print 'get total points:', count
    if 0.0<i<0.8:
     if data.angle_min+data.angle_increment*count<=data.angle_max+data.angle_increment: 
      point=Point()
      angle=data.angle_min+data.angle_increment*count+self.oriation_angle
      point.x=i*numpy.cos(angle)+self.pose.position.x-0.1
      point.y=i*numpy.sin(angle)+self.pose.position.y+0.05
      #print 'i:', i, type(point.x), point.x ,type(numpy.nan) ,point.x==numpy.nan
      LaserData.append(point)
     else:
      rospy.loginfo( 'out of range' )
    else:
     pass
    count+=1
   
   if self.check(LaserData): #判断是否在误差许可之内为地图上已知点
    rospy.loginfo( 'obstacle detected' )
    self.stop_flag.publish('stop')
   else:
    #print False
    pass
   
   self.LaserDataMarker(LaserData)####################visual_test

#检查镭射是否为地图上的静态障碍物（eg，wall door etc.）
 def check(self,data):
  trigger=0
  if len(data)<80:
   pass
  else:
   #print 'data length',len(data)
   for i in data:
    result=self.read(self.root,i)
    #print result,i.x,i.y
    if result is True:
     #print result,i.x,i.y
     trigger+=1
     #print trigger
    else:
     pass
   #print 'trigger',trigger
   if trigger>=0.8*len(data):
    return False
   else:
    return True

# 视觉显示检测结果
 def visual_test(self,data,Type,color,scale):#data=[point1,point2,point3...]###################visual_test
#plot POINTS
  #print len(data),data[0],data[1]
  if Type==Marker.POINTS:
   #print 'pub POINTS Marker'
   point_marker=Marker()
   point_marker.header.frame_id='/map'
   point_marker.header.stamp=rospy.Time.now()
   point_marker.ns='detector_visual_test'
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
   
#plot LINE_LIST  
  if Type==Marker.LINE_LIST:
   #print 'pub LINE_LIST Marker'
   line_marker=Marker()
   line_marker.header.frame_id='/map'
   line_marker.header.stamp=rospy.Time.now()
   line_marker.ns='detector_visual_test'
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


 def Q2A(self,quat):
  rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
  return rot.GetRPY()[2]
  
  
if __name__ == '__main__':
 rospy.init_node("detector", anonymous=True)
 try:
  rospy.loginfo ("initialization system")
  laser_point()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")
