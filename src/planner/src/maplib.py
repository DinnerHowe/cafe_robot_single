#!/usr/bin/env python  
#coding=utf-8

""" 
marker's utils tools
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""
#import collections
from geometry_msgs.msg import Point
import PyKDL
import numpy
 
 
 ##转化坐标系
def Trans(self, quaterion, Mapdata):
 theata = quat_to_angle([quaterion.x,quaterion.y,quaterion.z,quaterion.w])
 Tdata = []
 Tpoint=Point()
 for i in Mapdata:
  Tpoint.x = i.x * numpy.cos(theata) + i.y * numpy.sin(theata)
  Tpoint.y = i.y * numpy.cos(theata) - i.x * numpy.sin(theata)
  Tdata.append(copy.deepcopy(Tpoint))
 return Tdata

#return RPY angle in rad
def quat_to_angle(quat):
 rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
 return rot.GetRPY()[2]

#return (x,y,z,w)
def angle_to_quat(angle):#in rad
 rot = PyKDL.Rotation.RotZ(angle)
 return rot.GetQuaternion()

#返回机器人的data格数[num]
def position_num(data,position):
 centremap_cell=map_center_cell(data.info.origin, data.info.resolution)
 pose_x = centremap_cell[0]+int(round(position.x/data.info.resolution))
 pose_y = centremap_cell[1]+int(round(position.y/data.info.resolution))
 num = pose_y * data.info.width + pose_x
 return num

#返回有效区域的坐标集
def get_effective_point(data):
 map_matrix=map_matrix_ranger(data)
 
 #print map_matrix
 
 clear_area,block_area=[],[]
 width=data.info.width
 height=data.info.height
 resolution=data.info.resolution
 map_origin=data.info.origin

 clear=Point()
 block=Point()
 
 centremap_cell=map_center_cell(map_origin,resolution)
 #print centremap_cell
 
 for y in range(height):
  for x in range(width):
   if map_matrix[y][x]==0:
    clear.x=(x-centremap_cell[0])*resolution
    clear.y=(y-centremap_cell[1])*resolution
    clear_area.append(clear)
   if map_matrix[y][x]==100:
    block.x=(x-centremap_cell[0])*resolution
    block.y=(y-centremap_cell[1])*resolution
    block_area.append(block)
    #print block,'\n'
    #print block_area

   clear=Point()
   block=Point()
 #print 'block_area',len(block_area)
 return [clear_area,block_area]
 
#返回地图矩阵
def map_matrix_ranger(data):
 height=data.info.height
 width=data.info.width
 #map_matrix=collections.deque()
 map_matrix=[]
 for i in range(height):
  map_width=[]
  #map_width=collections.deque()
  for j in range(width):
   map_width.append(data.data[j+width*i])
  map_matrix.append(map_width)
 return map_matrix
 
 #cell(0,0) in matrixs frame
def map_center_cell(map_origin,resolution):
 centremap_cell=[0-int(round(map_origin.position.x/resolution)),0-int(round(map_origin.position.y/resolution))]
 return centremap_cell

 
