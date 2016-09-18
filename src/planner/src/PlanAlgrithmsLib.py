#!/usr/bin/env python
#coding=utf-8
""" 
plan algrithms

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import numpy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import maplib

# 产生一个朝向当前目标点的角度
def Angle(sub_point,pre_point):
 if sub_point.y-pre_point.y>0:
  if sub_point.x<pre_point.x:
   #print 'sub_point.y-pre_point.y>0 && sub_point.x<pre_point.x'
   angle=numpy.pi-abs(numpy.atan((sub_point.y - pre_point.y)/(sub_point.x - pre_point.x)))
  if sub_point.x==pre_point.x:
   #print 'sub_point.y-pre_point.y>0 && sub_point.x==pre_point.x'
   angle=numpy.pi/2
  if sub_point.x>pre_point.x:
   #print 'sub_point.y-pre_point.y>0 && sub_point.x==sub_point.x>pre_point.x'
   angle=numpy.atan((sub_point.y - pre_point.y)/(sub_point.x - pre_point.x))
   
 if sub_point.y-pre_point.y<0:
  if sub_point.x<pre_point.x:
   #print 'sub_point.y-pre_point.y<0 && sub_point.x<pre_point.x'
   angle=-(numpy.pi-abs(numpy.atan((sub_point.y - pre_point.y)/(sub_point.x - pre_point.x))))
  if sub_point.x==pre_point.x:
   #print 'sub_point.y-pre_point.y<0 && sub_point.x==pre_point.x'
   angle=-numpy.pi/2
  if sub_point.x>pre_point.x:
   #print 'sub_point.y-pre_point.y<0 && sub_point.x>pre_point.x'
   angle=-abs(numpy.atan((sub_point.y - pre_point.y)/(sub_point.x - pre_point.x)))
  
 if sub_point.y-pre_point.y==0:
  if sub_point.x>pre_point.x:
   #print 'sub_point.y-pre_point.y==0 && sub_point.x>pre_point.x'
   angle=0.0
  if sub_point.x<pre_point.x:
   #print 'sub_point.y-pre_point.y==0 && sub_point.x<pre_point.x'
   angle=numpy.pi

 return angle

#使用网上Christopher Chu的JPS库测试planner 替代amcl的情况
def Simple_JPS_Planner(_map, start, goal):
 #_map : [] grid map
 #start: position
 #goal : pose
 #Open_list = [current jump point1: {'LeftDown': False, 'LeftUp': False, 
 #                     'RightDown': False, 'RightUp': False, 
 #                     'Right': False, 'Left': False,
 #                     'Up': False, 'Down': False},
 #             current jump point2: {'LeftDown': False, 'LeftUp': False, 
 #                     'RightDown': False, 'RightUp': False, 
 #                     'Right': False, 'Left': False,
 #                     'Up': False, 'Down': False}]
 import JPS 
 width = _map.info.width
 height = _map.info.height
 start_num = maplib.position_num(_map, start)
 end_num   = maplib.position_num(_map, goal.position)
 DENSITY = 5
 raw_field = JPS_GenerateMap(_map.data, width, height)
 field = JPS.generate_field(raw_field, (lambda cell: True if cell > DENSITY else False), True)
 (start_row, start_colomn, end_row, end_colomn) = Generate_Cor(start_num, end_num)
 path = JPS.jps(field, start_row, start_colomn, end_row, end_colomn)
 path = JPS.get_full_path(path)
 print len(path)
 return path
 
 
def JPS_Planner(_map, start, goal):
 #_map : [] grid map
 #start: position
 #goal : pose
 #Open_list = [current jump point1: {'LeftDown': False, 'LeftUp': False, 
 #                     'RightDown': False, 'RightUp': False, 
 #                     'Right': False, 'Left': False,
 #                     'Up': False, 'Down': False},
 #             current jump point2: {'LeftDown': False, 'LeftUp': False, 
 #                     'RightDown': False, 'RightUp': False, 
 #                     'Right': False, 'Left': False,
 #                     'Up': False, 'Down': False}]
 width = _map.info.width
 height = _map.info.height
 start_num = maplib.position_num(_map, start)
 end_num   = maplib.position_num(_map, goal.position)
 Map_ = JPS_GenerateMap(_map.data, width, height)
 Open_list = {start_num:{'LeftDown' : False, 'LeftUp' : False, 'RightDown' : False, 'RightUp' : False, 'Right' : False, 'Left' : False, 'Up' : False, 'Down' : False}}
 
 visited_list = []
 #Open_list = Jumping(Map_, Open_list, start_num, end_num )
 
 # jump point search algrithm 
def Jumping(Map_, Open_list, start_num, end_num ):
 (Victor_X, Victor_Y) = Tend_Victor(start_num, end_num, Map_, width)

 if Victor_X > 0 and Victor_Y > 0: #右下
  pass
 if Victor_X > 0 and Victor_Y < 0: #右上
  pass
 if Victor_X < 0 and Victor_Y > 0: #左下
  pass
 if Victor_X < 0 and Victor_Y < 0: #左上
  pass
 if Victor_X > 0 and Victor_Y == 0: #正右
  dX = 1
  dY = 0
  current = StraightMove(dX, dY, start, goal)
  Open_list.append(current)
  JPS_Planner(_map, current, goal) 
  
 if Victor_X < 0 and Victor_Y == 0: #正左
  pass
 if Victor_X == 0 and Victor_Y > 0: #正下
  pass
 if Victor_X == 0 and Victor_Y < 0: #正上
  pass

 if Victor_X == 0 and Victor_Y == 0: #起点终点重合
  Open_list.append(goal)
  return Open_list
 
#def StraightMove(dX, dY, start, goal):
  
  
# generate coordiante in map matrix from start_num and end_num of map list   
def Generate_Cor(start_num, end_num):
 start_row = int(numpy.ceil(float(start_num) / float(width)))
 start_colomn = start_num % width
 
 end_row = int(numpy.ceil(float(end_num) / float(width)))
 end_colomn = end_num % width
 return (start_row, start_colomn, end_row, end_colomn)

# calculate a path motion tend victor, it could help path moving  
def Tend_Victor(start_num, end_num, Map_, width):
 (start_row, start_colomn, end_row, end_colomn) = Generate_Cor(start_num, end_num)
 Victor_Y = end_row - start_row
 Victor_X = end_colomn - start_colomn
 return (Victor_X, Victor_Y)

# generate a map matrix
def JPS_GenerateMap(data, width, height):
 return numpy.array(data).reshape(height, width)


