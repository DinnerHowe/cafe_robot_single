#!/usr/bin/env python
#coding=utf-8
"""
Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 
"""
import numpy as np
import rospy
import matplotlib.pyplot as plt
from matplotlib import colors

class grid_map():
 def define(self):
  self.color_map = colors.ListedColormap(["white", "gray", "red", "green"])
  self.button_define={1:'gray',2:'red',3:'green'}
  self.manue=['gray','red','green']
  self.scale=50
  self.release=True
  self.click=False
  
 # The grid initialization function
 def init_grid(self,size):
  matrix=np.zeros([size,size],int)
  return matrix

 def plot_grid(self):
  plt.ion() 
  self.fig,self.ax=plt.subplots(figsize=(100,100))
  plt.title("interactive test")
  self.ax=plt.gca()
  self.ax.grid(True)
  self.ax.imshow(self.grid, interpolation='none', cmap=self.color_map)


 def mouse_even(self):
  #print 'mouse_event'
  cid=self.fig.canvas.mpl_connect('button_press_event', self.click_evencb)
  mid=self.fig.canvas.mpl_connect('motion_notify_event', self.move_evencb)
  rid=self.fig.canvas.mpl_connect('button_release_event', self.release_evencb) 
  plt.show(block=True)
  
 def click_evencb(self,event):
  if event.xdata!=None and event.ydata!=None:
   print('clicked: x=%d, y=%d, xdata=%f, ydata=%f, button=%d, %s' %(round(event.xdata),round (event.ydata), event.xdata, event.ydata,event.button,self.button_define[event.button]))
   if 0<=round(event.xdata)<=self.scale and 0<=round(event.ydata)<=self.scale:
    self.click=True
    self.release=False
    self.button=event.button
    self.y=int(round(event.ydata))
    self.x=int(round(event.xdata))
    self.grid[self.y][self.x]=self.button
    self.ax.imshow(self.grid, interpolation='none', cmap=self.color_map)
    plt.draw()
    
 def move_evencb(self,event):
  if event.xdata!=None and event.ydata!=None:
   #print event.xdata,event.ydata
   self.y=int(round(event.ydata))
   self.x=int(round(event.xdata))
   if 0<=self.x<=self.scale and 0<=self.y<=self.scale:
    if self.click and not self.release:
     #print 'move: x=%s, y=%s'%(event.xdata,event.ydata)
     self.grid[self.y][self.x]=self.button
     self.ax.imshow(self.grid, interpolation='none', cmap=self.color_map)
     plt.draw()
     
 def release_evencb(self, event):
  if event.xdata!=None and event.ydata!=None:
   if 0<=round(event.xdata)<=self.scale and 0<=round(event.ydata)<=self.scale:
    if self.button==event.button:
     self.release=True
     self.click=False
    #print 'move: x=%s, y=%s'%(event.xdata,event.ydata)
   
 def keyboard_even(self):
  while not rospy.is_shutdown():
   row=int(raw_input('input row No.:'))
   column=int(raw_input('input column No.:'))
   number=int(raw_input('input color（0:white，1:gray，2:red，3:green）:'))
   self.grid[row][column]=number
   self.ax.imshow(self.grid, interpolation='none', cmap=self.color_map)
   #print self.grid
   plt.draw()

 def __init__(self):
  self.define()
  self.grid = self.init_grid(self.scale)  
  self.plot_grid()
  
  try:
   #keyboard event
   #self.keyboard_even()   

   #mouse event
   self.mouse_even()
  except:
   pass

  
if __name__=='__main__':
 #rospy.init_node('grid_map')
 try:
  rospy.loginfo ("initialization system")
  grid_map()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
