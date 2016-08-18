#!/usr/bin/env python
#coding=utf-8
"""
可操作，可移动的障碍物的库文件
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy,copy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarkerControl


from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerInit
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate
from visualization_msgs.msg import MenuEntry
from threading import Lock

from geometry_msgs.msg import PointStamped
  
 
###############################
##### Interactive Server ######
###############################
class Server:
 def __init__(self,root_topic):
  self.locker = Lock()
  self.obstacle=None
  self.root_topic='/'+root_topic
  self.update_ = rospy.Publisher(self.root_topic+"/update", InteractiveMarkerUpdate ,queue_size=100)
  self.init_ = rospy.Publisher(self.root_topic+'/update_full', InteractiveMarkerInit, queue_size=100)
  self.projection_ = rospy.Publisher(self.root_topic+'/projection', PointStamped, queue_size=1)
    
  self.seq_num = 0
  self.server_id = self.root_topic
  
  self.Current = InteractiveMarkerUpdate()
  #self.Candidate = InteractiveMarkerFeedback()
  self.CurrentP = PointStamped()
  
  rospy.Subscriber(self.root_topic+'/feedback', InteractiveMarkerFeedback, self.RvizFeedback, queue_size=10)
  rospy.Timer(rospy.Duration(0.1), self.standbycb)
  self.PublishInit()
  
 def AddObstacle(self, obstacle):
  with self.locker:
   self.obstacle=obstacle
   self.Current.type = InteractiveMarkerFeedback.KEEP_ALIVE
   self.Current.server_id = self.server_id
   self.Current.seq_num = self.seq_num
   self.Current.markers.append(self.obstacle)
 
 def start(self):
  with self.locker:
   if self.obstacle == None:
    return
   else:
    self.Current.type = InteractiveMarkerUpdate.UPDATE
    self.Current.seq_num = self.seq_num
    self.Current.markers.append(copy.deepcopy(self.obstacle))

    self.seq_num += 1   
    self.PublishCB(self.Current)
    self.PublishInit()
  
 #sever for moving object
 def MovingServer(self,feedback):
  #with self.locker:
  self.Current.type = InteractiveMarkerUpdate.UPDATE
  self.Current.seq_num = self.seq_num
  self.Current.markers.append(copy.deepcopy(self.obstacle))
   
  position = InteractiveMarkerPose()
  position.header.seq = self.seq_num
  position.header.stamp = rospy.Time.now()
  position.header.frame_id = 'map'
  position.name = self.obstacle.name
  position.pose = feedback.pose

  self.Current.poses.append(copy.deepcopy(position))

  self.seq_num += 1   
  self.PublishCB(self.Current)
  self.PublishInit()
  self.Projection(feedback)
   
 #投影到地图上
 def Projection(self,feedback):
  #print 'into Projection'
  self.CurrentP.point = feedback.pose.position
  self.projection_.publish(self.CurrentP)
        
        
 def RvizFeedback(self, feedback):
  with self.locker: 
   #self.Candidate = feedback
   #rospy.loginfo('\nfeedback: ' + '%s'%feedback)
   self.MovingServer(feedback)

   
 def standbycb(self, event):
  with self.locker:
   standby=InteractiveMarkerUpdate()
   standby.type = InteractiveMarkerUpdate.KEEP_ALIVE
   self.PublishCB(standby)
   
 def PublishCB(self,data):
  data.server_id = self.server_id
  data.seq_num = self.seq_num
  self.update_.publish(data)
  
 def PublishInit(self):
  initData = InteractiveMarkerInit()
  initData.server_id = self.server_id
  initData.seq_num = self.seq_num
  if self.obstacle == None:
   #print 'self.obstacle == None\n',initData
   initData.markers=[]
   self.init_.publish(initData)
  else:
   initData.markers.append(self.obstacle)
   #print 'self.obstacle != None\n',initData
   self.init_.publish(initData)

###############################
#########  obstacles ##########
###############################
class obstacles():
 def __init__(self):
 
  root_topic = self.define()
  self.server = Server(root_topic)
  position =  rospy.wait_for_message('/clicked_point', PointStamped)
  self.marker(root_topic, position.point)
  self.server.start()
  rospy.spin()
  
 #initial define 
 def define(self):
  self.obstacle = InteractiveMarker()
  self.obstacle.header.frame_id = 'map'
  self.obstacle.name = 'obstacles'
  self.obstacle.description = 'test_marker'
  self.obstacle.pose.position = position
  #self.obstacle.pose.position.z = 0.0
  self.obstacle.pose.position.z = 0.5 #for testing
  self.obstacle.scale = 0.6
  root_topic = 'test_obstacles'
  return root_topic

 def InsertMenue(self, ID, parent_id, title):
  menue = MenuEntry()
  menue.id = ID
  menue.parent_id = parent_id
  menue.title = title
  menue.command_type = MenuEntry.FEEDBACK
  return menue
  
 def MessUnit(self):
  unit=Marker()
  unit.type = Marker.CUBE
  unit.scale.x = 0.45
  unit.scale.y = 0.45
  unit.scale.z = 0.45
  unit.color.r = 1.0
  unit.color.g = 1.0
  unit.color.b = 0.5
  unit.color.a = 1.0
  return unit
 
 def MessControl(self, unit, mode):
  control=InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.y = 1
  control.interaction_mode= mode
  control.always_visible=True
  control.markers.append(copy.deepcopy(unit))
 
 #the moving object  
 def marker(self,root_topic,position):
  
  unit = self.MessUnit() 
  control = self.MessControl(unit, InteractiveMarkerControl.MOVE_PLANE)
  
  self.obstacle.pose.position.z += unit.scale.z/2

  self.obstacle.controls.append(copy.deepcopy(control))
  
  self.server.AddObstacle(self.obstacle)



if __name__=='__main__':
 rospy.init_node('test_obstacles')
 try:
  rospy.loginfo ("initialization system")
  obstacles()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
