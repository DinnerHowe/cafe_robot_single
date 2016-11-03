#!/usr/bin/env python
#coding=utf-8
"""
用来转变当前的导航模式
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import copy
from threading import Lock
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MenuEntry
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import Marker
import InteractiveMarkerLib  
from std_msgs.msg import String

#import traceback

#from interactive_markers.interactive_marker_server import *
#from interactive_markers.menu_handler import *

class ModelTrans():
 def __init__(self):
  self.define()
  #print 1
  self.InitMenu()
  #print 2
  #print 3
  self.MakeMuneObject(self.MenuMarkerName)
  rospy.Subscriber('/turtlebot_position_in_map', Pose, self.OdomCB)
  #print 4
  self.MenuApplyChanges(self.MenuMarkerName, self.server)  
  #print 5
  self.server.applyChanges()
  #print 6
  #traceback.print_exc() 
  rospy.spin()
  print 7
  
  
 def OdomCB(self, data):
  #with self.locker:
   self.odom = data
   self.UpdateMuneObject(self.MenuMarkerName , self.odom)

 def UpdateMuneObject(self, MenuName, MenuPose):
  
  
 def MakeMuneObject(self, MenuName, MenuPose):
 
  MenuInteractiveMarker = InteractiveMarker()
  MenuInteractiveMarker.name = MenuName
  MenuInteractiveMarker.header.frame_id = self.frame_id
  MenuInteractiveMarker.pose.position.z += self.MenuHight
  MenuInteractiveMarker.scale = self.MenuScale
  
  MenuControl = InteractiveMarkerControl()
  MenuControl.interaction_mode = InteractiveMarkerControl.MENU
  MenuControl.always_visible = False
  
  MenuMarker = Marker()
  
  MenuMarker.type = Marker.ARROW
  MenuMarker.scale.x = MenuInteractiveMarker.scale * 2
  MenuMarker.scale.y = MenuInteractiveMarker.scale * 0.45
  MenuMarker.scale.z = MenuInteractiveMarker.scale * 0.45
  MenuMarker.color.r = 0.5
  MenuMarker.color.g = 0.5
  MenuMarker.color.b = 0.5
  MenuMarker.color.a = 1.0
  MenuMarker.pose = MenuPose
    
  MenuControl.markers.append(MenuMarker)
  
  MenuInteractiveMarker.controls.append(MenuControl)
  
  #print '###################MenuInteractiveMarker info:\n', MenuInteractiveMarker
  
  self.server.insert(MenuInteractiveMarker)
  rospy.loginfo('insert Menu Marker Object')
  
 
 def MenuApplyChanges(self, MenuName, server):
  self.menu.ApplyMenuChanges( server, MenuName )
  #self.menu.apply(server, MenuName)


 def InitMenu(self):
  # insert parent entry
  
  ParentItemID = self.menu.InsertMenu(ChirldTitle = self.ParentItemName)
  #ParentItemID = self.menu.insert(self.ParentItemName)
  print 'insert ParentEntry: ', self.ParentItemName, ParentItemID

  self.MenuItemID = ParentItemID
  for ChirldItem in self.ChirldItemName:
   ChirldItemTitle = 'Mode ' + ChirldItem
   # insert chirld entry
   
   ChirldItemID = self.menu.InsertMenu(ChirldTitle = ChirldItemTitle, ParentTitleID = ParentItemID, Callback = self.modeCb)
   #ChirldItemID = self.menu.insert(ChirldItemTitle, parent = ParentItemID, callback = self.modeCb)

   print 'insert ChirldEntry: ', str(ChirldItemTitle), ChirldItemID

   # uncheck all items   
   result = self.menu.SetEntryCheckState( ChirldItemID, InteractiveMarkerLib.MenuHandler.UNCHECKED)
   #self.menu.setCheckState( ChirldItemID, InteractiveMarkerLib.MenuHandler.UNCHECKED)
   if result:
    rospy.loginfo('Setting ' + str(ChirldItemID) + ' State To UNCHECKED')
   
      
  #default to set single task module
  result = self.menu.SetEntryCheckState( ChirldItem, InteractiveMarkerLib.MenuHandler.CHECKED )
  #self.menu.setCheckState( ChirldItemID, InteractiveMarkerLib.MenuHandler.UNCHECKED)
  if result:
   rospy.loginfo('Setting ' + ChirldItemID + ' State To CHECKED')
 
 
 def modeCb(self, feedback):
  
  self.menu.SetEntryCheckState(self.MenuItemID, InteractiveMarkerLib.MenuHandler.UNCHECKED)
  #self.menu.setCheckState( self.MenuItemID, InteractiveMarkerLib.MenuHandler.UNCHECKED)
  
  Itemid = feedback.menu_entry_id
  self.MenuItemID = Itemid
  
  self.menu.SetEntryCheckState(Itemid, InteractiveMarkerLib.MenuHandler.CHECKED)
  #self.menu.setCheckState( Itemid, InteractiveMarkerLib.MenuHandler.CHECKED)
  
  rospy.loginfo("Switching to menu entry : " + self.ChirldItemName[self.MenuItemID - 2])
  self.ModePub.publish(self.ChirldItemName[self.MenuItemID - 2])
  
  self.menu.ReApplyMenuChanges( self.server )
  #self.menu.reApply( self.server )
  
  self.server.applyChanges()
 
 
 def define(self):
  if not rospy.has_param('/GoalManage/ControlBase'):
   rospy.set_param('/GoalManage/ControlBase','move_base')
  else:
   self.ControlBase = rospy.get_param('/GoalManage/ControlBase')
   
  self.locker = Lock()
  self.ModePub = rospy.Publisher('/%s/Nav_Mode'%self.ControlBase, String, queue_size = 1)
  self.PubMode = ''
  self.frame_id = 'map'
  self.MenuHight = 0.45
  self.MenuScale = 0.1
  self.root_topic = 'menu'
  self.MenuMarkerName = 'MenuMarker'
  
  self.MenuItemID = 0
  self.ParentItemName = 'Nav Model'
  self.ChirldItemName = ['Single task Model', 'Multi tasks Model']
  self.odom = None
  
  self.server = InteractiveMarkerLib.InteractiveMarkerServer(self.root_topic)
  self.menu = InteractiveMarkerLib.MenuHandler()
  
  """self.server = InteractiveMarkerServer(self.root_topic)
  self.menu = MenuHandler()"""

 
if __name__=='__main__':
 rospy.init_node('MenuHandler')
 rospy.loginfo ("initialization system")
 ModelTrans()
 rospy.loginfo ("process done and quit")

