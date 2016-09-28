#!/usr/bin/env python
#coding=utf-8
"""
用来转变当前的导航模式
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MenuEntry
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import Marker
import InteractiveMarkerLib  
from std_msgs.msg import String


class ModelTrans():
 def __init__(self):
  self.define()
  self.InitMenu()
  rospy.Subscriber('/turtlebot_position_in_map', Pose, self.OdomCB)
  self.server.applyChanges()
  rospy.spin()
  
  
 def OdomCB(self, data)
  with self.locker:
   self.odom = data
   self.MakeMuneObject(self.MenuMarkerName , self.odom)
   self.MenuApplyChanges(self.MenuMarkerName, self.server)
     
  
 def MakeMuneObject(self, MenuName, MenuPose):
  MenuInteractiveMarker = InteractiveMarker()
  MenuInteractiveMarker.name = MenuName
  MenuInteractiveMarker.header.frame_id = self.frame_id
  MenuInteractiveMarker.pose = MenuPose
  MenuInteractiveMarker.pose.position.z += self.MenuHight
  MenuInteractiveMarker.pose.position.y += -3.0 * self.MenuPose
  self.MenuPose += 1
  MenuInteractiveMarker.scale = self.MenuScale
  
  MenuControl = InteractiveMarkerControl()
  MenuControl.interaction_mode = InteractiveMarkerControl.MENU
  MenuControl.always_visible = False
  
  MenuMarker = Marker()
  
  MenuMarker.type = Marker.ARROW
  MenuMarker.scale.x = msg.scale * 2
  MenuMarker.scale.y = msg.scale * 0.45
  MenuMarker.scale.z = msg.scale * 0.45
  MenuMarker.color.r = 1.0
  MenuMarker.color.g = 1.0
  MenuMarker.color.b = 1.0
  MenuMarker.color.a = 1.0
  
  MenuControl.markers.append(MenuMarker)
  
  MenuInteractiveMarker.controls.append(MenuControl)
  
  self.server.insert(MenuInteractiveMarker)
  
 
 def MenuApplyChanges(self, MenuName, server):
  self.menu.ApplyMenuChanges( server, MenuName )
  
  
 def InitMenu(self):
  ParentItemID = self.menu.InsertMenu(ChirldTitle = self.ParentItemName)
  self.MenuItemID = ParentItemID
  for ChirldItem in self.ChirldItemName:
   ChirldItemTitle = 'Mode ' + ChirldItem
   ChirldItemID = self.menu.InsertMenu(ChirldTitle = ChirldItemTitle, ParentTitleID = ParentItemID, Callback = modeCb)
   self.menu.SetEntryCheckState( ChirldItemID, MenuHandler.UNCHECKED)
      
  #default to set single task module
  #self.menu.SetEntryCheckState( ChirldItemS, MenuHandler.CHECKED )
 
 
 def modeCb(self, feedback):
  self.menu.SetEntryCheckState(self.MenuItemID, MenuHandler.UNCHECKED)
  Itemid = feedback.menu_entry_id
  self.MenuItemID = Itemid
  self.menu.SetEntryCheckState(Itemid, MenuHandler.CHECKED)
  rospy.loginfo("Switching to menu entry : " + self.ChirldItemName[self.MenuItemID - 2])
  self.menu.ReApplyMenuChanges( self.server )
  self.server.applyChanges()
 
 def define(self):
  self.ControlBase = rospy.get_param('/GoalManage/ControlBase')
  self.ModePub = rospy.Publisher('/%s/Nav_Mode'%self.ControlBase, String, queue_size = 1)
  self.PubMode = ''
  self.frame_id = '\map'
  self.MenuHight = 0.6
  self.MenuScale = 1
  self.MenuPose = 0
  self.root_topic = 'menu'
  self.MenuMarkerName = 'MenuMarker'
  
  self.MenuItemID = 0
  self.ParentItemName = 'Nav Model'
  self.ChirldItemName = ['Single task Model', 'Multi tasks Model']

  self.server = InteractiveMarkerLib.InteractiveMarkerServer(self.root_topic)
  self.menu = InteractiveMarkerLib.MenuHandler()
 
 
if __name__=='__main__':
 rospy.init_node('MenuHandler')
 rospy.loginfo ("initialization system")
 ModelTrans()
 rospy.loginfo ("process done and quit")

