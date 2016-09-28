#!/usr/bin/env python
#coding=utf-8
"""
InteractiveMaker 的库文件，包含server 和 menu handle (wait for testing code)

Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerInit
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate
from threading import Lock

from visualization_msgs.msg import MenuEntry

###############################
##### Interactive Server ######
###############################

"""
使用方法：
 server = Server(root_topic)
 server.insert(obstacle)
 server.applyChanges()
"""


class InteractiveMarkerServer:
 def __init__(self, root_topic):
  self.define()
  rospy.Subscriber(topic_ns+"/feedback", InteractiveMarkerFeedback, self.processFeedback, queue_size=q_size)
  rospy.Timer(rospy.Duration(0.1), self.StandBy)
  rospy.Timer(rospy.Duration(10), self.ClearAll)
  self.publishInit()
    
    
 def define(self):
  self.locker = Lock()
  self.seq_num = 0
  self.root_topic = '/'+root_topic
  self.server_id = root_topic
  self.obstacle=None
  self.Current = InteractiveMarkerUpdate()
  self.Candidate = InteractiveMarkerFeedback()
  self.CurrentP = Pose()
  
  self.DEFAULT_FEEDBACK_CB = 255
  self.Marker_Context = dict()
  
  self.update_ = rospy.Publisher(self.root_topic+"/update", InteractiveMarkerUpdate ,queue_size=1)
  self.init_ = rospy.Publisher(self.root_topic+'/update_full', InteractiveMarkerInit, queue_size=1)
    
  
 def processFeedback(self, feedback)
  with self.locker: 
   self.Candidate = feedback
   rospy.loginfo('\nfeedback: ' + '%s'%self.Candidate)
   self.MovingServer()
  
  
 def MovingServer(self):
  self.Current.type = InteractiveMarkerUpdate.UPDATE
  self.Current.seq_num = self.seq_num
  self.Current.markers.append(copy.deepcopy(self.obstacle))
   
  position = InteractiveMarkerPose()
  position.header.seq = self.seq_num
  position.header.stamp = rospy.Time.now()
  position.header.frame_id = 'map'
  position.name = self.obstacle.name
  position.pose = self.Candidate.pose

  self.Current.poses.append(copy.deepcopy(position))

  self.seq_num += 1   
  self.UpdateCB(self.Current)
  self.PublishInit()
  
  
 def UpdateCB(self,data):
  data.server_id = self.server_id
  data.seq_num = self.seq_num
  self.update_.publish(data)
  
  
 def ClearAll(self, event):
  self.update_ = rospy.Publisher(self.root_topic+"/update", InteractiveMarkerUpdate ,queue_size=1)
  self.init_ = rospy.Publisher(self.root_topic+'/update_full', InteractiveMarkerInit, queue_size=1)
  
  self.Current = InteractiveMarkerUpdate()
  self.Candidate = InteractiveMarkerFeedback()
  self.CurrentP = Pose()
  self.PublishInit()


 def StandBy(self, event):
  with self.locker:
   stand_BY=InteractiveMarkerUpdate()
   stand_BY.type = InteractiveMarkerUpdate.KEEP_ALIVE
   self.UpdateCB(standby)

  
 def insert(self, obstacle, feedback_cb = None, feedback_type = self.DEFAULT_FEEDBACK_CB):
  with self.locker:
   self.obstacle = obstacle
   self.Current.type = InteractiveMarkerFeedback.KEEP_ALIVE
   self.Current.server_id = self.server_id
   self.Current.seq_num = self.seq_num
   self.Current.markers.append(self.obstacle)
   self.Marker_Context[self.obstacle.name] = self.obstacle
   
 def applyChanges(self):
  with self.locker:
   if self.obstacle == None:
    return
   else:
    self.Current.type = InteractiveMarkerUpdate.UPDATE
    self.Current.seq_num = self.seq_num
    self.Current.markers.append(copy.deepcopy(self.obstacle))

    self.seq_num += 1   
    self.UpdateCB(self.Current)
    self.PublishInit()
  
  
 def publishInit(self):
  with self.locker:
   initData = InteractiveMarkerInit()
   initData.server_id = self.server_id
   initData.seq_num = self.seq_num
   if self.obstacle == None:
    #if there is no marker existing
    initData.markers=[]
    self.init_.publish(initData)
   else:
    initData.markers.append(self.obstacle)
    #if there is a marker existing
    self.init_.publish(initData)
    
    
 # @brief Get marker by name   
 def GetMarker(self, MarkerName):
  if MarkerName in self.Marker_Context.keys():
   return self.Marker_Context[MarkerName]
  else:
   rospy.loginfo('there is no marker named'+ MarkerName)
   return None


###############################
##### MenuHandler Server ######
###############################   
 
class MenuHandler:
 def __init__(self, root_topic):
  self.define()



 def define(self):
  self.Menu_Context = {'Title': '', 'Command': '', 'CommandType': 0, 'ChirldItem': list(), 'Visible': True, 'CheckState': 0, 'FeedBack': None} 
  
  self.MenuItemID = 1

  self.NO_CHECKBOX = 0
  self.CHECKED = 1
  self.UNCHECKED = 2
  
  self.MenuTree = dict()
  self.MenuItemIDs = list()
  self.MenuMarkerNames = list()
  
  
  
 # insert a Mune info
 def InsertMenu(self, ChirldTitle, ParentTitleID = None, CommandType = MenuEntry.FEEDBACK, Command = '', Callback=None):
  CurrentItemID = self.Formalization(ChirldTitle, CommandType, Command, Callback)
  if ParentTitleID is not None:
   # check if input parent is exist, if did exist, then store current CurrentItemID into Parent_Context['ChirldItem'], so that a menu tree is built
   if ParentTitleID in self.MenuTree.keys():
    Parent_Context = self.MenuTree[ParentTitleID]
    Parent_Context['ChirldItem'].append(CurrentItemID)
   else:
    rospy.logerr("Parent menu ID" + str(ParentTitleID) + " not found.")
    return None
  else:
   # store all menu items' id in to a list for recording
   self.MenuItemIDs.append(CurrentItemID)
  return CurrentItemID



 # store menue into a form make data orgnised
 def Formalization(self, ChirldTitle, CommandType, Command, Callback):
  CurrentItemID = self.MenuItemID
  self.MenuItemID += 1
  
  #formalise Menu Item info. by dict
  MenuContext = copy.deepcopy(self.Menu_Context)
  MenuContext['Title'] = ChirldTitle
  MenuContext['CommandType'] = CommandType
  MenuContext['Command'] = Command
  MenuContext['Visible'] = True
  MenuContext['CheckState'] = self.NO_CHECKBOX
  MenuContext['FeedBack'] = Callback
  
  # store one Menu Item info. into a dict named MenuTree
  self.MenuTree[self.MenuItemID] = MenuContext
  
  return CurrentItemID
 
 
 
 #setting the entry do visible or not
 def SetEntryVisible(self, ItemID, Visible):
  if ItemID in self.MenuTree.keys():
   Item_Context = self.MenuTree[ItemID]
   Item_Context['Visible'] = Visible
   return True
  else:
   rospy.logerr("Item menu ID" + str(ItemID) + " not found.")
   return False
 
  
  
 #setting if an entry is checked or can't be checked at all
 def SetEntryCheckState(self, ItemID, CheckState):
  if ItemID in self.MenuTree.keys():
   Item_Context = self.MenuTree[ItemID]
   Item_Context['CheckState'] = CheckState
   return True
  else:
   rospy.logerr("Item menu ID" + str(ItemID) + " not found.")
   return False
  
 
 
 #return an entry's Check State
 def GetEntryCheckState(self, ItemID, CheckState):
  if ItemID in self.MenuTree.keys():
   Item_Context = self.MenuTree[ItemID]
   return Item_Context['CheckState']
  else:
   rospy.loginfo("Item menu ID" + str(ItemID) + " not found.")
   return None
   
   
  
 #divert callback for MENU_SELECT feedback to this manager 
 def ApplyMenuChanges(self, Server, MarkerName)
  marker = Server.GetMarker(MarkerName)#return InteractiveMarker
  self.MenuMarkerNames.append(MarkerName)
  #if has no marker
  if not marker:
   self.MenuMarkerNames.remove(MarkerName)
   rospy.loginfo('there is no marker named' + MarkerName + 'apply error change to menu object')
   return False
  marker.menu_entries = []
  parent_ID = 0
  self.AttachMenuToMarker(self.MenuItemIDs, marker.menu_entries, parent_ID)
  FeedbackType = InteractiveMarkerFeedback.MENU_SELECT
  Server.insert(marker, feedback_cb = self.MenueFeedback, feedback_type = FeedbackType)
  return True
  
  
  
 def AttachMenuToMarker(self, MenuItemIDs, MarkerMenuEntries, parent_ID):
  for ID in MenueItemIDs:
   if ID in self.MenuTree.keys():
    Menu_Context = self.MenuTree[ID]
    if Menu_Context['Visible']:
     MarkerMenuEntries.append(self.MakeMenuEntry(Menu_Context, ID, parent_ID)
    if not self.AttachMenuToMarker(Menu_Context['ChirldItem'], MarkerMenuEntries, ID):
     return False
   else:
    rospy.logerr("AttachMenuToMarker error: MenuItemIDs not in MenuTree! ")
    return False
   return True
   
   
   
 def MakeMenuEntry(self, Menu_Context, ID, parent_ID):
  Menu_Entry = MenuEntry()
  if Menu_Context['CheckState'] == self.NO_CHECKBOX:
   Menu_Entry.title = Menu_Context['Title']
  elif Menu_Context['CheckState'] == self.CHECKED:
   Menu_Entry.title = '[x] ' + Menu_Context['Title']
  elif Menu_Context['CheckState'] == self.UNCHECKED:
   Menu_Entry.title = '[ ] ' + Menu_Context['Title']
  else:
   rospy.loginfo('unexpect CheckState input')
  Menu_Entry.command = Menu_Context['Command']
  Menu_Entry.command_type = Menu_Context['CommandType']
  Menu_Entry.id = ID
  Menu_Entry.parent_id = parent_ID
  return Menu_Entry
 
 
 
 def MenueFeedback(self, feedback):
  if feedback.menu_entry_id in self.MenuTree.keys():
   Menu_Context = self.MenuTree[feedback.menu_entry_id]
  else:
   rospy.logerr("MenueFeedback error: feedback.menu_entry_id not in MenuTree! ")
   return
  Menu_Context['FeedBack'] = feedback
   
 # Re-apply to all markers that this was applied to previously
 def ReApplyMenuChanges(self, server):
  success = True
  for markername in self.MenuMarkerNames:
   success = self.ApplyMenuChanges(Server, MarkerName) and success
  return success
   
   
