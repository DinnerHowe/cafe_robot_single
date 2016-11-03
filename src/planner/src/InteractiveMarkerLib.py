#!/usr/bin/env python
#coding=utf-8
"""
InteractiveMaker 的库文件，包含server 和 menu handle (wait for testing code)

Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
import copy
from std_msgs.msg import Header
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerInit
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate
from threading import Lock

from visualization_msgs.msg import MenuEntry

from geometry_msgs.msg import Pose

###############################
##### Interactive Server ######
###############################

"""
使用方法：
 server = Server(root_topic)
 server.insert(obstacle)
 server.applyChanges()
"""
DEFAULT_FEEDBACK_CB = 255

class InteractiveMarkerServer:
 def __init__(self, root_topic):
  self.define(root_topic)
  rospy.Subscriber(self.root_topic+"/feedback", InteractiveMarkerFeedback, self.processFeedback, queue_size = self.q_size) 
  rospy.Timer(rospy.Duration(0.5), self.StandBy)
  rospy.Timer(rospy.Duration(0.5), self.ClearAll)
  self.PublishInit()
    
    
 def define(self, root_topic):
  self.locker = Lock()
  self.q_size = 10
  self.seq_num = 0
  self.root_topic = '/'+root_topic
  self.server_id = root_topic
  self.obstacle = None
  self.Current = InteractiveMarkerUpdate()
  self.Candidate = InteractiveMarkerFeedback()
  self.CurrentP = Pose()
  
  self.Marker_Context = dict()
  
  self.update_ = rospy.Publisher(self.root_topic+"/update", InteractiveMarkerUpdate ,queue_size = self.q_size)
  self.init_ = rospy.Publisher(self.root_topic+'/update_full', InteractiveMarkerInit, queue_size = self.q_size)
    
  
 def processFeedback(self, feedback):
  with self.locker: 
   self.Candidate = feedback
   #rospy.loginfo('\nfeedback: ' + '%s'%self.Candidate)
   if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
    self.MovingServer(self.Candidate) #1
   if feedback.event_type == InteractiveMarkerFeedback.KEEP_ALIVE:
    self.KeepAlive() #0
   if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
    self.MenuServer() #2
   if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
    self.ClickServer() #3
   if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
    self.ClickPushServer() #4
   if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
    self.ClickReleaseServer() #5


 def MovingServer(self, Candidate):
  self.Current.type = InteractiveMarkerUpdate.UPDATE
  #print 'self.Current.type', self.Current.type
  #print 'MovingServer: self.obstacle: ', self.obstacle
  self.Current.seq_num = self.seq_num
  self.Current.markers.append(copy.deepcopy(self.obstacle))
   
  position = InteractiveMarkerPose()
  position.header.seq = self.seq_num
  position.header.stamp = rospy.Time.now()
  position.header.frame_id = 'map'
  position.name = self.obstacle.name
  position.pose = Candidate.pose

  self.Current.poses.append(copy.deepcopy(position))

  self.seq_num += 1   
  self.UpdateCB(self.Current)
  self.PublishInit()
  
  
 def KeepAlive(self):
 #waiting for fill up function
  pass
  
  
 def MenuServer(self):
 #waiting for fill up function
  pass
  
  
 def ClickServer(self):
 #waiting for fill up function
  pass
  
  
 def ClickPushServer(self):
 #waiting for fill up function
  pass
  
  
 def ClickReleaseServer(self):
 #waiting for fill up function
  pass


 def UpdateCB(self,data):
  data.server_id = self.server_id
  data.seq_num = self.seq_num
  self.update_.publish(data)
  
  
 def ClearAll(self, event):
  #with self.locker:
   self.update_ = rospy.Publisher(self.root_topic+"/update", InteractiveMarkerUpdate ,queue_size=1)
   self.init_ = rospy.Publisher(self.root_topic+'/update_full', InteractiveMarkerInit, queue_size=1)
  
   self.Current = InteractiveMarkerUpdate()
   self.Candidate = InteractiveMarkerFeedback()
   self.CurrentP = Pose()
   self.PublishInit()


 def StandBy(self, event):
  #with self.locker:
   stand_BY=InteractiveMarkerUpdate()
   stand_BY.type = InteractiveMarkerUpdate.KEEP_ALIVE
   self.UpdateCB(stand_BY)

  
 def insert(self, obstacle, feedback_cb = None, feedback_type = DEFAULT_FEEDBACK_CB):
  with self.locker:
   self.obstacle = copy.deepcopy(obstacle)
   self.Current.type = InteractiveMarkerFeedback.KEEP_ALIVE
   self.Current.server_id = copy.deepcopy(self.server_id)
   self.Current.seq_num = copy.deepcopy(self.seq_num)
   self.Current.markers.append(copy.deepcopy(self.obstacle))
   self.Marker_Context[copy.deepcopy(self.obstacle.name)] = copy.deepcopy(self.obstacle)
   print 'insert self.Marker_Context: %s\n'%len(self.Marker_Context)#, self.Marker_Context
   
 def applyChanges(self):
  with self.locker:
   #print 'applyChanges 1'
   if self.obstacle == None:
    return
   else:
    #print 'applyChanges 2'
    self.Current.type = InteractiveMarkerUpdate.UPDATE
    self.Current.seq_num = self.seq_num
    self.Current.markers.append(copy.deepcopy(self.obstacle))
    #print 'applyChanges 3'
    self.seq_num += 1   
    self.UpdateCB(self.Current)
    #print 'applyChanges 4'
    self.PublishInit()
    #print 'applyChanges 5'
  
 def PublishInit(self):
  #with self.locker:
   #print 'PublishInit 1'
   initData = InteractiveMarkerInit()
   initData.server_id = self.server_id
   initData.seq_num = self.seq_num
   #print 'PublishInit 2'
   if self.obstacle == None:
    #if there is no marker existing
    initData.markers=[]
    #print 'PublishInit 3'
    self.init_.publish(initData)
    #print 'PublishInit 4'
   else:
    initData.markers.append(self.obstacle)
    #print 'PublishInit 5'
    #if there is a marker existing
    self.init_.publish(initData)
    #print 'PublishInit 6'
    
    
 # @brief Get marker by name   
 def GetMarker(self, MarkerName):
  print 'GetMarker: self.Marker_Context: \n', len(self.Marker_Context)
  if MarkerName in self.Marker_Context.keys():
   return self.Marker_Context[MarkerName]
  else:
   rospy.loginfo('GetMarker: there is no marker named: '+ MarkerName)
   return None


###############################
##### MenuHandler Server ######
###############################   
 
class MenuHandler:
 NO_CHECKBOX = 0
 CHECKED = 1
 UNCHECKED = 2

 def __init__(self):
  self.define()



 def define(self):
  #self.locker = Lock()
  self.Menu_Context = {'Title': '', 'Command': '', 'CommandType': 0, 'ChirldItem': list(), 'Visible': True, 'CheckState': 0, 'FeedBack': None} 
  
  self.MenuItemID = 0

  self.NO_CHECKBOX = MenuHandler.NO_CHECKBOX
  self.CHECKED = MenuHandler.CHECKED
  self.UNCHECKED = MenuHandler.UNCHECKED
  
  self.MenuTree = dict()
  self.MenuItemIDs = list()
  self.MenuMarkerNames = list()
  
  
  
 # insert a Mune info
 def InsertMenu(self, ChirldTitle, ParentTitleID = None, CommandType = MenuEntry.FEEDBACK, Command = '', Callback=None):
  CurrentItemID = self.Formalization(ChirldTitle, CommandType, Command, Callback)
  #print 'lib CurrentItemID:', CurrentItemID
  if ParentTitleID is not None:
   # check if input parent is exist, if did exist, then store current CurrentItemID into Parent_Context['ChirldItem'], so that a menu tree is built
   if ParentTitleID in self.MenuTree.keys():
    #print 'self.MenuTree.keys()\n', self.MenuTree.keys()
    Parent_Context = self.MenuTree[ParentTitleID]
    Parent_Context['ChirldItem'].append(CurrentItemID)
   else:
    rospy.logerr("Parent menu ID" + str(ParentTitleID) + " not found.")
    return None
  else:
   pass
  # store all menu items' id in to a list for recording
  self.MenuItemIDs.append(CurrentItemID)

  #print '\n\n\n##############\n', 'lib self.MenuTree.keys: ', self.MenuTree.keys(), '\n##############\n\n\n'
  print '\n\n\n##############\n', 'lib self.MenuTree: ', self.MenuTree, '\n##############\n\n\n'
  
  return CurrentItemID



 # store menue into a form make data orgnised
 def Formalization(self, ChirldTitle, CommandType, Command, Callback):
  CurrentItemID = copy.deepcopy(self.MenuItemID)
  self.MenuItemID += 1
  
  #formalise Menu Item info. by dict
  MenuContext = copy.deepcopy(self.Menu_Context)
  MenuContext['Title'] = ChirldTitle
  MenuContext['CommandType'] = CommandType
  MenuContext['Command'] = Command
  MenuContext['Visible'] = True
  MenuContext['CheckState'] = copy.deepcopy(self.NO_CHECKBOX)
  MenuContext['FeedBack'] = Callback
  
  # store one Menu Item info. into a dict named MenuTree
  self.MenuTree[CurrentItemID] = MenuContext
  
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
   #rospy.logerr("Item menu ID" + str(ItemID) + " not found.")
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
 def ApplyMenuChanges(self, Server, MarkerName):
  marker = Server.GetMarker(MarkerName)#return InteractiveMarker
  self.MenuMarkerNames.append(MarkerName)
  #if has no marker
  print 'ApplyMenuChanges if has no marker: ', not marker , '\nmarker named: ', MarkerName #, '\nmarker: ', marker,
  if not marker:
   self.MenuMarkerNames.remove(MarkerName)
   rospy.loginfo('there is no marker named ' + MarkerName + ' apply error change to menu object')
   return False
  marker.menu_entries = []
  parent_ID = 0
  #print 'self.MenuItemIDs: ', self.MenuItemIDs
  self.AttachMenuToMarker(self.MenuItemIDs, marker.menu_entries, parent_ID)
  FeedbackType = InteractiveMarkerFeedback.MENU_SELECT
  Server.insert(marker, feedback_cb = self.MenueFeedback, feedback_type = FeedbackType)
  return True
  
  
  
 def AttachMenuToMarker(self, MenuItemIDs, MarkerMenuEntries, parent_ID):
  #print 'MarkerMenuEntries:\n', MarkerMenuEntries, '\nMenuItemIDs:\n', MenuItemIDs, '\n########################'
  for ID in MenuItemIDs:
   if ID in self.MenuTree.keys():
    Menu_Context = self.MenuTree[ID]
    if Menu_Context['Visible']:
     MarkerMenuEntries.append(self.MakeMenuEntry(Menu_Context, ID, parent_ID))
    else:
     rospy.loginfo("AttachMenuToMarker info: Menu Entry" + Menu_Context['Title'] + " not in MenuTree! ")
     pass
    #print Menu_Context['ChirldItem'], MarkerMenuEntries, ID
    if not self.AttachMenuToMarker(Menu_Context['ChirldItem'], MarkerMenuEntries, ID):
     return False
   else:
    rospy.logerr("AttachMenuToMarker error: MenuItemIDs:%s not in MenuTree! "%ID)
    return False
    
   ropsy.loginfo('MarkerMenuEntries: \n'+ str(MarkerMenuEntries))
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
   
   
