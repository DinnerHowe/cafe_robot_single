#!/usr/bin/env python
#coding=utf-8
"""
fake robot total program

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""

import speech,time,rospy  
  
response = speech.input("Say something, please.")  
speech.say("You said " + response)  
  
def callback(phrase, listener):
    if phrase == "goodbye":  
        listener.stoplistening()  
    speech.say(phrase)  
    print phrase  
  
listener = speech.listenforanything(callback)  
while listener.islistening():  
    time.sleep(.5) 
