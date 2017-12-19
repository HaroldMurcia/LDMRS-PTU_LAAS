#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# LAAS-CNRS: Robotic and Interaction Systems
# 3D SCANNING, Platine Light Unit and SICK LD-MRS LiDAR
# Harold F. MURCIA - November 2017

"""
Created on Wed Nov 29 14:09:21 2017

This file enables the interaction with minnie position ROS nodes

@author: haroldfmurcia
"""

import sys, os, rospy
from rmp440.msg import *

class ROVER(object):
    def __init__ (self):
        self.ip = '10.40.40.40'
        self.port = '8080'
        self.x = []
        self.y = []
        self.z = []
        self.qw = []
        self.qx = []
        self.qy = []
        self.qz = []
	self.ts = []
        self.sub1 = rospy.Subscriber("/rmp440/Pose", or_pose_estimator_state, self.callbackPose)

    def callbackPose(self, data):
	P=  str(data.pos)
	P= P.split("\n")
	Px= float(P[0].split(':')[1])
	Py= float(P[1].split(':')[1])
	Pz= float(P[2].split(':')[1])
	Qw= float(P[3].split(':')[1])
	Qx= float(P[4].split(':')[1])
	Qy= float(P[5].split(':')[1])
	Qz= P[6].split(':')[1]
	Qz= float(Qz[:-1])
        self.x = Px
        self.y = Py
        self.z = Pz
        self.qw =Qw
        self.qx =Qx
        self.qy =Qy
        self.qz =Qz
	self.ts = data.ts

    def ini_Rover(self):
        os.system(" rosaction call /rmp440/Init '10.40.40.40:8080' &")
        os.system(" rosservice call /rmp440/connect_port '{local: 'Joystick', remote: 'joystick/device/Logitech_Gamepad_F710'}' &")
        os.system(" rosaction call /rmp440/Gyro '{params: {port: '/dev/ttyS4', mode: {value: 1}, type: {value: 3}, latitude: 43, woffset: 0}}' &")
        os.system(" rosaction call /rmp440/JoystickOn {} &" )

    def stop_Rover(self):
        os.system(" rosservice call  /rmp440/kill &")
        os.system(" rosservice call  /joystick/kill &")
        os.system(" sudo systemctl start  joyd.service &")

    def stopReading(self):
        self.sub1.unregister()
