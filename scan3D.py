#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# ------------------------------------------

# LAAS-CNRS: Robotic and Interaction Systems
# 3D SCANNING, Platine Light Unit and SICK LD-MRS LiDAR
# Harold F. MURCIA - September 2017
"""
Created on Thu Sep 28 16:01:59 2017

@author: haroldfmurcia
"""
# ------------------------------------------

import sys, rospy, time, os

import actionlib
#actionlib_tutorials.msg
import platine_light, platine_light.msg
import LiDARldmrs, LiDARldmrs.msg

from std_msgs.msg import String
from platine_light.msg import platine_timed_pos
from platine_light.msg import platine_ptu_state
from platine_light.srv import *
from LiDARldmrs.msg import *
from LiDARldmrs.srv import *

import math

# DATA----------------------------------------------------------------------------------
class point_class(object):
    def __init__ (self,Tilt, Pan, layer,echo,flags,horizontal_angle,radial_distance,pulse_width):
        self.Tilt =Tilt
        self.Pan = Pan
        self.layer = layer
        self.echo=echo
        self.flags=flags
        self.horizontal_angle=horizontal_angle
        self.radial_distance=radial_distance
        self.pulse_width=pulse_width
        self.x =[]
        self.y =[]
        self.z =[]

class ldmrsScan_class(object):
    def __init__ (self):
        self.Tilt=[]
        self.Pan=[]
        self.scan_number =[]
        self.scanner_status=[]
        self.sync_phase_offset=[]
        self.angle_ticks_per_rotation=[]
        self.start_angle=[]
        self.end_angle=[]
        self.scan_points=[]
        self.mount_yaw=[]
        self.mount_pitch=[]
        self.mount_roll=[]
        self.mount_x=[]
        self.mount_y=[]
        self.mount_z=[]
        self.flags=[]
        self.points =[]
    def add_points(self,Tilt, Pan,  layer,echo,flags,horizontal_angle,radial_distance,pulse_width):
        self.points.append(point_class(Tilt, Pan,layer,echo,flags,horizontal_angle,radial_distance,pulse_width))

class LDMRS_CLOUD(object):
    def __init__ (self):
        self.ip=[]
        self.firmware_version=[]
        self.fpga_version=[]
        self.serial=[]
        self.port=[]
        self.scanFreq=[]
        self.start_angle=[]
        self.end_angle=[]
        self.temperature=[]
        self.ldmrsScan= []
        self.ldmrsScansN= 0
        self.description = ""
    def add_ldmrsScan(self,ldmrsScan):
        self.ldmrsScansN = self.ldmrsScansN + 1
        self.ldmrsScan.append(ldmrsScan)

# DEVICES-----------------------------------------------------------------------

class ptu_light(object):

    def __init__(self,device=None):
        self.device = device
        self.Tilt=-1;
        self.Tilt_1 = []
        self.Tilt_2 = []
        self.Pan = 0;
        self.Pan_1 = []
        self.Pan_2 = []
        self.TiltMax = []
        self.TiltMin = []
        self.TiltRes = []
        self.TiltSpeed = []
        self.TiltBaseSpeed = []
        self.TiltAcc = []
        self.TiltPos = []
        self.PanMax  = []
        self.PanMin  = []
        self.PanRes  = []
        self.PanSpeed = []
        self.PanBaseSpeed = []
        self.PanAcc = []
        self.PanPos = []
        self.sub1 = rospy.Subscriber("platine_light/ptu_position", platine_timed_pos, self.callback1)
        self.sub2 = rospy.Subscriber("platine_light/ptu_state",    platine_ptu_state, self.callback2)
        print ("PTU Light: " + "Device -" +str(device))

    def PlatineLight_open_client(self):
        print "-- PTU: Opening --"
        client = actionlib.SimpleActionClient('platine_light/open', platine_light.msg.openAction)
        client.wait_for_server()
        goal = platine_light.msg.openGoal()
        goal.device= self.device
        client.send_goal(goal)
        while(self.TiltRes==[]):
            nop=0
        return client.get_result()

    def PlatineLight_kill(self):
        time.sleep(1)
        #s = rospy.ServiceProxy('/platine_light/kill', kill)
        os.system ("bash -c 'rosservice call /platine_light/kill'")
        print "-- PTU: bye --"

    def PlatineLight_set_baudRate(self):
        print "Setting BaudRate MAX"
        client=actionlib.SimpleActionClient('/platine_light/set_baudrate', platine_light.msg.set_baudrateAction)
        client.wait_for_server()
        goal = platine_light.msg.set_baudrateGoal()
        goal.device= self.device
	goal.current_baudrate.value = 0
        goal.new_baudrate.value = 1

    def Go2Position(self, axis, position, unit):
        client = actionlib.SimpleActionClient('/platine_light/goto_position', platine_light.msg.goto_positionAction)
        client.wait_for_server()
        goal = platine_light.msg.goto_positionGoal()
        goal.axis.value=axis
        goal.value=position
        goal.unit.value=unit
        client.send_goal(goal)
        client.wait_for_result()
        #print "-- Going to Position --"
        return client.get_result()

    def callback1(self, data):
        #self.sub1 = rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", data.value, data.axis)
        axis = (str(data.axis))
        if "0" in axis:
            # Updating Pan[k], Pan[k-1] and Pan[k-2]
            self.Pan_2 = self.Pan_1
            self.Pan_1 = self.Pan
            Pan = data.value
            self.Pan = Pan
        elif "1" in axis:
            # Updating Tilt[k], Tilt[k-1] and Tilt[k-2]
            self.Tilt_2 = self.Tilt_1
            self.Tilt_1 = self.Tilt
            Tilt = data.value
            self.Tilt=Tilt

    def callback2(self, data):
        #self.sub2 = rospy.loginfo(rospy.get_caller_id() + "I heard Max Min Res are: %s %s %s", data.tilt_state.max_position, data.tilt_state.min_position, data.tilt_state.resolution )
        self.TiltMax = data.tilt_state.max_position
        self.TiltMin = data.tilt_state.min_position
        self.TiltRes = data.tilt_state.resolution
        self.TiltSpeed = data.tilt_state.speed
        self.TiltBaseSpeed = data.tilt_state.base_speed
        self.TiltAcc = data.tilt_state.acceleration
        self.TiltPos = data.tilt_state.position
        self.PanMax  = data.pan_state.max_position
        self.PanMin  = data.pan_state.min_position
        self.PanRes  = data.pan_state.resolution
        self.PanSpeed = data.pan_state.speed
        self.PanBaseSpeed = data.pan_state.base_speed
        self.PanAcc = data.pan_state.acceleration
        self.PanPos = data.pan_state.position

    def PlatineLight_getPos(self, axis,unit):
        #Axis: [0 1]=[Pan Tilt]
        #unit: [0 1 2]=[deg rad steps]
        #print "Get Unit position"
        client=actionlib.SimpleActionClient('platine_light/monitor_position', platine_light.msg.monitor_positionAction)
        client.wait_for_server()
        goal = platine_light.msg.monitor_positionGoal()
        goal.axis.value=axis
        goal.unit.value=unit
        client.send_goal(goal)


    def PTU_profile(self,mode):
        client=actionlib.SimpleActionClient('platine_light/set_speed_profile', platine_light.msg.set_speed_profileAction)
        client.wait_for_server()
        goal = platine_light.msg.set_speed_profileGoal()
        # Tilt
	if (mode == "Fast"):
		print "Fast PTU profile"
		TA = 2000
		TS = 1500
		TB = 1000
	else:
		print "Slow PTU profile"
		TS = 200
		TA = 2000
		TB = 200
        goal.axis.value = 1
        goal.base_speed = TB
        goal.acceleration = TA
        goal.speed = TS
        goal.unit.value = 2
        client.send_goal(goal)

    def PlatineLight_close(self):
        print "-- PTU: Closing --"
	os.system("rosservice call /platine_light/stop_monitor_position")
        #self.sub1.unregister()
        #self.sub2.unregister()
        #rospy.wait_for_service('/platine_light/close')
        os.system ("bash -c 'rosservice call /platine_light/close'")
        #rospy.ServiceProxy('/platine_light/close', close)
#------------------------------------------------------------------------------

class sickLDMRS(object):
    def __init__(self,ip = None, port = None, scanFindx = None):
        #scanFindx [0 1 2] = [12.5 25 50] Hz
        self.ip = ip
        self.port = port
        self.scanFindx = scanFindx
        self.lastScan = []
        self.sub = rospy.Subscriber("LiDARldmrs/sickScanner", LiDARldmrs_sickldmrsScan, self.callbackData)
        print ("SICK LD-MRS -Py")

    def LDMRS_open_client(self):
        client = actionlib.SimpleActionClient('/LiDARldmrs/open', LiDARldmrs.msg.openAction)
        client.wait_for_server()
        goal = LiDARldmrs.msg.openGoal()
        goal.ip = self.ip
        goal.port = self.port
        goal.scan_freq.value = self.scanFindx
        client.send_goal(goal)
        time.sleep(0.25)
        return client.get_result()

    def callbackData(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %d %d", data.scan_points, data.scan_number )
        lastscane = ldmrsScan_class()
        lastscane.scan_points = data.scan_points
        lastscane.sync_phase_offset = data.sync_phase_offset
        lastscane.angle_ticks_per_rotation = data.angle_ticks_per_rotation
        lastscane.start_angle = data.start_angle
        lastscane.end_angle = data.end_angle
        lastscane.mount_yaw = data.mount_yaw
        lastscane.mount_pitch = data.mount_pitch
        lastscane.mount_roll = data.mount_roll
        lastscane.mount_x = data.mount_x
        lastscane.mount_y = data.mount_y
        lastscane.mount_z = data.mount_z
        lastscane.flags = data.flags
        N=lastscane.scan_points
	df= self.lastScan
        for n in range(0,N):
            layer= data.points[n].layer
            echo=data.points[n].echo
            flags = data.points[n].flags
            horizontal_angle = data.points[n].horizontal_angle
            radial_distance =  data.points[n].radial_distance
            pulse_width = data.points[n].pulse_width
            lastscane.add_points( self.Tilt, self.Pan, layer,echo,flags,horizontal_angle,radial_distance,pulse_width)
        self.lastScan = lastscane
	del lastscane


    def LDMRS_getData(self, Tilt, Pan):
        self.Tilt= Tilt
        self.Pan = Pan
        client = actionlib.SimpleActionClient('/LiDARldmrs/startAcquisition', LiDARldmrs.msg.startAcquisitionAction)
        client.wait_for_server()
        goal = LiDARldmrs.msg.startAcquisitionGoal()
        client.send_goal(goal)
        r=client.get_result()
        lastscane=self.lastScan
        sync = 0
        while(lastscane==[] or sync==0):
            lastscane=self.lastScan
            if lastscane != []:
                n =lastscane.scan_points
                if(lastscane.points[n-1].Tilt == Tilt):
                	sync = 1
        del sync
        #n=lastscane.scan_points
        #print "\n\t layerGet on GetData: "+str(n) #+str(lastscane.points[n].layer)
        return lastscane

    def LDMRS_close_client(self):
        print "-- LDMRS: Closing --"
        self.sub.unregister()
        #rospy.wait_for_service('/platine_light/close')
        os.system ("rosaction call /LiDARldmrs/quit {}")
        os.system ("rosservice call /LiDARldmrs/close")
        #s = rospy.ServiceProxy('/LiDARldmrs/close', close)

    def LDMRS_kill(self):
        #s = rospy.ServiceProxy('/LiDARldmrs/kill', kill)
        os.system ("rosservice call /LiDARldmrs/kill")
        time.sleep(1.0)
        os.system("rosnode kill "+"/LiDARldmrs")
        print "-- LDMRS: bye --"
