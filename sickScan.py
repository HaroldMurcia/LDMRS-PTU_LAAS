#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# ------------------------------------------

# LAAS-CNRS: Robotic and Interaction Systems
# SICK LDMRS, Platine Light
# Harold F. MURCIA - September 2017
"""
Created on Thu Sep 28 16:01:59 2017
This is the main file for acquisition tasks. The main loop creates the rospy node scan3D_client_py
and needs in execution the nodes_ platine_light-ros and LiDARldmrs-ros and/OR rmp440-ros and/OR joystick-ros.
There is a function of help -h, and a function main_check which try to to guide the use the adcquisition tool.
For a simple uses, run: >> python sickScan.py min_angle max_angle interes_angle sigma
    where:
    min_angle is the minimum tilt angle in degrees
    max_angle is the maximum tilt angle in degrees
    interes_angle is the angle where the system will scann more points
    sigma is an indicator of resolution between [0 and 1]. USE 0.1 by default
    * At the end of its execution, the program must creates a .txt file with the initial name parameters and the
    adcquired information on the folder: data
For help run: >> python sickScan.py -h
** Speed of the PTU can be changed on code line 206 to 213 of file scan3D.py
** The flag: moving_mode, refer to ROS nodes rmp440-ros and joystick-ros and implies save data with/out rover possitions
** The dev name for PTU, ip addres and port of SICK LiDAR must be configured on lines 35 to 37.
@author: haroldfmurcia
"""
# ------------------------------------------
import  rospy, os, sys, time, math, getpass, pickle, psutil
import  scan3D, rover
import numpy as np
from subprocess import check_output


path_data = os.getcwd() + "/data/raw"
path_bin  = "/home/"+getpass.getuser()+"/ROS_GenoM3/bin"
dev = '/dev/tty_PTU'
ip_address = '192.168.0.2'
ip_port='12002'
pid = os.getpid()
py = psutil.Process(pid)

def initScan(moving_mode):
    if moving_mode == 1:
    	minnie = rover.ROVER()
	minnie.ini_Rover()
    else:
	minnie = 0
    PTU   = scan3D.ptu_light(device= dev)
    LiDAR = scan3D.sickLDMRS(ip= ip_address, port= ip_port, scanFindx=1)
    pointCloud = scan3D.LDMRS_CLOUD()
    pointCloud.description = raw_input("Short description of Data: ")
    pointCloud.ip = LiDAR.ip
    pointCloud.port = LiDAR.port
    pointCloud.scanFreq = LiDAR.scanFindx
    save_mode = 't'
    return PTU, LiDAR, pointCloud, minnie, save_mode

def TiltScan(PTU, LiDAR, cloud, rover, TiltMin, TiltMax, angle_interest ,sigma, save_mode, moving_mode):
    #Phisical TiltMax= maxAng #2354  PTU46-> approx. +32 deg
    #Phisical TiltMin= minAng #-3569 PTU46-> approx. -45 deg
    #Phisical Tilt stepRes           PTU46-> approx. 0.0127 deg
    radsPerStep = PTU.TiltRes
    degPerStep = radsPerStep*180.0/math.pi
    MAX_T = PTU.TiltMax
    MIN_T = PTU.TiltMin
    u = angle_interest
    if (TiltMax > MAX_T*degPerStep):
        print ("\n ERROR: \n")
        print "\n Maximum indicated angle exceeds maximum allowed value \n"
        return
    if (TiltMin < MIN_T*degPerStep):
        print "\n ERROR: \n"
        print "Minimum indicated angle exceeds minimum allowed value \n"
        return
    if (sigma < 0.001 or sigma>1):
    	print "\n ERROR: \n"
        print "Indicated resolution out of range: [From 0.001 to 1.0 for full resolution]  \n"
        return
    if (u < MIN_T*degPerStep or u>MAX_T*degPerStep):
        print "\n ERROR: \n"
        print "Indicated interest angle - out of range  \n"
        return
    if (u < TiltMin or u>TiltMax):
        print "\n WARNING: \n"
        print "Indicated interest angle -out of scanning range  \n"
    print ("STARTING SCAN -----------------------------------------------------------")
    print "\n Go2 Initial Position: minimum Tilt: " + str(TiltMin) + "\n"
    PTU.PTU_profile('Fast')
    PTU.Go2Position(1,TiltMin,0)
    PTU.Go2Position(0,0,0)
    PTU.PlatineLight_getPos(1,0)  # to obtain Tilt in Degs
    stepMin = int( TiltMin/degPerStep )
    stepMax = int( TiltMax/degPerStep )
    lastTime=time.time()
    firstTime=lastTime
    error = abs(stepMin - PTU.Tilt/degPerStep)
    while (error > 1.0):
	error = abs(stepMin - PTU.Tilt/degPerStep)
    os.system("rosservice call /platine_light/stop_monitor_position &")
    time.sleep(0.5)
    PTU.PTU_profile('Slow')
    PTU.PlatineLight_getPos(1,0)
    sNum = 0
    sigma = sigma * 250.0
    k = 0
    tilt_ref = TiltMin
    tilt_ref_steps = int(tilt_ref/degPerStep)
    while tilt_ref_steps < stepMax:
	k = k+1
	delta = math.exp( -pow(tilt_ref-u,2) / pow(2*sigma,2) )
	delta = 1.0 - delta
        if abs(delta)<degPerStep:
		delta = degPerStep
	tilt_ref = tilt_ref + delta
	tilt_ref_steps = int(tilt_ref/degPerStep)
        PTU.Go2Position(1,tilt_ref_steps,2)
	sNum = sNum + 1
        error = abs(tilt_ref_steps - PTU.Tilt/degPerStep)
        # Tilt_speed = (Tilt[k]-Tilt[k-1])/dT
        T_speed = abs( PTU.Tilt - PTU.Tilt_1)
        # Tilt_acc = (Tilt_speed[k]-Tilt_speed[k-1])/dT
        T_acc = abs(PTU.Tilt_2 - PTU.Tilt_1)
        while ((error > 1.0) or ((T_speed !=0) or (T_acc !=0)) ):
            error = abs(tilt_ref_steps - PTU.Tilt/degPerStep)
            T_speed = abs(PTU.Tilt - PTU.Tilt_1)
            T_acc   = abs(PTU.Tilt_2 - PTU.Tilt_1)
	lastScan=LiDAR.LDMRS_getData(PTU.Tilt,PTU.Pan)
	if save_mode == 'p':
		cloud.add_ldmrsScan(lastScan)
	else:
        	nP = lastScan.scan_points
		for i in range(0,nP):
            		Tilt_file = lastScan.points[i].Tilt
            		Pan_file  = lastScan.points[i].Pan
            		beta_file = lastScan.points[i].horizontal_angle
            		r_file  = lastScan.points[i].radial_distance
            		layer_file = lastScan.points[i].layer
            		echo_file = lastScan.points[i].echo
            		pulse_width_file = lastScan.points[i].pulse_width
            		flag_file = lastScan.points[i].flags
			df = lastScan
			if moving_mode == 1:
            			dataFile =  str(Tilt_file)+"\t"+ str(Pan_file)+"\t"+  str(beta_file) +"\t"+  str(layer_file)+"\t" +str(r_file) +"\t"+  str(echo_file) +"\t"+  str(pulse_width_file) +"\t"+  str(flag_file)+  "\t" + str(i) + "\t" + str(sNum) + "\t" + str(rover.x) + "\t" + str(rover.y) + "\t" + str(rover.z) + "\t" + str(rover.qw) + "\t" + str(rover.qx) + "\t" + str(rover.qy) + "\t" + str(rover.qz) + "\n"
			else:
				dataFile =  str(Tilt_file)+"\t"+ str(Pan_file)+"\t"+  str(beta_file) +"\t"+  str(layer_file)+"\t" +str(r_file) +"\t"+  str(echo_file) +"\t"+  str(pulse_width_file) +"\t"+  str(flag_file)+  "\t" + str(i) + "\t " + str(sNum) + "\n"
			f.write(dataFile)
			del Tilt_file, Pan_file, beta_file, r_file, layer_file, echo_file, pulse_width_file, flag_file
		del nP
	dT=time.time()-lastTime
        lastTime=time.time()
	mem = py.memory_info()[0]/float(2 ** 20)
        del lastScan, dT, error, T_speed, T_acc, mem
    	sys.stdout.write('\r')
    	j = int(10*(tilt_ref_steps - stepMin)/(stepMax - stepMin))
    	sys.stdout.write("[%-10s] %d%%" % ('='*j, 10*j))
    	sys.stdout.flush()
    mem = py.memory_info()[0]/float(2 ** 20)
    dT=time.time()-firstTime
    print("\n" + "END OF TILT SCAN ----------------------------------------------------------------")
    print ("\t" + "TiltScann memory used: " +str(mem) + "\t Acquisition Time: "+ str(dT))
    print "\t" + "Scanned range: from " + str(TiltMin) + " to " + str(TiltMax) + " [deg]"
    print ( "\t" "Interest region: " + str(u) + " [degs] with sigma = " + str(sigma) )
    print "\t" + "Number of scans: " + str(sNum) + " of " + str(stepMax-stepMin) + " possible scans. Approx: " + str( int( 100.0*sNum/(stepMax-stepMin) )) + " %" 
    print("---------------------------------------------------------------- END OF TILT SCAN")
    # Go2Zero
    os.system("rosservice call /platine_light/stop_monitor_position &")
    time.sleep(0.5)
    PTU.PTU_profile('Fast')
    PTU.Go2Position(1,0,2)
    PTU.Go2Position(0,0,2)
    return cloud

def saveCloud_pkl(cloud, fName):
    fileName= fName + "_" + time.strftime("%d-%m-%y")+'-'+time.strftime("%I-%M-%S")
    completeName = os.path.join(path_data, fileName + '.pkl')
    print "\n -- SAVING DATA --"
    print "\t File: " + fileName
    print "\t Path: " + path_data
    output = open(completeName, 'w')
    #pickle.dump(cloud, output)
    # Pickle the list using the highest protocol available.
    pickle.dump(cloud, output, -1)
    output.close()

def saveCloud_txt(fileName):
    fileName= fileName + "_" + time.strftime("%d-%m-%y")+'-'+time.strftime("%I-%M-%S")
    completeName = os.path.join(path_data, fileName + '.txt')
    f = open(completeName,"a") #opens file with name of "[fileName].txt"
    dataFile= "%" + pointCloud.description + "\n"
    f.write(dataFile)
    dataFile= "%" + "TILT"+"\t"+ "PAN"+"\t"+  "HOR. ANGLE" +"\t"+ "LAYER" +"\t" + "RADIAL DIST."+"\t"+  "ECHO" +"\t"+  "PULSE WIDTH" +"\t"+  "SICK FLAG" + "\t" + "POINT NUMBER" + "\t" + "SCAN NUMBER"  "\t" + "rovX" + "\t" + "rovY"  + "\t" + "rovZ" + "\t" + "rovQw" + "\t" + "rovQx" + "\t" + "rovQy" + "\t" + "rovQz" + "\n"
    f.write(dataFile)
    return f

def main_check():
    if sys.argv[1] == "-h" or sys.argv[1] == "--h":
        print "use: python sickScan.py Tilt_min Tilt_max Tilt_int sigma"
        print "\t Tilt_min:  Minimun angle in deg for TILT 3Dscan"
        print "\t Tilt_max:  Minimun angle in deg for TILT 3Dscan"
        print "\t Tilt_int:  Interest region to scan in deg"
        print "\t sigma:  Scan index for resolution from 0 to 1.0=Full_resolution.  put 0.1 for default"
        sys.exit()
    if len(sys.argv) !=5:
        print ("Unspecified arguments.\nTry with: see -h")
        sys.exit()
    if os.path.isdir(path_data) == False:
        print "Path: " + path_data + " not found."
        sys.exit()
    if os.path.isdir(path_bin) == False:
        print "Path: " + path_bin + " not found."
        sys.exit()
    else:
        if os.path.exists(path_bin + "/platine_light-ros") == False:
            print("File platine_light-ros not found on " + path_bin + " folder")
            sys.exit()
        if os.path.exists(path_bin + "/LiDARldmrs-ros") == False:
            print("File LiDARldmrs-ros not found on"  + path_bin + " folder")
            sys.exit()
    if os.path.exists(dev) == False:
        print("Device tty_PTU not found")
        sys.exit()
    response = os.system("ping -c 1 " + ip_address)
    if response != 0:
        print hostname, 'is down!'
        sys.exit()
    try:
	p1 = check_output(["pidof","./platine_light-ros"])
	p2 = check_output(["pidof","./LiDARldmrs-ros"])
    except:
	print "No process detected."
	print "Warning: Nodes\n see: rosnode list"
	sys.exit()
    try:
	p3 = check_output(["pidof","./rmp440-ros"])
	p4 = check_output(["pidof","./joystick-ros"])
	moving_mode = 1
	print "moving -mode detected "
    except:
	moving_mode = 0
	print "No moving -mode detected "
    print ("main check " + u'\u221A'.encode('utf8'))
    time.sleep(1)
    return moving_mode

# ------------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------------

if __name__ == "__main__":
    moving_mode = main_check()
    # Init Basic 3d Scan Data
    fileName = raw_input("Name of the pointCloud-File? : ")
    [PTU, LiDAR, pointCloud, minnie, save_mode]=initScan(moving_mode)
    if save_mode == 't':
        f = saveCloud_txt(fileName)
    try:
        rospy.init_node('scan3D_client_py')
        time.sleep(1)
	PTU.PlatineLight_set_baudRate()
        PTU.PlatineLight_open_client()
        LiDAR.LDMRS_open_client()
        # scanning parameters in deg
        scan_Tilt_min = float(sys.argv[1])
        scan_Tilt_max = float(sys.argv[2])
        scan_interest = float(sys.argv[3])
	scan_resolution = float(sys.argv[4])
        pointCloud= TiltScan(PTU, LiDAR, pointCloud, minnie,  scan_Tilt_min, scan_Tilt_max, scan_interest, scan_resolution, save_mode, moving_mode)
        PTU.PlatineLight_close()
#        PTU.PlatineLight_kill()
        LiDAR.LDMRS_close_client()
	if moving_mode == 1:
		minnie.stopReading()
	if save_mode=='p':
        	saveCloud_pkl(pointCloud, fileName)
	else:
		f.close()
#        LiDAR.LDMRS_kill()
	print ("FINISH SCAN ---------------------------")
	time.sleep(1)
        sys.exit()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
