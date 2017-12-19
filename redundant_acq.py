# LAAS-CNRS: Robotic and Interaction Systems
# SICK LDMRS, Platine Light
# Harold F. MURCIA - November 2017
"""
Created on Fri Nov 10 11:01:59 2017

This file contains an example of redundant data acquisition for calibrationg tasks.
The "ref" array have the tilt positions where the system will scan.
Each tilt position will be scanned 1000 times. Data can be saved on .pkl format or .txt format

@author: haroldfmurcia
"""
# ------------------------------------------
import  rospy, os, sys, time, math, getpass, pickle, psutil
import  scan3D
import numpy as np

path_data = os.getcwd() + "/data"
path_bin  = "/home/"+getpass.getuser()+"/ROS_GenoM3/bin"
dev = '/dev/tty_PTU'
ip_address = '192.168.0.2'
ip_port='12002'
pid = os.getpid()
py = psutil.Process(pid)

def initScan():
    PTU   = scan3D.ptu_light(device= dev)
    LiDAR = scan3D.sickLDMRS(ip= ip_address, port= ip_port, scanFindx=1)
    pointCloud = scan3D.LDMRS_CLOUD()
    pointCloud.description = raw_input("Short description of Data: ")
    pointCloud.ip = LiDAR.ip
    pointCloud.port = LiDAR.port
    pointCloud.scanFreq = LiDAR.scanFindx
    save_mode = 't'
    return PTU, LiDAR, pointCloud, save_mode

def TiltScan(PTU, LiDAR, cloud, save_mode):
    #Phisical TiltMax= maxAng #2354  PTU46-> approx. +32 deg
    #Phisical TiltMin= minAng #-3569 PTU46-> approx. -45 deg
    #Phisical Tilt stepRes           PTU46-> approx. 0.0127 deg
    ref=np.array([  -40.4800,
                  -34.3106,
                  -29.8169,
                  -26.3627,
                  -23.6045,
                  -21.3386,
                  -19.4358,
                  -17.8094,
                  -16.3992,
                  -15.1619,
                  -14.0652,
                  -13.0846,
                  -12.2014,
                  -11.4006,
                  -10.6703,
                  -10.0009,
                   -9.3845,
                   -8.8145,
                   -8.2855,
                   -7.7929,
                   -7.3326,
                   -6.9015,
                   -6.4966,
                   -6.1153,
                   -5.7555])
    radsPerStep = PTU.TiltRes
    degPerStep = radsPerStep*180.0/math.pi
    MAX_T = PTU.TiltMax
    MIN_T = PTU.TiltMin
    print ("STARTING SCAN -----------------------------------------------------------")
    print "\n Min Position: minimum Tilt: " + str(MIN_T) + "  MA position: " + str(MAX_T) + "  delta: " + str(degPerStep)
    PTU.Go2Position(1,MIN_T*degPerStep,0)
    PTU.Go2Position(0,0,0)
    PTU.PlatineLight_getPos(1,0)  # to obtain Tilt in Degs
    lastTime = time.time()
    ref=ref/degPerStep
    sNum = 0
    for k in range (0,len(ref)-1):
        sNum = sNum + 1
        angle_steps= int(ref[k])
        PTU.Go2Position(1,angle_steps,2)
        error = abs(angle_steps - PTU.Tilt/degPerStep)
        # Tilt_speed = (Tilt[k]-Tilt[k-1])/dT :
        T_speed = abs( PTU.Tilt - PTU.Tilt_1)
        # Tilt_acc = (Tilt_speed[k]-Tilt_speed[k-1])/dT :
        T_acc = abs(PTU.Tilt_2 - PTU.Tilt_1)
        while ((error > 1.0) or ((T_speed !=0) or (T_acc !=0)) ):
            error = abs(angle_steps - PTU.Tilt/degPerStep)
            T_speed = abs(PTU.Tilt - PTU.Tilt_1)
            T_acc   = abs(PTU.Tilt_2 - PTU.Tilt_1)
        for j in range (1,1000):
            lastScan=LiDAR.LDMRS_getData(PTU.Tilt,PTU.Pan)
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
                dataFile= str(Tilt_file)+"\t"+ str(Pan_file)+"\t"+  str(beta_file) +"\t"+  str(layer_file)+"\t" +str(r_file) +"\t"+  str(echo_file) +"\t"+  str(pulse_width_file) +"\t"+  str(flag_file)+  "\t" + str(i) + "\t" + str(sNum) + "\n"
                f.write(dataFile)
            del Tilt_file, Pan_file, beta_file, r_file, layer_file, echo_file, pulse_width_file, flag_file
        del nP
        dT=time.time()-lastTime
        lastTime=time.time()
        del lastScan, dT, error, T_speed, T_acc
        sys.stdout.write('\r')
        J = int(10*(k - 0)/(len(ref)))
        sys.stdout.write("[%-10s] %d%%" % ('='*J, 10*J))
        sys.stdout.flush()
    mem = py.memory_info()[0]/float(2 ** 20)
    print ("\t" + "TiltScann memory use: " +str(mem))
    print("----------------------------------------------------------- END OF TILT SCAN")
    # Go2Zero
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
    dataFile= "%" + "TILT"+"\t"+ "PAN"+"\t"+  "HOR. ANGLE" +"\t"+ "LAYER" +"\t" + "RADIAL DIST."+"\t"+  "ECHO" +"\t"+  "PULSE WIDTH" +"\t"+  "SICK FLAG" + "\t" + "POINT NUMBER" + "\t" + "SCAN NUMBER"  "\n"
    f.write(dataFile)
    return f

def main_check():
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
    print ("main check " + u'\u221A'.encode('utf8'))

# ------------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------------

if __name__ == "__main__":
    main_check()
    # Init Basic 3d Scan Data
    fileName = raw_input("Name of the pointCloud-File: ")
    [PTU, LiDAR, pointCloud, save_mode]=initScan()
    if save_mode == 't':
        f = saveCloud_txt(fileName)
    try:
        time.sleep(0.1)
    except:
        print("Warning: Nodes\n see: rosnode list")
    try:
        rospy.init_node('scan3D_client_py')
        time.sleep(1)
        PTU.PlatineLight_open_client()
        PTU.PlatineLight_set_baudRate()
        LiDAR.LDMRS_open_client()
	PTU.PTU_profile()
        # scanning parameters in deg
        pointCloud= TiltScan(PTU, LiDAR, pointCloud,save_mode)
        PTU.PlatineLight_close()
        PTU.PlatineLight_kill()
        LiDAR.LDMRS_close_client()
	if save_mode=='p':
        	saveCloud_pkl(pointCloud, fileName)
	else:
		f.close()
        LiDAR.LDMRS_kill()
	time.sleep(1)
        sys.exit()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

