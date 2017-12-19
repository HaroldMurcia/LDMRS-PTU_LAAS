#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# LAAS-CNRS: Robotic and Interaction Systems
# SICK LDMRS, Platine Light
# Harold F. MURCIA - October 2017

"""
Created on Tue Oct 10 18:13:58 2017

This file include the calibrated parameters to calculates the local XYZ positions of the point clouds.
TODO: Add the position of the rover to calculate the absolute point cloud

To use it, execute: >> python raw2txt.py file_path

@author: haroldfmurcia
"""

import math, os, sys
import numpy as np
import pandas as pd


if __name__ == "__main__":
    path   = sys.argv[1]
    path_s = path.split("/")
    L = len(path_s)
    path_txt =""
    for k in range(1,L-1):
        path_txt = path_txt + "/" + path_s[k]
    txt_name = path_s[-1]
    txt_name = txt_name[0:-4]
    workfile = os.path.join(path_txt, txt_name + ".txt")
    try:
        data = pd.read_csv( workfile, sep='\t', names = ["tilt", "pan", "beta", "layer","r","echo","wp","flag","point","scan","rovX","rovY","rovZ","rovQw","rovQx","rovQy","rovQz"], header=2)
        moving_mode = 1
    except:
        data = pd.read_csv( workfile, sep='\t', names = ["tilt", "pan", "beta", "layer","r","echo","wp","flag","point","scan"], header=2)
        moving_mode = 0
    # ECHOES = 0
    data= data.reset_index(); del data['index']
    nP=data.shape[0]
    NUM_RET=np.ones([nP,1]);
    df=data[(data.duplicated(["beta","layer","scan"],keep=False)) ]
    df['ind'] = df.index
    groups = df.groupby(['beta', 'layer', 'scan'])
    df.set_index(['beta', 'layer', 'scan'], inplace=True)
    df['ordering'] = groups.size()
    nP2 = df.shape[0]
    NR = df.ordering[1:nP2-1]
    NR = np.array(NR)
    pos= df.ind[1:nP2-1]
    pos = np.array(pos)
    NUM_RET[pos,[0]]=NR
    PI = math.pi
    X=np.zeros([data.beta.shape[0],1])
    Y=np.zeros([data.beta.shape[0],1])
    Z=np.zeros([data.beta.shape[0],1])
    # Parameters
    # Intrinsic
    L1z         =   0.100578984193110;
    L2x         =  -0.023760000000000;
    L2y         =   0.018797850189242;
    L2z         =   0.083186331806302;
    alpha_1     =  -0.026310185433179;
    alpha_2     =  -0.007321821152752;
    alpha_3     =   0.008640101362527;
    alpha_4     =   0.023020415513343;
    beta_off1   =  1.049615948354090e-10;
    beta_off2   =  -1.244528810882754e-11;
    tilt_off    =  -3.962113176508846e-05;
    # Extrinsic
    Lox         =   8.101654718704621e-04;
    Loy         =   0.610023964751478;
    Loz         =   0.698845572764671
    pitch       =   0.022730176585901
    roll        =   0.008251591674425
    yaw         =   -0.018670163578264;
    # Data
    data_layer1 = data[(data["layer"] == 0) ]
    data_layer2 = data[(data["layer"] == 1) ]
    data_layer3 = data[(data["layer"] == 2) ]
    data_layer4 = data[(data["layer"] == 3) ]
    data_layer1_ind = np.array( data_layer1.index )
    data_layer2_ind = np.array( data_layer2.index )
    data_layer3_ind = np.array( data_layer3.index )
    data_layer4_ind = np.array( data_layer4.index )
    tilt_1 = np.array(data_layer1.tilt)*PI/180.0 + tilt_off
    tilt_2 = np.array(data_layer2.tilt)*PI/180.0 + tilt_off
    tilt_3 = np.array(data_layer3.tilt)*PI/180.0 + tilt_off
    tilt_4 = np.array(data_layer4.tilt)*PI/180.0 + tilt_off
    r_1    = np.array(data_layer1.r)
    r_2    = np.array(data_layer2.r)
    r_3    = np.array(data_layer3.r)
    r_4    = np.array(data_layer4.r)
    beta_1 = np.array(data_layer1.beta) + beta_off1
    beta_2 = np.array(data_layer2.beta) + beta_off1
    beta_3 = np.array(data_layer3.beta) + beta_off2
    beta_4 = np.array(data_layer4.beta) + beta_off2
    # Data from Robot odometry
    off_x = 0
    off_y = 0
    yaw_robot = 0
    # Transformation Equations
    # X for Layer 1, 2, 3 and 4
    X_1 = off_x + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + Lox*np.cos(yaw_robot) - Loy*np.sin(yaw_robot) - L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + r_1*np.sin(alpha_1)*(np.cos(tilt_1)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + np.cos(pitch)*np.sin(tilt_1)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - r_1*np.cos(alpha_1)*np.sin(beta_1)*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + r_1*np.cos(alpha_1)*np.cos(beta_1)*(np.sin(tilt_1)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - np.cos(pitch)*np.cos(tilt_1)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)))
    X_2 = off_x + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + Lox*np.cos(yaw_robot) - Loy*np.sin(yaw_robot) - L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + r_2*np.sin(alpha_2)*(np.cos(tilt_2)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + np.cos(pitch)*np.sin(tilt_2)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - r_2*np.cos(alpha_2)*np.sin(beta_2)*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + r_2*np.cos(alpha_2)*np.cos(beta_2)*(np.sin(tilt_2)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - np.cos(pitch)*np.cos(tilt_2)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)))
    X_3 = off_x + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + Lox*np.cos(yaw_robot) - Loy*np.sin(yaw_robot) - L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + r_3*np.sin(alpha_3)*(np.cos(tilt_3)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + np.cos(pitch)*np.sin(tilt_3)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - r_3*np.cos(alpha_3)*np.sin(beta_3)*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + r_3*np.cos(alpha_3)*np.cos(beta_3)*(np.sin(tilt_3)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - np.cos(pitch)*np.cos(tilt_3)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)))
    X_4 = off_x + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + Lox*np.cos(yaw_robot) - Loy*np.sin(yaw_robot) - L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + r_4*np.sin(alpha_4)*(np.cos(tilt_4)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + np.cos(pitch)*np.sin(tilt_4)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - r_4*np.cos(alpha_4)*np.sin(beta_4)*(np.cos(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) - np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) + r_4*np.cos(alpha_4)*np.cos(beta_4)*(np.sin(tilt_4)*(np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw))) - np.cos(pitch)*np.cos(tilt_4)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)))
    # Y
    Y_1 = off_y + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + Loy*np.cos(yaw_robot) + Lox*np.sin(yaw_robot) + L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + r_1*np.sin(alpha_1)*(np.cos(tilt_1)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - np.cos(pitch)*np.sin(tilt_1)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - r_1*np.cos(alpha_1)*np.sin(beta_1)*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + r_1*np.cos(alpha_1)*np.cos(beta_1)*(np.sin(tilt_1)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + np.cos(pitch)*np.cos(tilt_1)*(np.cos(yaw_robot)*np.cos(yaw) -  np.sin(yaw_robot)*np.sin(yaw)));
    Y_2 = off_y + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + Loy*np.cos(yaw_robot) + Lox*np.sin(yaw_robot) + L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + r_2*np.sin(alpha_2)*(np.cos(tilt_2)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - np.cos(pitch)*np.sin(tilt_2)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - r_2*np.cos(alpha_2)*np.sin(beta_2)*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + r_2*np.cos(alpha_2)*np.cos(beta_2)*(np.sin(tilt_2)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + np.cos(pitch)*np.cos(tilt_2)*(np.cos(yaw_robot)*np.cos(yaw) -  np.sin(yaw_robot)*np.sin(yaw)));
    Y_3 = off_y + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + Loy*np.cos(yaw_robot) + Lox*np.sin(yaw_robot) + L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + r_3*np.sin(alpha_3)*(np.cos(tilt_3)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - np.cos(pitch)*np.sin(tilt_3)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - r_3*np.cos(alpha_3)*np.sin(beta_3)*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + r_3*np.cos(alpha_3)*np.cos(beta_3)*(np.sin(tilt_3)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + np.cos(pitch)*np.cos(tilt_3)*(np.cos(yaw_robot)*np.cos(yaw) -  np.sin(yaw_robot)*np.sin(yaw)));
    Y_4 = off_y + L2x*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L1z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + L2z*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + Loy*np.cos(yaw_robot) + Lox*np.sin(yaw_robot) + L2y*np.cos(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw)) + r_4*np.sin(alpha_4)*(np.cos(tilt_4)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - np.cos(pitch)*np.sin(tilt_4)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) - r_4*np.cos(alpha_4)*np.sin(beta_4)*(np.cos(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) + np.sin(pitch)*np.sin(roll)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + r_4*np.cos(alpha_4)*np.cos(beta_4)*(np.sin(tilt_4)*(np.sin(roll)*(np.cos(yaw_robot)*np.sin(yaw) + np.sin(yaw_robot)*np.cos(yaw)) - np.cos(roll)*np.sin(pitch)*(np.cos(yaw_robot)*np.cos(yaw) - np.sin(yaw_robot)*np.sin(yaw))) + np.cos(pitch)*np.cos(tilt_4)*(np.cos(yaw_robot)*np.cos(yaw) -  np.sin(yaw_robot)*np.sin(yaw)));
    # Z
    Z_1 = Loz + L2y*np.sin(pitch) - r_1*np.sin(alpha_1)*(np.sin(pitch)*np.sin(tilt_1) - np.cos(pitch)*np.cos(roll)*np.cos(tilt_1)) + L1z*np.cos(pitch)*np.cos(roll) + L2z*np.cos(pitch)*np.cos(roll) - L2x*np.cos(pitch)*np.sin(roll) + r_1*np.cos(alpha_1)*np.cos(beta_1)*(np.cos(tilt_1)*np.sin(pitch) + np.cos(pitch)*np.cos(roll)*np.sin(tilt_1)) + r_1*np.cos(alpha_1)*np.cos(pitch)*np.sin(beta_1)*np.sin(roll)
    Z_2 = Loz + L2y*np.sin(pitch) - r_2*np.sin(alpha_2)*(np.sin(pitch)*np.sin(tilt_2) - np.cos(pitch)*np.cos(roll)*np.cos(tilt_2)) + L1z*np.cos(pitch)*np.cos(roll) + L2z*np.cos(pitch)*np.cos(roll) - L2x*np.cos(pitch)*np.sin(roll) + r_2*np.cos(alpha_2)*np.cos(beta_2)*(np.cos(tilt_2)*np.sin(pitch) + np.cos(pitch)*np.cos(roll)*np.sin(tilt_2)) + r_2*np.cos(alpha_2)*np.cos(pitch)*np.sin(beta_2)*np.sin(roll)
    Z_3 = Loz + L2y*np.sin(pitch) - r_3*np.sin(alpha_3)*(np.sin(pitch)*np.sin(tilt_3) - np.cos(pitch)*np.cos(roll)*np.cos(tilt_3)) + L1z*np.cos(pitch)*np.cos(roll) + L2z*np.cos(pitch)*np.cos(roll) - L2x*np.cos(pitch)*np.sin(roll) + r_3*np.cos(alpha_3)*np.cos(beta_3)*(np.cos(tilt_3)*np.sin(pitch) + np.cos(pitch)*np.cos(roll)*np.sin(tilt_3)) + r_3*np.cos(alpha_3)*np.cos(pitch)*np.sin(beta_3)*np.sin(roll)
    Z_4 = Loz + L2y*np.sin(pitch) - r_4*np.sin(alpha_4)*(np.sin(pitch)*np.sin(tilt_4) - np.cos(pitch)*np.cos(roll)*np.cos(tilt_4)) + L1z*np.cos(pitch)*np.cos(roll) + L2z*np.cos(pitch)*np.cos(roll) - L2x*np.cos(pitch)*np.sin(roll) + r_4*np.cos(alpha_4)*np.cos(beta_4)*(np.cos(tilt_4)*np.sin(pitch) + np.cos(pitch)*np.cos(roll)*np.sin(tilt_4)) + r_4*np.cos(alpha_4)*np.cos(pitch)*np.sin(beta_4)*np.sin(roll)
    #
    X[data_layer1_ind,0] = X_1
    Y[data_layer1_ind,0] = Y_1
    Z[data_layer1_ind,0] = Z_1
    X[data_layer2_ind,0] = X_2
    Y[data_layer2_ind,0] = Y_2
    Z[data_layer2_ind,0] = Z_2
    X[data_layer3_ind,0] = X_3
    Y[data_layer3_ind,0] = Y_3
    Z[data_layer3_ind,0] = Z_3
    X[data_layer4_ind,0] = X_4
    Y[data_layer4_ind,0] = Y_4
    Z[data_layer4_ind,0] = Z_4
    completeName = os.path.join(path_txt[:-3]+"xyz",  txt_name + "_xyz" + ".txt")
    dataF = data
    dataF['Num Rets'] = NUM_RET
    dataF['X'] = X
    dataF['Y'] = Y
    dataF['Z'] = Z
    if moving_mode == 1:
        hLine= "% Tilt [rads], Pan [rads], Beta [rads], Layer, Radial Distance [m], Echo, Pulse width [cm], Flag, Point Number, Scan Number, rovX[m], rovY[m], rovZ[m], rovQw, rovQx, rovQy, rovQz,  Return Number, X [m], Y[m], Z[m]" 
        numFormat = "%2.12f %2.12f %2.12f %d %3.12f %d %2.12f %d %d %d %3.12f %3.12f %3.12f %3.12f %3.12f %3.12f %3.12f %d %3.12f %3.12f %3.12f"
    else:
        hLine= "% Tilt [rads], Pan [rads], Beta [rads], Layer, Radial Distance [m], Echo, Pulse width [cm], Flag, Point Number, Scan Number, Return Number, X [m], Y[m], Z[m]" 
        numFormat = "%2.12f %2.12f %2.12f %d %3.12f %d %2.12f %d %d %d %d %3.12f %3.12f %3.12f"
    np.savetxt(completeName, dataF.values, fmt= numFormat ,delimiter='\t',header = hLine,  comments="")
    print("(ok)")
