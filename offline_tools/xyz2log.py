#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# LAAS-CNRS: Robotic and Interaction Systems
# SICK LDMRS, Platine Light
# Harold F. MURCIA - October 2017

"""
Created on Tue Oct 10 18:13:58 2017

This file convert from XYZ to log file by removing the other columns.
TODO: impr... Acceptance of different formats and possible columns separator
.log files are used by octoMap
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
        data = pd.read_csv( workfile, sep='  ', names = ["tilt", "pan", "beta", "layer","r","echo","wp","flag","point","scan","NR","rovX","rovY","rovZ","rovQw","rovQx","rovQy","rovQz","X","Y","Z"], header=2)
        moving_mode = 1
        data = data.drop(data.columns[[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17]],axis=1)
    except:
        data = pd.read_csv( workfile, sep='\t', names = ["tilt", "pan", "beta", "layer","r","echo","wp","flag","point","scan","NR","rovX","rovY","rovZ","rovQw","rovQx","rovQy","rovQz","X","Y","Z"], header=2)
        moving_mode = 0 
        data = data.drop(data.columns[[0,1,2,3,4,5,6,7,8,9]],axis=1)
    completeName = os.path.join(path_txt+"/octomap/",  txt_name  + ".log")
    dataF = data
    if moving_mode == 1:
        hLine= "#  X [m], Y[m], Z[m]" 
        numFormat = "%3.12f %3.12f %3.12f"
    else:
        hLine= "#  X [m], Y[m], Z[m]" 
        numFormat = "%3.12f %3.12f %3.12f"
    np.savetxt(completeName, dataF.values, fmt= numFormat ,delimiter='\t',header = hLine,  comments="")
    print("(ok)")
