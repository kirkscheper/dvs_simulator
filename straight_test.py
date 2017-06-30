#!/usr/bin/env python

""" 
Author: F. Paredes Valles
Created: 19-06-2017
"""

import os
import glob
import shutil
import rosbag
import struct
from math import floor
from math import fabs
import numpy as np
from PIL import Image
from dataset import *

# initial positions
xStart = [-1.5]
yStart = [-1.5]

# time
t_ = 6000

# circle
x_  = []
y_  = []
alt = 0.5
folders = 'test_circle'
th = np.linspace(0, 2*np.pi, t_)
for i in xrange(0,len(th)):
	x_.append(xStart + np.cos(th[i]))
	y_.append(yStart + np.sin(th[i]))

# allocate memory for the trajectory array
traj 		= np.zeros((50000, 8))
ventralFlow = np.zeros((50000, 4))

# starting point of the trajectory
traj[0,:] = [0, 0.000000, 0.000000, 0.500000, 1.000000, 0.000000, 0.000000, 0.000000]
					
for i in xrange(0, int(t_)):

	# trajectory information
	traj[i,0] = traj[i-1, 0] + 1
	traj[i,1] = y_[i]
 	traj[i,2] = x_[i]
 	traj[i,3] = alt
 	traj[i,4] = 1
 	traj[i,5] = traj[i-1, 5]
 	traj[i,6] = traj[i-1, 6]
 	traj[i,7] = traj[i-1, 7]

# save the file where it should be
with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
	for i in xrange(0,int(t_)):
 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))

# run the simulator
run_simulator()
path = '/media/fedepare/Datos/Ubuntu/Projects/DeepSiameseDVS'
datasetFolder = 'images'
data = dataset(path, datasetFolder)

# generate images
imgStart = 0
imgCnt = data.generate_images(1000, folderName = folders, imgStart = imgStart) # us

# generate groundtruth data
xProc = []
yProc = []
tProc = []
for i in xrange(0,int(t_)):
	tProc.append(int(traj[i, 0]))
	xProc.append(traj[i, 2]) # axis interchanged
	yProc.append(traj[i, 1]) # axis interchanged
data.generate_ventralFlow(tProc, xProc, yProc, alt, imgCnt, imgStart = imgStart)

# clean data generated
path = 'src/rpg_davis_simulator/datasets/full_datasets'
if os.path.exists(path):
	shutil.rmtree(path)

path = 'src/rpg_davis_simulator/datasets/rosbags'
if os.path.exists(path):
	shutil.rmtree(path)
