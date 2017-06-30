#!/usr/bin/env python

""" 
Author: F. Paredes Valles
Created: 04-05-2017
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

def generate_relPose(tProc, xProc, yProc, altitude, folderName = '', path = '', datasetFolder = ''):

	path   = path + '/' + datasetFolder + '/' + folderName
	imgCnt = len(glob.glob1(path, "*.png"))

	# write the txt file
	with open(path + '/GT_pose.txt', 'w') as text_file:
		for i in xrange(0, imgCnt):
	 		text_file.write("%i %.6f %.6f %.6f\n" % (i+1, xProc[i], yProc[i], altitude))


# Start at
Start = 0

# initialize variables
x_ = []
y_ = []
z_ = []
xInit = []
yInit = []
zInit = []
folders = []

# all the trajectories last the same
t_ = 1000.0 # ms

# compute the angles
thDelta = 33.5
thEnd   = (360-thDelta)*np.pi/180.0
th      = np.linspace(0, thEnd, num=360/thDelta)

# velocity vector
velocities = [0.375, 0.625, 0.89, 1.16, 1.375, 1.62]

# initial positions
xStart = [1.5]
yStart = [1.5]

# test for a fixed altitude of 0.5m
cntData = 0
alt     = 0.5
folders = []
for i in xrange(0,len(th)):
	for ii in xrange(0,len(velocities)):
		x_.append(velocities[ii]*np.sin(th[i]))
		y_.append(velocities[ii]*np.cos(th[i]))
		z_.append(0)
		xInit.append(xStart[0])
		yInit.append(yStart[0])
		zInit.append(alt)
		folders.append(cntData)
		cntData += 1

# check the x_=0 and y_=0 index
noValid = []
for dataIdx in xrange(0, len(x_)):
	if x_[dataIdx] == 0 and y_[dataIdx] == 0 and z_[dataIdx] == 0:
		noValid.append(dataIdx)

# generate all the datasets
for dataIdx in xrange(0, len(x_)):

	if dataIdx in noValid:
		continue

	if dataIdx >= Start:

		# allocate memory for the trajectory array
		traj 		= np.zeros((50000, 8))
		ventralFlow = np.zeros((50000, 4))

		# starting point of the trajectory
		traj[0,:] = [0, 0.000000, 0.000000, 0.500000, 1.000000, 0.000000, 0.000000, 0.000000]

		# custom starting point
		traj[0, 1] = xInit[dataIdx]
		traj[0, 2] = yInit[dataIdx]
		traj[0, 3] = zInit[dataIdx]

		# generate the desired trajectory
		cntRow = 0

		# compute the trajectory for this segment
		deltaX = y_[dataIdx] / t_ # X and Y in the camera reference frame are interchanged
		deltaY = x_[dataIdx] / t_ # X and Y in the camera reference frame are interchanged
		deltaZ = - z_[dataIdx] / t_
							
		for i in xrange(cntRow + 1, int(t_) + cntRow + 1):

			# trajectory information
			traj[i,0] = traj[i-1, 0] + 1
			traj[i,1] = traj[i-1, 1] + deltaX
		 	traj[i,2] = traj[i-1, 2] + deltaY
		 	traj[i,3] = traj[i-1, 3] + deltaZ
		 	traj[i,4] = traj[i-1, 4]
		 	traj[i,5] = traj[i-1, 5]
		 	traj[i,6] = traj[i-1, 6]
		 	traj[i,7] = traj[i-1, 7]

	 	cntRow += int(t_)

		# save the file where it should be
		with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
			for i in xrange(0,cntRow):
		 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))

 	 	# run the simulator
 	 	run_simulator()
 		path = '/media/fedepare/Datos/Ubuntu/Projects/DeepDVS'
		datasetFolder = 'images_dual'
		data = dataset(path, datasetFolder)

		# generate images
		imgStart = 0
		imgCnt = data.generate_images(1000, folderName = 'val_' + str(folders[dataIdx]), imgStart = imgStart) # us

		# generate groundtruth data
		xProc = []
		yProc = []
		tProc = []
		for i in xrange(0,cntRow):
			tProc.append(int(traj[i, 0]))
			xProc.append(traj[i, 2]) # axis interchanged
			yProc.append(traj[i, 1]) # axis interchanged
		data.generate_ventralFlow(tProc, xProc, yProc, alt, imgCnt, imgStart = imgStart)

		generate_relPose(tProc, xProc, yProc, alt, folderName = 'val_' + str(folders[dataIdx]), path = path, datasetFolder = datasetFolder)

		# clean data generated
		path = 'src/rpg_davis_simulator/datasets/full_datasets'
		if os.path.exists(path):
			shutil.rmtree(path)

		path = 'src/rpg_davis_simulator/datasets/rosbags'
		if os.path.exists(path):
			shutil.rmtree(path)