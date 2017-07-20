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

# Start at
Start = 53

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
thDelta = 15
thEnd   = (360-thDelta)*np.pi/180.0
th      = np.linspace(0, thEnd, num=360/15)

# velocity vector
velocities = [0.25, 0.5, 0.75, 1.0, 1.25, 1.5]

# initial positions
xStart = [0]
yStart = [0]

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

# generate all the datasets
for dataIdx in xrange(0, len(x_)):

	if dataIdx >= Start and dataIdx < 54:

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
 	 	os.system("roslaunch dvs_simulator_py custom_render.launch")

 	 	# copy the ext folder to the desired path
 	 	pathTo = '/media/fedepare/Datos/Ubuntu/Projects/images_prueba'
 	 	if not os.path.exists(pathTo): os.makedirs(pathTo)
		pathFrom = os.getcwd() + '/src/rpg_davis_simulator/datasets/full_datasets/3planesDVSext/data/exr'
		os.system('cp -a ' + pathFrom + ' ' + pathTo)

		# rename the folder
		os.system('mv ' + pathTo + '/exr ' + pathTo + '/' + str(folders[dataIdx]))

		# convert the .exr files into .png
		curDir = os.getcwd()
		nxtDir = pathTo + '/' + str(folders[dataIdx])
		os.chdir(nxtDir)
		os.system("mogrify -format png *.exr")
		os.system("find . -type f -iname \*.exr -delete")
		#os.system("mogrify -colorspace Gray *.png")

		# rename the files
		pngs  = len(glob.glob1(nxtDir,"*.png"))
		for i in xrange(0,pngs):
			file = '%04d.png' % i
			os.system('mv ' + file + ' ' + str(i) + '.png')

		# delete extra files
		# gt    = np.loadtxt('GT_flow.txt')
		# gtNum = gt[-1, 0]
		# os.system('rm 0.png')
		# for i in xrange(int(gtNum) + 1, int(pngs)): os.system('rm ' + str(i) + '.png')

		# go back to the original directory
		os.chdir(curDir)

		# clean data generated
		path = 'src/rpg_davis_simulator/datasets/full_datasets'
		if os.path.exists(path):
			shutil.rmtree(path)
