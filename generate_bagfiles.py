#!/usr/bin/env python

""" 
Author: F. Paredes Valles
Created: 27-06-2017
"""

import os
import shutil
from dataset_bagfiles import *


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
thDelta = 15
thEnd   = (360-thDelta)*np.pi/180.0
th      = np.linspace(0, thEnd, num=360/15)
th = [30 * np.pi/180.0]

# velocity vector
velocities = [0.25, 0.5, 0.75, 1.0, 1.25, 1.5]
velocities = [1.25]

# initial positions
xStart = [0, -1.5, 2]
yStart = [0, -1.5, -0.5]

xStart = [-1.5]
yStart = [-1.5]


# test for a fixed altitude of 0.5m
cntData = 0
alt     = 0.5
folders = []
for iii in xrange(0,len(xStart)):
	for i in xrange(0,len(th)):
		for ii in xrange(0,len(velocities)):
			x_.append(velocities[ii]*np.sin(th[i]))
			y_.append(velocities[ii]*np.cos(th[i]))
			z_.append(0)
			xInit.append(xStart[iii])
			yInit.append(yStart[iii])
			zInit.append(alt)
			folders.append(cntData)
			cntData += 1

# generate all the datasets
for dataIdx in xrange(0, len(x_)):

	if dataIdx >= Start:

		# allocate memory for the trajectory array
		traj = np.zeros((50000, 8))

		# starting point of the trajectory
		traj[0,:] = [0, 0.000000, 0.000000, 0.500000, 1.000000, 0.000000, 0.000000, 0.000000]

		# custom starting point
		traj[0, 1] = xInit[dataIdx]
		traj[0, 2] = yInit[dataIdx]
		traj[0, 3] = zInit[dataIdx]

		# compute the trajectory for this segment
		deltaX = y_[dataIdx] / t_ # X and Y in the camera reference frame are interchanged
		deltaY = x_[dataIdx] / t_ # X and Y in the camera reference frame are interchanged
		deltaZ = - z_[dataIdx] / t_
							
		for i in xrange(1, int(t_) + 1):

			# trajectory information
			traj[i,0] = traj[i-1, 0] + 1
			traj[i,1] = traj[i-1, 1] + deltaX
		 	traj[i,2] = traj[i-1, 2] + deltaY
		 	traj[i,3] = traj[i-1, 3] + deltaZ
		 	traj[i,4] = traj[i-1, 4]
		 	traj[i,5] = traj[i-1, 5]
		 	traj[i,6] = traj[i-1, 6]
		 	traj[i,7] = traj[i-1, 7]

		# save the file where it should be
		with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
			for i in xrange(0, int(t_) + 1):
		 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))

 	 	# run the simulator
 	 	run_simulator(texture = 'natural')
 		path = '/media/fedepare/Datos/Ubuntu/Projects/bagfiles'
		data = dataset(path, folderName = str(folders[dataIdx]))
		data.copy_bagfile(trajectory = traj, time = int(t_))

		# clean data generated
		path = 'src/rpg_davis_simulator/datasets/full_datasets'
		if os.path.exists(path):
			shutil.rmtree(path)

		path = 'src/rpg_davis_simulator/datasets/rosbags'
		if os.path.exists(path):
			shutil.rmtree(path)