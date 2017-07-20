#!/usr/bin/env python

""" 
Author: F. Paredes Valles
Created: 19-06-2017
"""

import os
import shutil
from dataset_bagfiles import *

# initial positions
xStart = [-1.5]
yStart = [-1.5]

# time
t_ = 6000

# circle
x_  = []
y_  = []
alt = 0.5
folders = 'circle_checker_150'
th = np.linspace(0, 2*np.pi, t_)
for i in xrange(0,len(th)):
	x_.append(xStart + 1.50*np.cos(th[i]))
	y_.append(yStart + 1.50*np.sin(th[i]))

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

# save the file where it should be
with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
	for i in xrange(0, int(t_) + 1):
 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))

# run the simulator
run_simulator(texture = 'checkerboard')
path = '/media/fedepare/Datos/Ubuntu/Projects/bagfiles'
data = dataset(path, folderName = folders)
data.copy_bagfile(trajectory = traj, time = int(t_))

# clean data generated
path = 'src/rpg_davis_simulator/datasets/full_datasets'
if os.path.exists(path):
	shutil.rmtree(path)

path = 'src/rpg_davis_simulator/datasets/rosbags'
if os.path.exists(path):
	shutil.rmtree(path)