#!/usr/bin/env python

import os
import shutil
from dataset_bagfiles import *

# define set tpye (train, val, test)
set_type = 'train'

# total time to complete the circle
t_ = [500, 1000, 1500, 2000] # ms

# initial positions
xStart = [-1, 1, 0.5]
yStart = [-1, 1, -2]

# circle
alt = 0.5

for iii in xrange(0,len(xStart)):
	for ii in xrange(0,len(t_)):
		
		# allocate memory for the trajectory array
		traj 		= np.zeros((50000, 8))
		ventralFlow = np.zeros((50000, 4))
		
		# starting point of the trajectory
		traj[0,:] = [0, 0.000000, 0.000000, 0.500000, 1.000000, 0.000000, 0.000000, 0.000000]
		
		th = np.linspace(0, 2*np.pi, t_[ii])
		for i in xrange(0,len(th)):
			
			# trajectory information
			traj[i,0] = traj[i-1, 0] + 1
			traj[i,1] = yStart[iii] + 1.00*np.sin(th[i])
		 	traj[i,2] = xStart[iii] + 1.00*np.cos(th[i])
		 	traj[i,3] = alt
		 	traj[i,4] = 1
		 	traj[i,5] = traj[i-1, 5]
		 	traj[i,6] = traj[i-1, 6]
		 	traj[i,7] = traj[i-1, 7]
		
		folder = set_type + '_circle_' + str(xStart[iii]) +  '_' + str(yStart[iii]) + '_' + str(t_[ii])
							
		# save the file where it should be
		with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
			for i in xrange(0,int(t_[ii])):
		 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))
		
		# save the file where it should be
		with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
			for i in xrange(0, int(t_[ii]) + 1):
		 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))
		
		# run the simulator
		run_simulator(texture = 'roadmap')
		path = 'generated_datasets/bagfiles'
		data = dataset(path, folderName = folder)
		data.copy_bagfile(trajectory = traj, time = int(t_[ii]))
		
		# clean data generated
		path = 'src/rpg_davis_simulator/datasets/full_datasets'
		if os.path.exists(path):
			shutil.rmtree(path)
		
		path = 'src/rpg_davis_simulator/datasets/rosbags'
		if os.path.exists(path):
			shutil.rmtree(path)
