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
t_ = 1000 # ms

# validation trajectories
cntData = 0
x_ = [2.18, -1.74, -1.04, -1.87, -0.84,  0.16, -0.57, -1.62,  1.44, -1.55]
y_ = [0.29, -1.79, -0.82,  2.11, -1.08, -0.92,  2.14,  1.59, -1.43,  1.06]
z_ = [0 for i in xrange(0,len(x_))]
yInit   = [0.0,  2.5, -1.5, -0.5, -0.5,  0.0, -0.5,  2.0, -2.0, 2.5]
xInit   = [1.0,  0.5, -1.5, -1.5,  2.0, -1.5, -1.0, -2.5,  2.5, 1.5]
zInit   = [1 for i in xrange(0,len(x_))]

text = 'val' + str(int(100)) + '_'
for i in xrange(0,len(x_)):
	folders.append(text + str(cntData))
	cntData += 1

# generate all the datasets
for dataIdx in xrange(0, len(x_)):

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
							
		for i in xrange(cntRow + 1, t_ + cntRow + 1):

			# trajectory information
			traj[i,0] = traj[i-1, 0] + 1
			traj[i,1] = traj[i-1, 1] + deltaX
		 	traj[i,2] = traj[i-1, 2] + deltaY
		 	traj[i,3] = traj[i-1, 3] + deltaZ
		 	traj[i,4] = traj[i-1, 4]
		 	traj[i,5] = traj[i-1, 5]
		 	traj[i,6] = traj[i-1, 6]
		 	traj[i,7] = traj[i-1, 7]

	 		# ventral flow information
			if i == 1: ventralFlow[i-1,0] = 1
			else: ventralFlow[i-1,0] = ventralFlow[i-2,0] + 1
			ventralFlow[i-1,1] = - deltaX * 1000 / traj[i,3] # wx
			ventralFlow[i-1,2] = - deltaY * 1000 / traj[i,3] # wy
			ventralFlow[i-1,3] = - 2 * deltaZ * 1000 / traj[i,3] # D

	 	cntRow += t_

		# save the file where it should be
		with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
			for i in xrange(0,cntRow):
		 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))

		# render and simulate the scene
		os.system("roslaunch dvs_simulator_py custom_render.launch")
		os.system("roslaunch dvs_simulator_py custom_simulate.launch")

		# store last millisecond of the simulation
		lastTime = traj[cntRow - 1,0]

		# accumulate events
		accumTime = 1000

		# get the current rosbag file
		curDir = os.getcwd()
		os.chdir(os.getcwd() + '/src/rpg_davis_simulator/datasets/rosbags/')
		bagFile = max(glob.glob('*.bag'), key=os.path.getctime)
		bagFilePath = os.getcwd() + '/' + bagFile
		os.chdir(curDir)

		# split the name and create the aedat filename
		bagSplit = bagFile.split('.')

		######################################################3
		# NO POLARITY

		# directory for data
		path = '/home/fedepare/Projects/catkin_ws/data/dataset/'

		# check directories for image storage
		if not os.path.exists(path + 'No_Polarity/'):
			os.makedirs(path + 'No_Polarity/')

		imgDir = path + 'No_Polarity/' + folders[dataIdx] + '/'
		accumEvents = np.zeros((128, 128), dtype=np.uint8)
		accumEvents[0, 0] = 0
		imgCnt = 0
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# open the rosbag file and process the events
		bag = rosbag.Bag(bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

				ts = int(e.ts.to_nsec() / 1000.0)

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					accumEvents[e.y, e.x] = 255
				else:
					imgCnt += 1
					if imgCnt >= 10:
						img = Image.fromarray(accumEvents)
						img.save(imgDir + str(imgCnt - 10) + '.png')
					accumEvents = np.zeros((128, 128), dtype=np.uint8)
					accumEvents[0, 0] = 0
					accumEvents[e.y, e.x] = 255

		# store the last image
		imgCnt += 1
		img = Image.fromarray(accumEvents)
		img.save(imgDir + str(imgCnt - 10) + '.png')

		bag.close()

		# generate the ground truth for the set of images generated	
		cnt = 0
		readFlag = False
		with open(imgDir + 'GT_flow.txt', "w") as text_file:

			while True:

				# create the name of the file
				file = str(cnt) + '.png'

				# check wether it is included in the directory
				if os.path.isfile(imgDir + file):

					# update the flag
					readFlag  = True
					time = cnt * accumTime / 1000.0

					# check the values of the ground truth file
					under = int(floor(time))

					vx = (ventralFlow[under-1,1] * (ventralFlow[under,0] - time) + ventralFlow[under,1] * (time - ventralFlow[under-1,0])) / (ventralFlow[under,0] - ventralFlow[under-1,0])
					vy = (ventralFlow[under-1,2] * (ventralFlow[under,0] - time) + ventralFlow[under,2] * (time - ventralFlow[under-1,0])) / (ventralFlow[under,0] - ventralFlow[under-1,0])
					vz = (ventralFlow[under-1,3] * (ventralFlow[under,0] - time) + ventralFlow[under,3] * (time - ventralFlow[under-1,0])) / (ventralFlow[under,0] - ventralFlow[under-1,0])

					# write ventral flow
					text_file.write("%i %.6f %.6f %.6f\n" % (cnt, vx, vy, vz))

				# if the file is not in the directory after reading some files, break
				elif readFlag == True:
					break

				# update the counter
				cnt += 1

		######################################################3
		# POLARITY

		# check directories for image storage
		if not os.path.exists(path + 'Polarity/'):
			os.makedirs(path + 'Polarity/')

		imgDir = path + 'Polarity/' + folders[dataIdx] + '/'
		accumEvents = np.zeros((128, 128), dtype=np.uint8)
		accumEvents[0, 0] = 0
		imgCnt = 0
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# open the rosbag file and process the events
		bag = rosbag.Bag(bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

				ts = int(e.ts.to_nsec() / 1000.0)
				p = '1' if e.polarity else '0'

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					if p == '1':
						accumEvents[e.y, e.x] = 255 # bright
					else:
						accumEvents[e.y, e.x] = 127 # not that bright
						
				else:
					imgCnt += 1
					if imgCnt >= 10:
						img = Image.fromarray(accumEvents)
						img.save(imgDir + str(imgCnt - 10) + '.png')
					accumEvents = np.zeros((128, 128), dtype=np.uint8)
					accumEvents[0, 0] = 0
					if p == '1':
						accumEvents[e.y, e.x] = 255 # green
					else:
						accumEvents[e.y, e.x] = 127 # not that bright

		# store the last image
		imgCnt += 1
		img = Image.fromarray(accumEvents)
		img.save(imgDir + str(imgCnt - 10) + '.png')

		bag.close()

		# the groundtruth file is the same
		os.system("cp -i " + path + 'No_Polarity/' + folders[dataIdx] + '/GT_flow.txt ' + imgDir)

		######################################################3
		# ON EVENTS

		# check directories for image storage
		if not os.path.exists(path + 'ON/'):
			os.makedirs(path + 'ON/')

		imgDir = path + 'ON/' + folders[dataIdx] + '/'
		accumEvents = np.zeros((128, 128), dtype=np.uint8)
		accumEvents[0, 0] = 0
		imgCnt = 0
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# open the rosbag file and process the events
		bag = rosbag.Bag(bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

				ts = int(e.ts.to_nsec() / 1000.0)
				p = '1' if e.polarity else '0'

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					if p == '1':
						accumEvents[e.y, e.x] = 255
				else:
					imgCnt += 1
					if imgCnt >= 10:
						img = Image.fromarray(accumEvents)
						img.save(imgDir + str(imgCnt - 10) + '.png')
					accumEvents = np.zeros((128, 128), dtype=np.uint8)
					accumEvents[0, 0] = 0
					if p == '1':
						accumEvents[e.y, e.x] = 255

		# store the last image
		imgCnt += 1
		img = Image.fromarray(accumEvents)
		img.save(imgDir + str(imgCnt - 10) + '.png')

		bag.close()

		# the groundtruth file is the same
		os.system("cp -i " + path + 'No_Polarity/' + folders[dataIdx] + '/GT_flow.txt ' + imgDir)

		######################################################3
		# OFF EVENTS

		# check directories for image storage
		if not os.path.exists(path + 'OFF/'):
			os.makedirs(path + 'OFF/')

		imgDir = path + 'OFF/' + folders[dataIdx] + '/'
		accumEvents = np.zeros((128, 128), dtype=np.uint8)
		accumEvents[0, 0] = 0
		imgCnt = 0
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# open the rosbag file and process the events
		bag = rosbag.Bag(bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

				ts = int(e.ts.to_nsec() / 1000.0)
				p = '1' if e.polarity else '0'

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					if p == '0':
						accumEvents[e.y, e.x] = 255
				else:
					imgCnt += 1
					if imgCnt >= 10:
						img = Image.fromarray(accumEvents)
						img.save(imgDir + str(imgCnt - 10) + '.png')
					accumEvents = np.zeros((128, 128), dtype=np.uint8)
					accumEvents[0, 0] = 0
					if p == '0':
						accumEvents[e.y, e.x] = 255

		# store the last image
		imgCnt += 1
		img = Image.fromarray(accumEvents)
		img.save(imgDir + str(imgCnt - 10) + '.png')

		bag.close()

		# the groundtruth file is the same
		os.system("cp -i " + path + 'No_Polarity/' + folders[dataIdx] + '/GT_flow.txt ' + imgDir)

		# clean data generated
		path = 'src/rpg_davis_simulator/datasets/full_datasets'
		if os.path.exists(path):
			shutil.rmtree(path)

		path = 'src/rpg_davis_simulator/datasets/rosbags'
		if os.path.exists(path):
			shutil.rmtree(path)