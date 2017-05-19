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

# number of segments in this trajectory
numSegments = 1

x_ = []
y_ = []
z_ = []
xInit = []
yInit = []
zInit = []
folders = []

# validate1
x_.append(-2.25)
y_.append(1.25)
z_.append(1.0)
xInit.append(-2.6)
yInit.append(1)
zInit.append(1.0)
folders.append('validate1')

# validate2
x_.append(2.25)
y_.append(-1.75)
z_.append(1.0)
xInit.append(2.8)
yInit.append(-0.75)
zInit.append(1.0)
folders.append('validate2')

# validate3
x_.append(2.57)
y_.append(1.46)
z_.append(1.0)
xInit.append(2.4)
yInit.append(-0.35)
zInit.append(1.0)
folders.append('validate3')

# validate4
x_.append(-0.87)
y_.append(-0.40)
z_.append(1.0)
xInit.append(0.18)
yInit.append(-0.7)
zInit.append(1.0)
folders.append('validate4')

# validate5
x_.append(-0.84)
y_.append(0.38)
z_.append(1.0)
xInit.append(0.64)
yInit.append(1.24)
zInit.append(1.0)
folders.append('validate5')

# validate6
x_.append(1.29)
y_.append(-0.47)
z_.append(1.0)
xInit.append(1.89)
yInit.append(-0.35)
zInit.append(1.0)
folders.append('validate6')

# validate7
x_.append(-1.15)
y_.append(-1.15)
z_.append(1.0)
xInit.append(-1.98)
yInit.append(-0.84)
zInit.append(1.0)
folders.append('validate7')

# validate8
x_.append(-1.79)
y_.append(-0.35)
z_.append(1.0)
xInit.append(-1.65)
yInit.append(0.19)
zInit.append(1.0)
folders.append('validate8')

# validate9
x_.append(-0.21)
y_.append(-0.98)
z_.append(1.0)
xInit.append(-0.87)
yInit.append(-1.34)
zInit.append(1.0)
folders.append('validate9')

# validate10
x_.append(-0.87)
y_.append(0.98)
z_.append(1.0)
xInit.append(-0.21)
yInit.append(1.34)
zInit.append(1.0)
folders.append('validate10')

# generate all the datasets
for dataIdx in xrange(0, len(x_)):

	# allocate memory for the trajectory array
	traj 		= np.zeros((int(numSegments * 50000), 8))
	ventralFlow = np.zeros((int(numSegments * 50000), 4))
	directions  = np.zeros((int(numSegments * 50000), 7))

	# the maximum time to be recorded is 50 seconds
	timePerSegment = (50000 / numSegments) * 10**(-3)

	# starting point of the trajectory
	traj[0,:] = [0, 0.000000, 0.000000, 0.500000, 1.000000, 0.000000, 0.000000, 0.000000]

	# custom starting point
	traj[0, 1] = xInit[dataIdx]
	traj[0, 2] = yInit[dataIdx]
	traj[0, 3] = zInit[dataIdx]

	# generate the desired trajectory
	cntRow = 0

	# all the trajectories last one second
	t_ = int(0.5 * 1000)

	# compute the trajectory for this segment
	deltaX = (x_[dataIdx] - traj[cntRow, 1]) / t_
	deltaY = (y_[dataIdx] - traj[cntRow, 2]) / t_
	deltaZ = (z_[dataIdx] - traj[cntRow, 3]) / t_
						
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
 		ventralFlow[i-1,1] = - deltaX * 1000 / traj[i,3] # vx
 		ventralFlow[i-1,2] = deltaY * 1000 / traj[i,3] # vy
 		ventralFlow[i-1,3] = - deltaZ * 1000 / traj[i,3] # vz

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
	accumFlag = True
	accumTime = 1000

	# get the current rosbag file
	curDir = os.getcwd()
	os.chdir(os.getcwd() + '/src/rpg_davis_simulator/datasets/rosbags/')
	bagFile = max(glob.glob('*.bag'), key=os.path.getctime)
	bagFilePath = os.getcwd() + '/' + bagFile
	os.chdir(curDir)

	# split the name and create the aedat filename
	bagSplit = bagFile.split('.')

	# check directories for image storage
	if accumFlag:
		if not os.path.exists('aedat/images/'):
			os.makedirs('aedat/images/')
		imgDir = 'aedat/images/' + folders[dataIdx] + '/'
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
	        x = '{0:07b}'.format(e.x)
	        y = '{0:07b}'.format(e.y)
	        p = '1' if e.polarity else '0'
	        address = "0" + y + x + p

	        # accumulate events in an image
	        if accumFlag:
				if ts / accumTime == imgCnt:
					if accumEvents[-e.y, -e.x] == 0:
						accumEvents[-e.y, -e.x] = 255
				else:
					imgCnt += 1
					if imgCnt >= 10:
						img = Image.fromarray(accumEvents)
						img.save(imgDir + str(imgCnt - 10) + '.png')
					accumEvents = np.zeros((128, 128), dtype=np.uint8)
					accumEvents[0, 0] = 0
					accumEvents[-e.y, -e.x] = 255

	# store the last image
	if accumFlag:
		imgCnt += 1
		img = Image.fromarray(accumEvents)
		img.save(imgDir + str(imgCnt - 10) + '.png')

	bag.close()

	# generate the ground truth for the set of images generated
	if accumFlag:
			
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

	# clean data generated
	path = 'src/rpg_davis_simulator/datasets/full_datasets'
	if os.path.exists(path):
		shutil.rmtree(path)

	path = 'src/rpg_davis_simulator/datasets/rosbags'
	if os.path.exists(path):
		shutil.rmtree(path)