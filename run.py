#!/usr/bin/env python

""" 
run.py is a Python script that generates a trajectory based on a set of 
straight segments for the rpg_davis_simulator [1], rederizes the scene 
using Blender, generates the DVS events, and converts the .rosbag output 
file into a 2.0 .aedat file [2] that can be processed by the software jAER [3].
Furthermore, it can store .png images with the accumulated events (polarity
not included) in a specified time window. Ground truth for these images is also
provided.

NOTE: Only works when rpg_davis_simulator [1] has been successfully installed.

[1] rpg_davis_simulator: https://github.com/uzh-rpg/rpg_davis_simulator
[2] AEDAT file formats: https://inilabs.com/support/software/fileformat/
[3] jAER: https://sourceforge.net/p/jaer/wiki/Home/

Author: F. Paredes Valles
Created: 04-05-2017
"""

import os
import glob
import rosbag
import struct
import numpy as np
from PIL import Image

# divide the trajectory in a set of straight segments
while True:
	entry = raw_input("\nIn how many segments do you want to divide the trajectory? [1,10] (integer) ")
	if int(entry) >= 1 and int(entry) <= 10:
		numSegments = int(entry)
		break
	else:
		print "Try again."

# allocate memory for the trajectory array
traj 		= np.zeros((int(numSegments * 50000), 8))
ventralFlow = np.zeros((int(numSegments * 50000), 4))

print "\n--------------------------------------\n TRAJECTORY GENERATION"

# starting point of the trajectory
traj[0,:] 		 = [0, 0.000000, 0.000000, 0.500000, 1.000000, 0.000000, 0.000000, 0.000000]
while True:
	entry = raw_input("\n[ORIGIN] -> The origin is at [0, 0, 0.5], would you like to change it? [y/n] ")
	if entry == 'y' or entry == 'Y':

			while True:
				entry = raw_input("[ORIGIN] -> x: [-2.0,2.0] ")

				if float(entry) >= -2.0 and float(entry) <= 2.0:
					traj[0, 1] = float(entry)

					while True:
						entry = raw_input("[ORIGIN] -> y: [-1.0,1.0] ")

						if float(entry) >= -1.0 and float(entry) <= 1.0:
							traj[0, 2] = float(entry)

							while True:
								entry = raw_input("[ORIGIN] -> z: [0.5,2.0] ")

								if float(entry) >= 0.5 and float(entry) <= 2.0:
									traj[0, 3] = float(entry)									
									break
								else:
									print "Try again."
							break
						else:
							print "Try again."
					break
				else:
					print "Try again."
			break
	elif entry == 'n' or entry == 'N':
		break
	else:
		print "Try again."

# generate the desired trajectory
cntRow = 0
for sgm in xrange(0, numSegments):
	while True:
		print("\n[SEGMENT " + str(sgm + 1) + "] -> From: [" + str(traj[cntRow, 1]) + ", " + str(traj[cntRow, 2]) + ", " + str(traj[cntRow, 3]) + "], To: [x, y, z]")
		entry = raw_input("[SEGMENT " + str(sgm + 1) + "] -> x: [-3.0,3.0] ")
		if float(entry) >= -3.0 and float(entry) <= 3.0:
			x_ = float(entry)
			while True:
				entry = raw_input("[SEGMENT " + str(sgm + 1) + "] -> y: [-1.5,1.5] ")
				if float(entry) >= -1.5 and float(entry) <= 1.5:
					y_ = float(entry)
					while True:
						entry = raw_input("[SEGMENT " + str(sgm + 1) + "] -> z: [0.5,2.5] ")
						if float(entry) >= 0.5 and float(entry) <= 2.0:
							z_ = float(entry)
							while True:
								entry = raw_input("[SEGMENT " + str(sgm + 1) + "] -> t: [0.1,5.0] ")
								if float(entry) >= 0.1 and float(entry) <= 5.0:
									t_ = int(float(entry) * 1000)

									# compute the trajectory for this segment
									deltaX = (x_ - traj[cntRow, 1]) / t_
									deltaY = (y_ - traj[cntRow, 2]) / t_
									deltaZ = (z_ - traj[cntRow, 3]) / t_
									
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

								 		# store groundtruth information
								 		if i == 1:
								 			ventralFlow[i-1,0] = 1
							 			else:
							 				ventralFlow[i-1,0] = ventralFlow[i-2,0] + 1
								 		ventralFlow[i-1,1] = deltaX * 1000 / traj[i,3] # vx
								 		ventralFlow[i-1,2] = deltaY * 1000 / traj[i,3] # vy
								 		ventralFlow[i-1,3] = deltaZ * 1000 / traj[i,3] # vz

								 	cntRow += t_
									break
								else:
									print "Try again."
							break
						else:
							print "Try again."
					break
				else:
					print "Try again."
			break
		else:
			print "Try again."

# save the file where it should be
with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
	for i in xrange(0,cntRow):
 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(traj[i, 0]), traj[i, 1], traj[i, 2], traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6], traj[i, 7]))

print "\n--------------------------------------\n SCENE RENDERING AND SIMULATION"

# make sure that the .bash file is in the source of ubuntu
os.system("source " + os.getcwd() + "/devel/setup.bash")

# render the scene
os.system("roslaunch dvs_simulator_py custom_render.launch")

# simulate the scene
os.system("roslaunch dvs_simulator_py custom_simulate.launch")

print "\n--------------------------------------\n GENERATE .AEDAT AND IMAGES"

# store last millisecond of the simulation
lastTime = traj[cntRow - 1,0]

# check if the user wants to accumulate events
while True:
	entry = raw_input("\nDo you want to record images with accumulated events? [y/n] ")
	if entry == 'y' or entry == 'Y':
		accumFlag = True
		while True:
			entry = raw_input("Enter the time window for the accumulation: [150, " + str(int(lastTime*50)) + "] (us) ")
			if int(entry) < 150 or int(entry) > lastTime*50:
				print "Try again."
			else:
				accumTime = int(entry)
				break
		break
	elif entry == 'n' or entry == 'N':
		accumFlag = False
		break
	else:
		print "Try again."

# get the current rosbag file
curDir = os.getcwd()
os.chdir(os.getcwd() + '/src/rpg_davis_simulator/datasets/rosbags/')
bagFile = max(glob.glob('*.bag'), key=os.path.getctime)
bagFilePath = os.getcwd() + '/' + bagFile
os.chdir(curDir)

# split the name and create the aedat filename
bagSplit = bagFile.split('.')
aedatFile = bagSplit[0] + '.aedat'

# check directories for image storage
if accumFlag:
	if not os.path.exists('aedat/images/'):
		os.makedirs('aedat/images/')
	imgDir = 'aedat/images/' + bagSplit[0] + '_' + str(accumTime) + 'us/'
	accumEvents = np.zeros((128, 128), dtype=np.uint8)
	accumEvents[0, 0] = 0
	imgCnt = 0
	if not os.path.exists(imgDir):
		os.makedirs(imgDir)

print "\nFormatting: .rosbag -> .aedat (This should take a couple of minutes)"

# check for the aedat directory
if not os.path.exists('aedat/'):
    os.makedirs('aedat/')

# open the file and write the headers
file = open("aedat/" + aedatFile, "w")
file.write('#!AER-DAT2.0\r\n')
file.write('# This is a raw AE data file created by saveaerdat.m\r\n');
file.write('# Data format is int32 address, int32 timestamp (8 bytes total), repeated for each event\r\n');
file.write('# Timestamps tick is 1 us\r\n');

# open the rosbag file and process the events
bag = rosbag.Bag(bagFilePath)
for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
    for e in msg.events:

        ts = int(e.ts.to_nsec() / 1000.0)
        x = '{0:07b}'.format(e.x)
        y = '{0:07b}'.format(e.y)
        p = '1' if e.polarity else '0'
        address = "0" + y + x + p

        # write the event using big endian format
        file.write("%s" % struct.pack('>I', int(address, 2)))
        file.write("%s" % struct.pack('>I', int(ts)))

        # accumulate events in an image
        if accumFlag:
			if ts / accumTime == imgCnt:
				if accumEvents[-e.y, -e.x] == 0:
					accumEvents[-e.y, -e.x] = 255
			else:
				imgCnt += 1
				if imgCnt >= 10:
					img = Image.fromarray(accumEvents)
					img.save(imgDir + bagSplit[0] + '_' + str(accumTime) + 'us' + '_' + str(imgCnt) + '.png')
				accumEvents = np.zeros((128, 128), dtype=np.uint8)
				accumEvents[0, 0] = 0
				accumEvents[-e.y, -e.x] = 255

# store the last image
if accumFlag:
	imgCnt += 1
	img = Image.fromarray(accumEvents)
	img.save(imgDir + bagSplit[0] + '_' + str(accumTime) + 'us' + '_' + str(imgCnt) + '.png')

# check if the user wants to save images using different time windows
if accumFlag:
	accumTimeVector = [accumTime]
	while True:
		entry = raw_input("\nDo you want to record images with a different time window? [y/n] ")
		if entry == 'y' or entry == 'Y':
			while True:
				entry = raw_input("Enter the time window for the accumulation: [150, " + str(int(lastTime*50)) + "] (us) ")
				if int(entry) < 150 or int(entry) > lastTime*50:
					print "Try again."
				elif int(entry) in accumTimeVector:
					print "Try again. Same time window as before."
				else:
					accumTime = int(entry)
					accumTimeVector.append(accumTime)
					imgDir = 'aedat/images/' + bagSplit[0] + '_' + str(accumTime) + 'us/'
					accumEvents = np.zeros((128, 128), dtype=np.uint8)
					accumEvents[0, 0] = 0
					imgCnt = 0
					if not os.path.exists(imgDir):
						os.makedirs(imgDir)

					for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
					    for e in msg.events:

							ts = int(e.ts.to_nsec() / 1000.0)
							x = '{0:07b}'.format(e.x)
							y = '{0:07b}'.format(e.y)

							# accumulate events in an image
							if ts / accumTime == imgCnt:
								if accumEvents[-e.y, -e.x] == 0:
									accumEvents[-e.y, -e.x] = 255
							else:
								imgCnt += 1
								if imgCnt >= 10:
									img = Image.fromarray(accumEvents)
									img.save(imgDir + bagSplit[0] + '_' + str(accumTime) + 'us' + '_' + str(imgCnt) + '.png')
								accumEvents = np.zeros((128, 128), dtype=np.uint8)
								accumEvents[0, 0] = 0
								accumEvents[-e.y, -e.x] = 255

					# store the last image
					imgCnt += 1
					img = Image.fromarray(accumEvents)
					img.save(imgDir + bagSplit[0] + '_' + str(accumTime) + 'us' + '_' + str(imgCnt) + '.png')

					break
		elif entry == 'n' or entry == 'N':
			break
		else:
			print "Try again."

bag.close()

# check for the GT directory inside aedat
if not os.path.exists('aedat/GT/'):
    os.makedirs('aedat/GT/')

# save the file where it should be
GTFile = bagSplit[0] + '.txt'
with open("aedat/GT/" + GTFile, "w") as text_file:
	for i in xrange(0,cntRow - 1):
 		text_file.write("%i %.6f %.6f %.6f\n" % (int(ventralFlow[i, 0]), ventralFlow[i, 1], ventralFlow[i, 2], - ventralFlow[i, 3]))

# generate the ground truth for the set of images generated
# if accumFlag:
# 	for accumTime in accumTimeVector:
# 		pass