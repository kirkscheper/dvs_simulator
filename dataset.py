import os
import sys
import csv
import glob
import rosbag
import struct
import numpy as np
from math import fabs
from math import floor
from PIL import Image


def run_simulator():
	os.system("roslaunch dvs_simulator_py custom_render.launch")
	os.system("roslaunch dvs_simulator_py custom_simulate.launch")


class dataset():

	# constructor
	def __init__(self, path):

		# path to store the variables
		self.path = path

		# get the current rosbag file
		curDir = os.getcwd()
		nxtDir = os.getcwd() + '/src/rpg_davis_simulator/datasets/rosbags/'
		if not os.path.exists(nxtDir): os.makedirs(nxtDir)
		os.chdir(nxtDir)
		self.bagFile = max(glob.glob('*.bag'), key=os.path.getctime)
		self.bagFilePath = os.getcwd() + '/' + self.bagFile
		os.chdir(curDir)

		# split the name and create the aedat filename
		self.bagSplit = self.bagFile.split('.')
		self.aedatFile = self.bagSplit[0] + '.aedat'
		

	def generate_aedat(self):

		# check for the aedat directory
		if not os.path.exists(self.path + '/aedat/'):
		    os.makedirs(self.path + '/aedat/')

	    # open the file and write the headers
		file = open("aedat/" + self.aedatFile, "w")
		file.write('#!AER-DAT2.0\r\n')
		file.write('# This is a raw AE data file created by saveaerdat.m\r\n');
		file.write('# Data format is int32 address, int32 timestamp (8 bytes total), repeated for each event\r\n');
		file.write('# Timestamps tick is 1 us\r\n');

		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

		        ts = int(e.ts.to_nsec() / 1000.0)
		        x = '{0:07b}'.format(128-e.x)
		        y = '{0:07b}'.format(128-e.y)
		        p = '1' if e.polarity else '0'
		        address = "0" + y + x + p # TODO: Polarity is INVERTED in .aedat files

		        # write the event using big endian format
		        file.write("%s" % struct.pack('>I', int(address, 2)))
		        file.write("%s" % struct.pack('>I', int(ts)))

		bag.close()


	def generate_images(self, accumTime, folderName = None):

		# name of the folder
		if folderName == None: self.folderName = self.bagSplit[0]
		else: self.folderName = folderName

		# check directories for image storage
		if not os.path.exists(self.path + '/images/'):
			os.makedirs(self.path + '/images/')

		imgDir = self.path + '/images/' + self.folderName + '/'
		
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# generate the image
		accumEvents = np.zeros((128, 128), dtype=np.uint8)
		accumEvents[0, 0] = 0
		imgCnt = 0
		
		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

				ts = int(e.ts.to_nsec() / 1000.0)
				p  = '1' if e.polarity else '0'

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					if p == '1':
						accumEvents[e.y, e.x] = 255 # bright
					else:
						accumEvents[e.y, e.x] = 127 # not that bright
						
				else:

					# counter
					imgCnt += 1

					# save the image
					img = Image.fromarray(accumEvents)
					img.save(imgDir + str(imgCnt) + '.png')

					# reset the image
					accumEvents = np.zeros((128, 128), dtype=np.uint8)
					if p == '1':
						accumEvents[e.y, e.x] = 255 # bright
					else:
						accumEvents[e.y, e.x] = 127 # not that bright

		# store the last image
		imgCnt += 1
		img = Image.fromarray(accumEvents)
		img.save(imgDir + str(imgCnt) + '.png')

		bag.close()

		return imgCnt


	def generate_groundtruth(self, tProc, xProc, yProc, altitude, imgCnt):

		idx = []
		wx  = []
		wy  = []
		D   = []
		for i in xrange(0, imgCnt):
			idx.append(i+1)
			wx.append(-(yProc[i+1]-yProc[i])*1000/altitude)
			wy.append(-(xProc[i+1]-xProc[i])*1000/altitude)
			D.append(0) # TODO

		# write the txt file
		with open(self.path + '/images/' + self.folderName + '/GT_flow.txt', 'w') as text_file:
			for i in xrange(0,len(idx)):
		 		text_file.write("%i %.6f %.6f %.6f\n" % (idx[i], wx[i], wy[i], D[i]))
