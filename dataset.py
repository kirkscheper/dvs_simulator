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
	def __init__(self, path, datasetFolder = 'images'):

		# path to store the variables
		self.path = path
		self.datasetFolder = datasetFolder

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


	def copy_bagfile(self, folderName = None):

		# name of the folder
		if folderName == None: self.folderName = self.bagSplit[0]
		else: self.folderName = folderName

		# check directories for bagfiles
		if not os.path.exists(self.path + '/'):
			os.makedirs(self.path + '/')

		bagDir = self.path + '/' + self.folderName + '/'
		if not os.path.exists(bagDir):
			os.makedirs(bagDir)

		# copy the bagfile to the desired directory
		os.system('cp ' + self.bagFilePath + ' ' + bagDir)

		# rename the file to the name of the folder
		os.system('mv ' + bagDir + self.bagFile + ' ' + bagDir + self.folderName + '.bag')

		# copy the trajectory file
		trajDir = os.getcwd() + '/src/rpg_davis_simulator/datasets/scenes/'
		os.system('cp ' + trajDir + 'customTraj.txt ' + bagDir)
		os.system('mv ' + bagDir + 'customTraj.txt ' + bagDir + 'trajectory.txt')


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


	def generate_images(self, accumTime, folderName = None, imgStart = 0, type = 'double_temporal', expScale = 0.0001, expTime = 'us'):

		# name of the folder
		if folderName == None: self.folderName = self.bagSplit[0]
		else: self.folderName = folderName

		# check directories for image storage
		if not os.path.exists(self.path + '/' + self.datasetFolder + '/'):
			os.makedirs(self.path + '/' + self.datasetFolder + '/')

		imgDir = self.path + '/' + self.datasetFolder + '/' + self.folderName + '/'
		
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# generate the image
		if type == 'normal': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(0)
		elif type == 'accum': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(127)
		elif type == 'temporal':
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(127)

			timeEvents   = np.zeros((128, 128))
			expMatrix    = np.full((128, 128), -expScale)
			refIntensity = np.full((128, 128), 127)
		elif type == 'double_temporal':
			accumEventsON  = np.zeros((128, 128), dtype=np.uint8)
			accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			accumEventsON.fill(0)
			accumEventsOFF.fill(0)

			timeEventsON  = np.zeros((128, 128))
			timeEventsOFF = np.zeros((128, 128))
			expMatrix     = np.full((128, 128), -expScale)

			if not os.path.exists(imgDir + 'ON/'): os.makedirs(imgDir + 'ON/')
			if not os.path.exists(imgDir + 'OFF/'): os.makedirs(imgDir + 'OFF/')
		
		imgCnt = 0
		
		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

				ts = int(e.ts.to_nsec() / 1000.0)
				p  = '1' if e.polarity else '0'

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					if type == 'normal':
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 127

					elif type == 'accum':
						if p == '1':
							accumEvents[e.y, e.x] += 0.1*255
						else:
							accumEvents[e.y, e.x] -= 0.1*255

					elif type == 'temporal':
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif type == 'double_temporal':
						if p == '1':
							if expTime == 'ms': timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts     # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms': timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts     # us
							accumEventsOFF[e.y, e.x] = 255
						
				else:

					# counter
					imgCnt += 1

					# apply temporal decay if needed
					if type == 'temporal':
						if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)     # us
						
						diffTime     = np.subtract(refTime, timeEvents)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						relIntensity = np.subtract(accumEvents, refIntensity)
						accumEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
						accumEvents  = np.asarray(accumEvents, dtype=np.uint8)

					elif type == 'double_temporal':

						# time difference with respect to the event that triggers the image generation
						if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

						# ON events
						diffTime     = np.subtract(refTime, timeEventsON)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						accumEventsON  = np.multiply(accumEventsON, expFactor)
						accumEventsON  = np.asarray(accumEventsON, dtype=np.uint8)

						# OFF events
						diffTime     = np.subtract(refTime, timeEventsOFF)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						accumEventsOFF  = np.multiply(accumEventsOFF, expFactor)
						accumEventsOFF  = np.asarray(accumEventsOFF, dtype=np.uint8)

					# save the image
					if imgCnt > imgStart:

						if type == 'double_temporal':
							img = Image.fromarray(accumEventsON)
							img.save(imgDir + 'ON/' + str(imgCnt - imgStart) + '.png')
							img = Image.fromarray(accumEventsOFF)
							img.save(imgDir + 'OFF/' + str(imgCnt - imgStart) + '.png')
						else:
							img = Image.fromarray(accumEvents)
							img.save(imgDir + str(imgCnt - imgStart) + '.png')

					# reset the image (if needed)
					if type == 'normal': 
						accumEvents = np.zeros((128, 128), dtype=np.uint8)
						accumEvents.fill(0)
					elif type == 'accum':
						accumEvents = np.zeros((128, 128), dtype=np.uint8)
						accumEvents.fill(127)

					# accumulate events in an image
					if type == 'normal':
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 127

					elif type == 'accum':
						if p == '1':
							accumEvents[e.y, e.x] += 0.1*255
						else:
							accumEvents[e.y, e.x] -= 0.1*255

					elif type == 'temporal':
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif type == 'double_temporal':
						if p == '1':
							if expTime == 'ms': timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts     # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms': timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts     # us
							accumEventsOFF[e.y, e.x] = 255

		if type == 'temporal':
			if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

			diffTime     = np.subtract(refTime, timeEvents)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			relIntensity = np.subtract(accumEvents, refIntensity)
			accumEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
			accumEvents  = np.asarray(accumEvents, dtype=np.uint8)

		elif type == 'double_temporal':

			# time difference with respect to the event that triggers the image generation
			if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

			# ON events
			diffTime     = np.subtract(refTime, timeEventsON)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			accumEventsON  = np.multiply(accumEventsON, expFactor)
			accumEventsON  = np.asarray(accumEventsON, dtype=np.uint8)

			# OFF events
			diffTime     = np.subtract(refTime, timeEventsOFF)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			accumEventsOFF  = np.multiply(accumEventsOFF, expFactor)
			accumEventsOFF  = np.asarray(accumEventsOFF, dtype=np.uint8)

		# store the last image
		imgCnt += 1
		if type == 'double_temporal':
			img = Image.fromarray(accumEventsON)
			img.save(imgDir + 'ON/' + str(imgCnt - imgStart) + '.png')
			img = Image.fromarray(accumEventsOFF)
			img.save(imgDir + 'OFF/' + str(imgCnt - imgStart) + '.png')
		else:
			img = Image.fromarray(accumEvents)
			img.save(imgDir + str(imgCnt - imgStart) + '.png')

		bag.close()

		return imgCnt


	def generate_ventralFlow(self, traj, idx):

		# write the txt file
		with open(self.path + '/' + self.folderName + '/ventral_flow.txt', 'w') as text_file:
			for i in xrange(0, idx):
		 		text_file.write("%i %.6f %.6f %.6f\n" % (i+1, -(traj[i+1, 1]-traj[i, 1])*1000/traj[i, 3], -(traj[i+1, 2]-traj[i, 2])*1000/traj[i, 3], 0))


	def generate_images_cnst_variance(self, accumTime, folderName = None, imgStart = 0, type = 'double_temporal', expScale = 0.0001, expTime = 'us'):

		# name of the folder
		if folderName == None: self.folderName = self.bagSplit[0]
		else: self.folderName = folderName

		# check directories for image storage
		if not os.path.exists(self.path + '/' + self.datasetFolder + '/'):
			os.makedirs(self.path + '/' + self.datasetFolder + '/')

		imgDir = self.path + '/' + self.datasetFolder + '/' + self.folderName + '/'
		
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# generate the image
		if type == 'normal': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(0)
		elif type == 'accum': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(127)
		elif type == 'temporal':
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(127)

			timeEvents   = np.zeros((128, 128))
			expMatrix    = np.full((128, 128), -expScale)
			refIntensity = np.full((128, 128), 127)
		elif type == 'double_temporal':
			accumEventsON  = np.zeros((128, 128), dtype=np.uint8)
			accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			accumEventsON.fill(0)
			accumEventsOFF.fill(0)

			timeEventsON  = np.zeros((128, 128))
			timeEventsOFF = np.zeros((128, 128))
			expMatrix     = np.full((128, 128), -expScale)

			if not os.path.exists(imgDir + 'ON/'): os.makedirs(imgDir + 'ON/')
			if not os.path.exists(imgDir + 'OFF/'): os.makedirs(imgDir + 'OFF/')
		
		imgCnt = 0
		
		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

				ts = int(e.ts.to_nsec() / 1000.0)
				p  = '1' if e.polarity else '0'

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					if type == 'normal':
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 127

					elif type == 'accum':
						if p == '1':
							accumEvents[e.y, e.x] += 0.1*255
						else:
							accumEvents[e.y, e.x] -= 0.1*255

					elif type == 'temporal':
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif type == 'double_temporal':
						if p == '1':
							if expTime == 'ms': timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts     # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms': timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts     # us
							accumEventsOFF[e.y, e.x] = 255
						
				else:

					# counter
					imgCnt += 1

					# apply temporal decay if needed
					if type == 'temporal':
						if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)     # us
						
						diffTime     = np.subtract(refTime, timeEvents)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						relIntensity = np.subtract(accumEvents, refIntensity)
						accumEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
						accumEvents  = np.asarray(accumEvents, dtype=np.uint8)

					elif type == 'double_temporal':

						# time difference with respect to the event that triggers the image generation
						if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

						# ON events
						diffTime     = np.subtract(refTime, timeEventsON)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						accumEventsON  = np.multiply(accumEventsON, expFactor)
						accumEventsON  = np.asarray(accumEventsON, dtype=np.uint8)

						# OFF events
						diffTime     = np.subtract(refTime, timeEventsOFF)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						accumEventsOFF  = np.multiply(accumEventsOFF, expFactor)
						accumEventsOFF  = np.asarray(accumEventsOFF, dtype=np.uint8)

					# save the image
					if imgCnt > imgStart:

						if type == 'double_temporal':
							img = Image.fromarray(accumEventsON)
							img.save(imgDir + 'ON/' + str(imgCnt - imgStart) + '.png')
							img = Image.fromarray(accumEventsOFF)
							img.save(imgDir + 'OFF/' + str(imgCnt - imgStart) + '.png')
						else:
							img = Image.fromarray(accumEvents)
							img.save(imgDir + str(imgCnt - imgStart) + '.png')

					# reset the image (if needed)
					if type == 'normal': 
						accumEvents = np.zeros((128, 128), dtype=np.uint8)
						accumEvents.fill(0)
					elif type == 'accum':
						accumEvents = np.zeros((128, 128), dtype=np.uint8)
						accumEvents.fill(127)

					# accumulate events in an image
					if type == 'normal':
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 127

					elif type == 'accum':
						if p == '1':
							accumEvents[e.y, e.x] += 0.1*255
						else:
							accumEvents[e.y, e.x] -= 0.1*255

					elif type == 'temporal':
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif type == 'double_temporal':
						if p == '1':
							if expTime == 'ms': timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts     # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms': timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts     # us
							accumEventsOFF[e.y, e.x] = 255

		if type == 'temporal':
			if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

			diffTime     = np.subtract(refTime, timeEvents)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			relIntensity = np.subtract(accumEvents, refIntensity)
			accumEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
			accumEvents  = np.asarray(accumEvents, dtype=np.uint8)

		elif type == 'double_temporal':

			# time difference with respect to the event that triggers the image generation
			if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

			# ON events
			diffTime     = np.subtract(refTime, timeEventsON)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			accumEventsON  = np.multiply(accumEventsON, expFactor)
			accumEventsON  = np.asarray(accumEventsON, dtype=np.uint8)

			# OFF events
			diffTime     = np.subtract(refTime, timeEventsOFF)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			accumEventsOFF  = np.multiply(accumEventsOFF, expFactor)
			accumEventsOFF  = np.asarray(accumEventsOFF, dtype=np.uint8)

		# store the last image
		imgCnt += 1
		if type == 'double_temporal':
			img = Image.fromarray(accumEventsON)
			img.save(imgDir + 'ON/' + str(imgCnt - imgStart) + '.png')
			img = Image.fromarray(accumEventsOFF)
			img.save(imgDir + 'OFF/' + str(imgCnt - imgStart) + '.png')
		else:
			img = Image.fromarray(accumEvents)
			img.save(imgDir + str(imgCnt - imgStart) + '.png')

		bag.close()

		return imgCnt