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


# render Blender scene and simulate DVS
def run_simulator(texture = 'roadmap'):
	if texture == 'roadmap':
		os.system("roslaunch dvs_simulator_py custom_render.launch")
		os.system("roslaunch dvs_simulator_py custom_simulate.launch")
	elif texture == 'checkerboard':
		os.system("roslaunch dvs_simulator_py checkerboard_render.launch")
		os.system("roslaunch dvs_simulator_py checkerboard_simulate.launch")
	elif texture == 'natural':
		os.system("roslaunch dvs_simulator_py natural_render.launch")
		os.system("roslaunch dvs_simulator_py natural_simulate.launch")
	else:
		print('Unknown Blender texture.')
		sys.exit()


class dataset():


	# constructor
	def __init__(self, path, folderName = None):

		# path to store the variables
		self.path = path

		# name of the folder
		self.folderName = folderName


	# get the most recent bagfile, move it, rename it, and generate ground truth information
	def copy_bagfile(self, trajectory = None, time = None):

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

		# get ventral flow information
		self.generate_ventralFlow(trajectory, time)


	# compute ventral flow information from the trajectory of the camera
	def generate_ventralFlow(self, traj, idx):

		# write the txt file
		with open(self.path + '/' + self.folderName + '/ventral_flow.txt', 'w') as text_file:
			for i in xrange(0, idx):
		 		text_file.write("%i %.6f %.6f %.6f\n" % (i+1, -(traj[i+1, 1]-traj[i, 1])*1000/traj[i, 3], -(traj[i+1, 2]-traj[i, 2])*1000/traj[i, 3], 0))


	# generate .aedat files
	def generate_aedat(self, 
		pathFrom = 'bagfiles', 
		bagFile = '0', 
		datasetFolder = 'images'):

		# bagFile path
		self.bagFilePath = pathFrom + '/' + bagFile + '/' + bagFile + '.bag'

		# name of the folder in the project directory
		self.datasetFolder = datasetFolder

		# check if the final directory exists
		finalDir = self.path + '/' + self.datasetFolder + '/' + bagFile + '/'
		if not os.path.exists(finalDir):
			os.makedirs(finalDir)

	    # open the file and write the headers
		file = open(finalDir + bagFile + ".aedat", "w")
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
		        p = '0' if e.polarity else '1' # TODO: Polarity is INVERTED in .aedat files
		        address = "0" + y + x + p # TODO: Polarity is INVERTED in .aedat files

		        # write the event using big endian format
		        file.write("%s" % struct.pack('>I', int(address, 2)))
		        file.write("%s" % struct.pack('>I', int(ts)))

		bag.close()

		# copy trajectory and ventral flow files
		os.system('cp ' + pathFrom + '/' + bagFile + '/trajectory.txt ' + finalDir)
		os.system('cp ' + pathFrom + '/' + bagFile + '/ventral_flow.txt ' + finalDir)


	# write .csv files
	def generate_csv(self, 
		pathFrom = 'bagfiles', 
		bagFile = '0', 
		datasetFolder = 'images'):

		# bagFile path
		self.bagFilePath = pathFrom + '/' + bagFile + '/' + bagFile + '.bag'

		# name of the folder in the project directory
		self.datasetFolder = datasetFolder

		# check if the final directory exists
		finalDir = self.path + '/' + self.datasetFolder + '/' + bagFile + '/'
		if not os.path.exists(finalDir):
			os.makedirs(finalDir)

		# open the file
		file   = open(finalDir + bagFile + ".csv", "w")
		writer = csv.writer(file, delimiter=',')

		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

		    	ts = int(e.ts.to_nsec() / 1000.0)
		    	writer.writerow([ts, e.x, e.y, '1' if e.polarity else '0'])

		bag.close()

		# copy trajectory and ventral flow files
		os.system('cp ' + pathFrom + '/' + bagFile + '/trajectory.txt ' + finalDir)
		os.system('cp ' + pathFrom + '/' + bagFile + '/ventral_flow.txt ' + finalDir)


	# generate .csv files
	def generate_csv_64(self, 
		pathFrom = 'bagfiles', 
		bagFile = '0', 
		datasetFolder = 'images'):

		# bagFile path
		self.bagFilePath = pathFrom + '/' + bagFile + '/' + bagFile + '.bag'

		# name of the folder in the project directory
		self.datasetFolder = datasetFolder

		# check if the final directory exists
		finalDir = self.path + '/' + self.datasetFolder + '/' + bagFile + '/'
		if not os.path.exists(finalDir):
			os.makedirs(finalDir)

		# open the file
		file   = open(finalDir + bagFile + ".csv", "w")
		writer = csv.writer(file, delimiter=',')

		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

		    	ts = int(e.ts.to_nsec() / 1000.0)
		    	writer.writerow([ts, e.x/2, e.y/2, '1' if e.polarity else '0'])

		bag.close()
		# copy trajectory and ventral flow files
		os.system('cp ' + pathFrom + '/' + bagFile + '/trajectory.txt ' + finalDir)
		os.system('cp ' + pathFrom + '/' + bagFile + '/ventral_flow.txt ' + finalDir)

	def generate_data_set_csv(self, datasetFolder = 'images'):
		
		# get the name of the folders
		folders = [name for name in os.listdir(datasetFolder) if os.path.isdir(datasetFolder+'/'+name)]

		ofile  = open(datasetFolder+'/data_file.csv', "wb")
		writer = csv.writer(ofile, delimiter=',', quoting=csv.QUOTE_ALL)

		for i in xrange(0,len(folders)):
			folderName = folders[i].split('_')
			if folderName[0] == 'train':
				writer.writerow(['train', folders[i]])
			elif folderName[0] == 'val':
				writer.writerow(['val', folders[i]])
			elif folderName[0] == 'test':
				writer.writerow(['test', folders[i]])

	# process the stored bagfiles and generate images in the desired directory
	def generate_images(self, 
		pathFrom = 'bagfiles', 
		bagFile = '0', 
		accumTime = 1000,
		imtype = 'temporal', 
		expScale = 0.0001, 
		expTime = 'us',
		datasetFolder = 'images',
		blur = False, 
		eventRateTh = 0.1,
		imageSkip = 1):

		# bagFile path
		self.bagFilePath = pathFrom + '/' + bagFile + '/' + bagFile + '.bag'
		
		if not os.path.exists(self.bagFilePath):
			print self.bagFilePath + ' empty, skipping.'
			return

		# name of the folder in the project directory
		self.datasetFolder = datasetFolder

		# check if the final directory exists
		imgDir = self.path + '/' + self.datasetFolder + '/' + bagFile + '/'
		if os.path.exists(imgDir):
			print imgDir + ' already exsists, skipping.'
			return
			
		os.makedirs(imgDir)

		# initialize the image
		if imtype == 'normal': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			timeEvents  = np.zeros((128, 128))
			accumEvents.fill(127)

		elif imtype == 'normal_mono': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			timeEvents  = np.zeros((128, 128))
			accumEvents.fill(0)

		elif imtype == 'split_normal': 
			accumEventsON  = np.zeros((128, 128), dtype=np.uint8)
			accumEventsON.fill(0)
			accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			accumEventsOFF.fill(0)
			timeEventsON   = np.zeros((128, 128))
			timeEventsOFF  = np.zeros((128, 128))

			# extra directories
			if not os.path.exists(imgDir + 'ON/'): os.makedirs(imgDir + 'ON/')
			if not os.path.exists(imgDir + 'OFF/'): os.makedirs(imgDir + 'OFF/')

		elif imtype == 'temporal':
			accumEvents  = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(127)
			storeEvents  = np.zeros((128, 128), dtype=np.uint8)
			timeEvents   = np.zeros((128, 128))
			expMatrix    = np.full((128, 128), -expScale)
			refIntensity = np.full((128, 128), 127)

		elif imtype == 'split_temporal':
			accumEventsON  = np.zeros((128, 128), dtype=np.uint8)
			accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			accumEventsON.fill(0)
			accumEventsOFF.fill(0)
			storeEventsON  = np.zeros((128, 128), dtype=np.uint8)
			storeEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			timeEventsON   = np.zeros((128, 128))
			timeEventsOFF  = np.zeros((128, 128))
			expMatrix      = np.full((128, 128), -expScale)

			# extra directories
			if not os.path.exists(imgDir + 'ON/'): os.makedirs(imgDir + 'ON/')
			if not os.path.exists(imgDir + 'OFF/'): os.makedirs(imgDir + 'OFF/')

		elif imtype == 'temp_mono':
			accumEvents  = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(0)
			storeEvents  = np.zeros((128, 128), dtype=np.uint8)
			timeEvents   = np.zeros((128, 128))
			expMatrix    = np.full((128, 128), -expScale)
	
		# images generated
		imgCnt = 0
		
		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

		    # event data
				ts = int(e.ts.to_nsec() / 1000.0)
				p  = '1' if e.polarity else '0'

				# update matrices
				if ts / (1000 * imageSkip) == imgCnt:

					if imtype == 'normal':

						if p == '1': accumEvents[e.y, e.x] = 255
						else:        accumEvents[e.y, e.x] = 0
						timeEvents[e.y, e.x] = ts

					elif imtype == 'normal_mono':

						accumEvents[e.y, e.x] = 255
						timeEvents[e.y, e.x]  = ts

					elif imtype == 'split_normal':

						if p == '1': timeEventsON[e.y, e.x]  = ts
						else:        timeEventsOFF[e.y, e.x] = ts

					elif imtype == 'temporal':

						# the exponential decay can be applied using ms or us scale
						if expTime == 'ms':   timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts       # us
						
						if p == '1': accumEvents[e.y, e.x] = 255
						else:        accumEvents[e.y, e.x] = 0

					elif imtype == 'split_temporal':

						if p == '1':
							if expTime == 'ms':   timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts       # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms':   timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts       # us
							accumEventsOFF[e.y, e.x] = 255

					elif imtype == 'temp_mono':

						# the exponential decay can be applied using ms or us scale
						if expTime == 'ms':   timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts       # us
						accumEvents[e.y, e.x] = 255

				else: # if the event under analysis belongs to a different image...

					# counter
					imgCnt += 1
					filename = str(imgCnt) + '.png'

					# generate images
					if imtype == 'normal':

						# check which events have to be included in the image
						refTime  = np.full((128, 128), ts)
						diffTime = np.subtract(refTime, timeEvents)

						# update image
						accumEvents[diffTime > accumTime] = 127

						# save the image
						img = Image.fromarray(accumEvents)
						img.save(imgDir + filename)

						# include this event
						if p == '1': accumEvents[e.y, e.x] = 255
						else:        accumEvents[e.y, e.x] = 0
						timeEvents[e.y, e.x] = ts

					elif imtype == 'normal_mono':

						# check which events have to be included in the image
						refTime  = np.full((128, 128), ts)
						diffTime = np.subtract(refTime, timeEvents)

						# update image
						accumEvents[diffTime > accumTime] = 0

						# save the image
						img = Image.fromarray(accumEvents)
						img.save(imgDir + filename)

						# include this event
						accumEvents[e.y, e.x] = 255
						timeEvents[e.y, e.x]  = ts

					elif imtype == 'split_normal':

						# check which events have to be included in the image
						refTime     = np.full((128, 128), ts)
						diffTimeON  = np.subtract(refTime, timeEventsON)
						diffTimeOFF = np.subtract(refTime, timeEventsOFF)

						accumEventsON.fill(0)
						accumEventsOFF.fill(0)
						accumEventsON[diffTimeON <= accumTime]   = 255
						accumEventsOFF[diffTimeOFF <= accumTime] = 255

						# save the image
						img = Image.fromarray(accumEventsON)
						img.save(imgDir + 'ON/' + filename)
						img = Image.fromarray(accumEventsOFF)
						img.save(imgDir + 'OFF/' + filename)

						# include this event
						if p == '1': timeEventsON[e.y, e.x]  = ts
						else:        timeEventsOFF[e.y, e.x] = ts

					elif imtype == 'temporal':

						# reference time
						if expTime == 'ms': refTime   = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)       # us
						
						# exponential decay of the image
						diffTime     = np.subtract(refTime, timeEvents)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						relIntensity = np.subtract(accumEvents, refIntensity)
						storeEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
						storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

						# save the image
						img = Image.fromarray(storeEvents)
						img.save(imgDir + filename)

						# include this event
						if expTime == 'ms':   timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts       # us
						if p == '1': accumEvents[e.y, e.x] = 255
						else:        accumEvents[e.y, e.x] = 0

					elif imtype == 'split_temporal':

						# reference time
						if expTime == 'ms':   refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)       # us

						# ON events
						diffTime      = np.subtract(refTime, timeEventsON)
						expFactor     = np.exp(np.multiply(expMatrix, diffTime))
						storeEventsON = np.multiply(accumEventsON, expFactor)
						storeEventsON = np.asarray(storeEventsON, dtype=np.uint8)

						# OFF events
						diffTime       = np.subtract(refTime, timeEventsOFF)
						expFactor      = np.exp(np.multiply(expMatrix, diffTime))
						storeEventsOFF = np.multiply(accumEventsOFF, expFactor)
						storeEventsOFF = np.asarray(storeEventsOFF, dtype=np.uint8)

						# save the image
						img = Image.fromarray(storeEventsON)
						img.save(imgDir + 'ON/' + filename)
						img = Image.fromarray(storeEventsOFF)
						img.save(imgDir + 'OFF/' + filename)

						# include this event
						if p == '1':
							if expTime == 'ms':   timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts       # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms':   timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts       # us
							accumEventsOFF[e.y, e.x] = 255

					elif imtype == 'temp_mono':

						# reference time
						if expTime == 'ms':   refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)       # us

						# exponential decay of the image
						diffTime     = np.subtract(refTime, timeEvents)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						storeEvents  = np.multiply(accumEvents, expFactor)
						storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

						# save the image
						img = Image.fromarray(storeEvents)
						img.save(imgDir + filename)

						# include this event
						if expTime == 'ms':   timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts       # us
						accumEvents[e.y, e.x] = 255

		# store final image
		imgCnt += 1
		filename = str(imgCnt) + '.png'

		if imtype == 'normal':

			# check which events have to be included in the image
			refTime  = np.full((128, 128), ts)
			diffTime = np.subtract(refTime, timeEvents)

			# update image
			accumEvents[diffTime > accumTime] = 127

			# save the image
			img = Image.fromarray(accumEvents)
			img.save(imgDir + filename)

		elif imtype == 'normal_mono':

			# check which events have to be included in the image
			refTime  = np.full((128, 128), ts)
			diffTime = np.subtract(refTime, timeEvents)

			# update image
			accumEvents[diffTime > accumTime] = 0

			# save the image
			img = Image.fromarray(accumEvents)
			img.save(imgDir + filename)

		elif imtype == 'split_normal':

			# check which events have to be included in the image
			refTime     = np.full((128, 128), ts)
			diffTimeON  = np.subtract(refTime, timeEventsON)
			diffTimeOFF = np.subtract(refTime, timeEventsOFF)

			accumEventsON.fill(0)
			accumEventsOFF.fill(0)
			accumEventsON[diffTimeON <= accumTime]   = 255
			accumEventsOFF[diffTimeOFF <= accumTime] = 255

			# save the image
			img = Image.fromarray(accumEventsON)
			img.save(imgDir + 'ON/' + filename)
			img = Image.fromarray(accumEventsOFF)
			img.save(imgDir + 'OFF/' + filename)

		elif imtype == 'temporal':

			# reference time
			if expTime == 'ms': refTime   = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)       # us
			
			# exponential decay of the image
			diffTime     = np.subtract(refTime, timeEvents)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			relIntensity = np.subtract(accumEvents, refIntensity)
			storeEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
			storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

			# save the image
			img = Image.fromarray(storeEvents)
			img.save(imgDir + filename)

		elif imtype == 'split_temporal':

			# reference time
			if expTime == 'ms':   refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)       # us

			# ON events
			diffTime      = np.subtract(refTime, timeEventsON)
			expFactor     = np.exp(np.multiply(expMatrix, diffTime))
			storeEventsON = np.multiply(accumEventsON, expFactor)
			storeEventsON = np.asarray(storeEventsON, dtype=np.uint8)

			# OFF events
			diffTime       = np.subtract(refTime, timeEventsOFF)
			expFactor      = np.exp(np.multiply(expMatrix, diffTime))
			storeEventsOFF = np.multiply(accumEventsOFF, expFactor)
			storeEventsOFF = np.asarray(storeEventsOFF, dtype=np.uint8)

			# save the image
			img = Image.fromarray(storeEventsON)
			img.save(imgDir + 'ON/' + filename)
			img = Image.fromarray(storeEventsOFF)
			img.save(imgDir + 'OFF/' + filename)

		elif imtype == 'temp_mono':

			# reference time
			if expTime == 'ms':   refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)       # us

			# exponential decay of the image
			diffTime     = np.subtract(refTime, timeEvents)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			storeEvents  = np.multiply(accumEvents, expFactor)
			storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

			# save the image
			img = Image.fromarray(storeEvents)
			img.save(imgDir + filename)

		# close the bagfile
		bag.close()
		
		if imageSkip == 1:
			# copy trajectory and ventral flow files
			os.system('cp ' + pathFrom + '/' + bagFile + '/trajectory.txt ' + imgDir)
			os.system('cp ' + pathFrom + '/' + bagFile + '/ventral_flow.txt ' + imgDir)
		else:
			ifile  = open(pathFrom + '/' + bagFile + '/trajectory.txt', "r")
			reader = csv.reader(ifile, delimiter=' ')
			ofile  = open(imgDir+'/trajectory.txt', "wb")
			writer = csv.writer(ofile, delimiter=' ', quoting=csv.QUOTE_NONE)
			
			i = 0
			for row in reader:
				if i % imageSkip == 0:
					row[0] = i / imageSkip + 1
					writer.writerow(row)
				i = i + 1
					
			ifile  = open(pathFrom + '/' + bagFile + '/ventral_flow.txt', "r")
			reader = csv.reader(ifile, delimiter=' ')
			ofile  = open(imgDir+'/ventral_flow.txt', "wb")
			writer = csv.writer(ofile, delimiter=' ', quoting=csv.QUOTE_NONE)
			
			i = 0
			for row in reader:
				if i % imageSkip == 0:
					row[0] = i / imageSkip + 1
					writer.writerow(row)
				i = i + 1
		
	# process the stored bagfiles and generate images in the desired directory
	def generate_images_cnst_variance(self, 
		pathFrom = 'bagfiles', 
		bagFile = '0',
		datasetFolder = 'images',
		variance_obj = 0.05):

		# bagFile path
		self.bagFilePath = pathFrom + '/' + bagFile + '/' + bagFile + '.bag'

		# name of the folder in the project directory
		self.datasetFolder = datasetFolder

		# check if the final directory exists
		imgDir = self.path + '/' + self.datasetFolder + '/' + bagFile + '/'

		# continue
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# open file
		ofile = open(imgDir + 'delta_t.csv', "wb")
		writer = csv.writer(ofile, delimiter=',')

		# initialize arrays
		accumEventsON = np.zeros((128, 128), dtype=np.uint8)
		accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
		accumEventsON.fill(0) # black canvas
		accumEventsOFF.fill(0) # black canvas
		timeEventsON  = np.zeros((128, 128))
		timeEventsOFF = np.zeros((128, 128))

		# extra directories
		if not os.path.exists(imgDir + 'ON/'): os.makedirs(imgDir + 'ON/')
		if not os.path.exists(imgDir + 'OFF/'): os.makedirs(imgDir + 'OFF/')
		
		# images generated
		imgCnt = 0
		
		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

		    	# event data
				ts = int(e.ts.to_nsec() / 1000.0)
				p  = '1' if e.polarity else '0'

		        # accumulate events in an image
				if ts / 1000 == imgCnt:
					if p == '1': timeEventsON[e.y, e.x] = ts
					else: timeEventsOFF[e.y, e.x] = ts
				
				else:

					# counter
					imgCnt += 1

					# check which events have to be included in the image
					refTime     = np.full((128, 128), ts)
					diffTimeON  = np.subtract(refTime, timeEventsON)
					diffTimeOFF = np.subtract(refTime, timeEventsOFF)

					start = 0
					end   = ts
					if ts > 30000: end = 30000
					while True:

						# get the point
						accumTime  = start + end
						accumTime /= 2

						# reset arrays
						accumEventsON.fill(0)
						accumEventsON[diffTimeON <= accumTime] = 255

						ION  = np.asarray(accumEventsON, dtype=np.float32)
						ION /= 255.0

						# evaluate the variance
						varON = np.var(ION)

						if varON >= variance_obj - 0.01 and varON <= variance_obj + 0.01:
							break
						elif abs(start - end) <= 1:
							break 
						elif varON > variance_obj + 0.01:
							end = accumTime
						elif varON < variance_obj - 0.01:
							start = accumTime

					# reset arrays
					accumEventsON.fill(0)
					accumEventsOFF.fill(0)
					accumEventsON[diffTimeON <= accumTime] = 255
					accumEventsOFF[diffTimeOFF <= accumTime] = 255

					ION   = np.asarray(accumEventsON, dtype=np.float32)
					IOFF  = np.asarray(accumEventsOFF, dtype=np.float32)
					ION  /= 255.0
					IOFF /= 255.0

					# evaluate the variance
					varON  = np.var(ION)
					varOFF = np.var(IOFF)

					# save the image
					img = Image.fromarray(accumEventsON)
					img.save(imgDir + 'ON/' + str(imgCnt) + '.png')
					img = Image.fromarray(accumEventsOFF)
					img.save(imgDir + 'OFF/' + str(imgCnt) + '.png')
					writer.writerow([imgCnt, accumTime, varON, varOFF])

					# accumulate events in an image
					if ts / 1000 == imgCnt:
						if p == '1':
							timeEventsON[e.y, e.x] = ts
						else:
							timeEventsOFF[e.y, e.x] = ts

		# counter
		imgCnt += 1

		# check which events have to be included in the image
		refTime     = np.full((128, 128), ts)
		diffTimeON  = np.subtract(refTime, timeEventsON)
		diffTimeOFF = np.subtract(refTime, timeEventsON)

		start = 0
		end   = ts
		if ts > 30000: end = 30000
		while True:

			# get the point
			accumTime  = start + end
			accumTime /= 2

			# reset arrays
			accumEventsON.fill(0)
			accumEventsON[diffTimeON <= accumTime] = 255

			ION  = np.asarray(accumEventsON, dtype=np.float32)
			ION /= 255.0

			# evaluate the variance
			varON = np.var(ION)

			if varON >= variance_obj - 0.01 and varON <= variance_obj + 0.01:
				break
			elif abs(start - end) <= 1:
				break 
			elif varON > variance_obj + 0.01:
				end = accumTime
			elif varON < variance_obj - 0.01:
				start = accumTime

		# reset arrays
		accumEventsON.fill(0)
		accumEventsOFF.fill(0)
		accumEventsON[diffTimeON <= accumTime] = 255
		accumEventsOFF[diffTimeOFF <= accumTime] = 255

		ION   = np.asarray(accumEventsON, dtype=np.float32)
		IOFF  = np.asarray(accumEventsOFF, dtype=np.float32)
		ION  /= 255.0
		IOFF /= 255.0

		# evaluate the variance
		varON  = np.var(ION)
		varOFF = np.var(IOFF)

		# save the image
		img = Image.fromarray(accumEventsON)
		img.save(imgDir + 'ON/' + str(imgCnt) + '.png')
		img = Image.fromarray(accumEventsOFF)
		img.save(imgDir + 'OFF/' + str(imgCnt) + '.png')
		writer.writerow([imgCnt, accumTime, varON, varOFF])

		# close the bagfile
		bag.close()
		ofile.close()

		# copy trajectory and ventral flow files
		os.system('cp ' + pathFrom + '/' + bagFile + '/ventral_flow.txt ' + imgDir)
