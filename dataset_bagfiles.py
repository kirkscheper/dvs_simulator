import os
import cv2
import sys
import csv
import glob
import rosbag
import struct
import numpy as np
from math import fabs
from math import floor
from PIL import Image


def run_simulator(texture = 'roadmap'):
	if texture == 'roadmap':
		os.system("roslaunch dvs_simulator_py custom_render.launch")
		os.system("roslaunch dvs_simulator_py custom_simulate.launch")
	elif texture == 'checkerboard':
		os.system("roslaunch dvs_simulator_py checkerboard_render.launch")
		os.system("roslaunch dvs_simulator_py checkerboard_simulate.launch")
	else:
		print('Unknown Blender texture.')
		sys.exit()


class dataset():

	# constructor
	def __init__(self, path, folderName = None):

		# path to store the variables
		self.path = path

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

		# get ventral flow information
		self.generate_ventralFlow(trajectory, time)


	# dataset statistics
	def data_statistics(self, 
		pathFrom = '/media/fedepare/Datos/Ubuntu/Projects/bagfiles', 
		bagFile = '0', 
		eventRateTh = 0.01):

		# bagFile path
		self.bagFilePath = pathFrom + '/' + bagFile + '/' + bagFile + '.bag'

		# counter of events
		self.eventRate = []

		# counters
		cnt = 0
		self.times = None
		self.cnt   = None
		self.Flag  = False
		self.rate  = []

		# open the rosbag file and process the events
		bag = rosbag.Bag(self.bagFilePath)
		for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
		    for e in msg.events:

		    	# event timestep
				ts = int(e.ts.to_nsec() / 1000.0)

				# initialize the vector
				if self.times == None:
					self.times = [ts]
					self.cnt   = [1]
				elif self.times[cnt] != ts:

					sys.stdout.write('\r' + '{0}\r'.format(ts/10.0**6,))
					sys.stdout.flush()

					cnt += 1
					self.times.append(ts)
					self.cnt.append(1)
					self.Flag = True
				else:
					self.cnt[cnt] += 1

				if self.Flag == True:
					self.Flag = False
					counter = 0
					for i in range(len(self.times)-1,-1,-1):
						if self.times[-1] - self.times[i] <= eventRateTh*10**6:
							counter += self.cnt[i]
						else:
							break
					self.rate.append(counter)
		    
		# close the bagfile
		bag.close()

		# write results
		with open('/media/fedepare/Datos/Ubuntu/Projects/bagfiles/circle_rate.txt', 'w') as text_file:
			for i in xrange(0, len(self.rate)):
		 		text_file.write("%.6f\n" % (self.rate[i]))

		return self.rate


	# compute ventral flow information from the trajectory of the camera
	def generate_ventralFlow(self, traj, idx):

		# write the txt file
		with open(self.path + '/' + self.folderName + '/ventral_flow.txt', 'w') as text_file:
			for i in xrange(0, idx):
		 		text_file.write("%i %.6f %.6f %.6f\n" % (i+1, -(traj[i+1, 1]-traj[i, 1])*1000/traj[i, 3], -(traj[i+1, 2]-traj[i, 2])*1000/traj[i, 3], 0))


	# process the stored bagfiles and generate images in the desired directory
	def generate_images(self, 
		pathFrom = '/media/fedepare/Datos/Ubuntu/Projects/bagfiles', 
		bagFile = '0', 
		accumTime = 1000, 
		imtype = 'temporal', 
		expScale = 0.0001, 
		expTime = 'us',
		datasetFolder = 'images',
		blur = False, 
		eventRateTh = 0.1):

		# bagFile path
		self.bagFilePath = pathFrom + '/' + bagFile + '/' + bagFile + '.bag'

		# name of the folder in the project directory
		self.datasetFolder = datasetFolder

		# check if the final directory exists
		imgDir = self.path + '/' + self.datasetFolder + '/' + bagFile + '/'
		if not os.path.exists(imgDir):
			os.makedirs(imgDir)

		# initialize the image
		if imtype == 'normal': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(127) # gray canvas

		elif imtype == 'normal_mono': 
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(0) # black canvas

		elif imtype == 'split_normal': 
			accumEventsON = np.zeros((128, 128), dtype=np.uint8)
			accumEventsON.fill(0) # black canvas
			accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			accumEventsOFF.fill(0) # black canvas

			# extra directories
			if not os.path.exists(imgDir + 'ON/'): os.makedirs(imgDir + 'ON/')
			if not os.path.exists(imgDir + 'OFF/'): os.makedirs(imgDir + 'OFF/')

		elif imtype == 'temporal':
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(127) # gray canvas
			storeEvents = np.zeros((128, 128), dtype=np.uint8)
			timeEvents   = np.zeros((128, 128))
			expMatrix    = np.full((128, 128), -expScale)
			refIntensity = np.full((128, 128), 127)

		elif imtype == 'split_temporal':
			accumEventsON  = np.zeros((128, 128), dtype=np.uint8)
			accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			accumEventsON.fill(0)  # black canvas
			accumEventsOFF.fill(0) # black canvas
			storeEventsON  = np.zeros((128, 128), dtype=np.uint8)
			storeEventsOFF = np.zeros((128, 128), dtype=np.uint8)
			timeEventsON  = np.zeros((128, 128))
			timeEventsOFF = np.zeros((128, 128))
			expMatrix     = np.full((128, 128), -expScale)

			# extra directories
			if not os.path.exists(imgDir + 'ON/'): os.makedirs(imgDir + 'ON/')
			if not os.path.exists(imgDir + 'OFF/'): os.makedirs(imgDir + 'OFF/')

		elif imtype == 'temp_mono':
			accumEvents = np.zeros((128, 128), dtype=np.uint8)
			accumEvents.fill(0) # gray canvas
			storeEvents = np.zeros((128, 128), dtype=np.uint8)
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

		        # accumulate events in an image
				if ts / accumTime == imgCnt:
					if imtype == 'normal':
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif imtype == 'normal_mono':
						accumEvents[e.y, e.x] = 255

					elif imtype == 'split_normal':
						if p == '1':
							accumEventsON[e.y, e.x] = 255
						else:
							accumEventsOFF[e.y, e.x] = 255

					elif imtype == 'temporal':

						# the exponential decay can be applied using ms or us scale
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif imtype == 'split_temporal':
						if p == '1':
							if expTime == 'ms': timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts     # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms': timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts     # us
							accumEventsOFF[e.y, e.x] = 255

					elif imtype == 'temp_mono':

						# the exponential decay can be applied using ms or us scale
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						accumEvents[e.y, e.x] = 255

				
				else: # if the event under analysis belongs to a different image...

					# counter
					imgCnt += 1

					# apply temporal decay if needed
					if imtype == 'temporal':
						if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)     # us
						
						diffTime     = np.subtract(refTime, timeEvents)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						relIntensity = np.subtract(accumEvents, refIntensity)
						storeEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
						storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

						# save the image
						img = Image.fromarray(storeEvents)
						img.save(imgDir + str(imgCnt) + '.png')

						# blur the image?
						if blur == True:
							img = cv2.imread(imgDir + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + str(imgCnt) + '.png', img)

					elif imtype == 'split_temporal':

						# time difference with respect to the event that triggers the image generation
						if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

						# ON events
						diffTime     = np.subtract(refTime, timeEventsON)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						storeEventsON  = np.multiply(accumEventsON, expFactor)
						storeEventsON  = np.asarray(storeEventsON, dtype=np.uint8)

						# OFF events
						diffTime     = np.subtract(refTime, timeEventsOFF)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						storeEventsOFF  = np.multiply(accumEventsOFF, expFactor)
						storeEventsOFF  = np.asarray(storeEventsOFF, dtype=np.uint8)

						# save the image
						img = Image.fromarray(storeEventsON)
						img.save(imgDir + 'ON/' + str(imgCnt) + '.png')
						img = Image.fromarray(storeEventsOFF)
						img.save(imgDir + 'OFF/' + str(imgCnt) + '.png')

						# blur the image?
						if blur == True:
							img = cv2.imread(imgDir + 'ON/' + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + 'ON/' + str(imgCnt) + '.png', img)
							img = cv2.imread(imgDir + 'OFF/' + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + 'OFF/' + str(imgCnt) + '.png', img)
	
					elif imtype == 'normal':

						# save the image
						img = Image.fromarray(accumEvents)
						img.save(imgDir + str(imgCnt) + '.png')

						# blur the image?
						if blur == True:
							img = cv2.imread(imgDir + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + str(imgCnt) + '.png', img)

						# reset the image
						accumEvents = np.zeros((128, 128), dtype=np.uint8)
						accumEvents.fill(127)

					elif imtype == 'normal_mono':

						# save the image
						img = Image.fromarray(accumEvents)
						img.save(imgDir + str(imgCnt) + '.png')

						# blur the image?
						if blur == True:
							img = cv2.imread(imgDir + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + str(imgCnt) + '.png', img)

						# reset the image
						accumEvents = np.zeros((128, 128), dtype=np.uint8)
						accumEvents.fill(0)

					elif imtype == 'split_normal':

						# save the image
						img = Image.fromarray(accumEventsON)
						img.save(imgDir + 'ON/' + str(imgCnt) + '.png')
						img = Image.fromarray(accumEventsOFF)
						img.save(imgDir + 'OFF/' + str(imgCnt) + '.png')

						# blur the image?
						if blur == True:
							img = cv2.imread(imgDir + 'ON/' + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + 'ON/' + str(imgCnt) + '.png', img)
							img = cv2.imread(imgDir + 'OFF/' + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + 'OFF/' + str(imgCnt) + '.png', img)

						# reset the image
						accumEventsON = np.zeros((128, 128), dtype=np.uint8)
						accumEventsON.fill(0)
						accumEventsOFF = np.zeros((128, 128), dtype=np.uint8)
						accumEventsOFF.fill(0)

					elif imtype == 'temp_mono':
						if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
						elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

						diffTime     = np.subtract(refTime, timeEvents)
						expFactor    = np.exp(np.multiply(expMatrix, diffTime))
						storeEvents  = np.multiply(accumEvents, expFactor)
						storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

						# save the image
						img = Image.fromarray(storeEvents)
						img.save(imgDir + str(imgCnt) + '.png')

						# blur the image?
						if blur == True:
							img = cv2.imread(imgDir + str(imgCnt) + '.png')
							img = cv2.GaussianBlur(img,(5,5),0)
							cv2.imwrite(imgDir + str(imgCnt) + '.png', img)

					# accumulate events in an image
					if imtype == 'normal':
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif imtype == 'normal_mono':
						accumEvents[e.y, e.x] = 255

					elif imtype == 'split_normal':
						if p == '1':
							accumEventsON[e.y, e.x] = 255
						else:
							accumEventsOFF[e.y, e.x] = 255

					elif imtype == 'temporal':
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						
						if p == '1':
							accumEvents[e.y, e.x] = 255
						else:
							accumEvents[e.y, e.x] = 0

					elif imtype == 'split_temporal':
						if p == '1':
							if expTime == 'ms': timeEventsON[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsON[e.y, e.x] = ts     # us
							accumEventsON[e.y, e.x] = 255
						else:
							if expTime == 'ms': timeEventsOFF[e.y, e.x] = ts/1000. # ms
							elif expTime == 'us': timeEventsOFF[e.y, e.x] = ts     # us
							accumEventsOFF[e.y, e.x] = 255

					elif imtype == 'temp_mono':
						if expTime == 'ms': timeEvents[e.y, e.x] = ts/1000. # ms
						elif expTime == 'us': timeEvents[e.y, e.x] = ts     # us
						accumEvents[e.y, e.x] = 255

		# counter
		imgCnt += 1

		if imtype == 'temporal':
			if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

			diffTime     = np.subtract(refTime, timeEvents)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			relIntensity = np.subtract(accumEvents, refIntensity)
			storeEvents  = np.add(np.multiply(relIntensity, expFactor), refIntensity)
			storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

			# save the image
			img = Image.fromarray(storeEvents)
			img.save(imgDir + str(imgCnt) + '.png')

			# blur the image?
			if blur == True:
				img = cv2.imread(imgDir + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + str(imgCnt) + '.png', img)

		elif imtype == 'split_temporal':

			# time difference with respect to the event that triggers the image generation
			if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

			# ON events
			diffTime     = np.subtract(refTime, timeEventsON)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			storeEventsON  = np.multiply(accumEventsON, expFactor)
			storeEventsON  = np.asarray(storeEventsON, dtype=np.uint8)

			# OFF events
			diffTime     = np.subtract(refTime, timeEventsOFF)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			storeEventsOFF  = np.multiply(accumEventsOFF, expFactor)
			storeEventsOFF  = np.asarray(storeEventsOFF, dtype=np.uint8)

			# save the image
			img = Image.fromarray(storeEventsON)
			img.save(imgDir + 'ON/' + str(imgCnt) + '.png')
			img = Image.fromarray(storeEventsOFF)
			img.save(imgDir + 'OFF/' + str(imgCnt) + '.png')

			# blur the image?
			if blur == True:
				img = cv2.imread(imgDir + 'ON/' + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + 'ON/' + str(imgCnt) + '.png', img)
				img = cv2.imread(imgDir + 'OFF/' + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + 'OFF/' + str(imgCnt) + '.png', img)

		elif imtype == 'normal':
			img = Image.fromarray(accumEvents)
			img.save(imgDir + str(imgCnt) + '.png')

			# blur the image?
			if blur == True:
				img = cv2.imread(imgDir + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + str(imgCnt) + '.png', img)

		elif imtype == 'normal_mono':
			img = Image.fromarray(accumEvents)
			img.save(imgDir + str(imgCnt) + '.png')

			# blur the image?
			if blur == True:
				img = cv2.imread(imgDir + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + str(imgCnt) + '.png', img)

		elif imtype == 'split_normal':

			# save the image
			img = Image.fromarray(accumEventsON)
			img.save(imgDir + 'ON/' + str(imgCnt) + '.png')
			img = Image.fromarray(accumEventsOFF)
			img.save(imgDir + 'OFF/' + str(imgCnt) + '.png')

			# blur the image?
			if blur == True:
				img = cv2.imread(imgDir + 'ON/' + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + 'ON/' + str(imgCnt) + '.png', img)
				img = cv2.imread(imgDir + 'OFF/' + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + 'OFF/' + str(imgCnt) + '.png', img)

		elif imtype == 'temp_mono':
			if expTime == 'ms': refTime = np.full((128, 128), ts/1000.) # ms
			elif expTime == 'us': refTime = np.full((128, 128), ts)     # us

			diffTime     = np.subtract(refTime, timeEvents)
			expFactor    = np.exp(np.multiply(expMatrix, diffTime))
			storeEvents  = np.multiply(accumEvents, expFactor)
			storeEvents  = np.asarray(storeEvents, dtype=np.uint8)

			# save the image
			img = Image.fromarray(storeEvents)
			img.save(imgDir + str(imgCnt) + '.png')

			# blur the image?
			if blur == True:
				img = cv2.imread(imgDir + str(imgCnt) + '.png')
				img = cv2.GaussianBlur(img,(5,5),0)
				cv2.imwrite(imgDir + str(imgCnt) + '.png', img)


		# close the bagfile
		bag.close()

		# copy trajectory and ventral flow files
		os.system('cp ' + pathFrom + '/' + bagFile + '/trajectory.txt ' + imgDir)
		os.system('cp ' + pathFrom + '/' + bagFile + '/ventral_flow.txt ' + imgDir)