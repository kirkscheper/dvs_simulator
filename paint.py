import os
import sys
import csv
import glob
import rosbag
import struct
import numpy as np
from math import fabs
from math import floor
from pygame import *
from pygamehelper import *
from pygame.locals import *

class Starter(PygameHelper):

	# constructor
	def __init__(self, width = 800, height = 600, xScale = 2.5, yScale = 2.5):
		self.w = width
		self.h = height
		self.xScale = xScale
		self.yScale = yScale
		PygameHelper.__init__(self, size = (self.w, self.h), fill = ((255, 255, 255)))

		# print axes
		pygame.draw.line(self.screen, (0, 0, 0), (self.w/2, self.h), (self.w/2, 0), 1)
		pygame.draw.line(self.screen, (0, 0, 0), (0, self.h/2), (self.w, self.h/2), 1)

		# segments
		wSeg = 10
		xSeg = self.w / 10
		ySeg = self.h / 10
		for i in xrange(1,10):
			pygame.draw.line(self.screen, (0, 0, 0), (0 + xSeg * i, self.h/2 - wSeg), (0 + xSeg * i, self.h/2 + wSeg), 1)
			pygame.draw.line(self.screen, (0, 0, 0), (self.w/2 - wSeg, 0 + ySeg * i), (self.w/2 + wSeg, 0 + ySeg * i), 1)

	# follow mouse motion
	def mouseMotion(self, buttons, pos, rel, time):

		global done
		global draw
		global prev
		global prevPos
		global prevTime
		global refTime
		global tRaw, xRaw, yRaw

		if done == False:

			# paint
			if buttons[0] == 1 and prev == True and prevTime != time:
				pygame.draw.line(self.screen, (0, 0, 0), pos, prevPos, 5)
				prevPos  = pos
				prevTime = time
				draw = True

				# store data
				tRaw.append((time-refTime) / 2)
				xRaw.append(self.xScale * 2*(pos[0] - self.w/2)/float(self.w))
				yRaw.append(self.yScale * 2*(self.h/2-pos[1])/float(self.h))

				# time
				print 'Time: {0}\r'.format((time-refTime) / 2 * 10**(-3)),
				sys.stdout.flush()

			elif buttons[0] == 0 and draw == True:
				done = True
				self.running = False

			# first event perceived
			if buttons[0] == 1 and prev == False:
				prev     = True
				prevPos  = pos
				prevTime = time
				refTime  = time

				# store data
				tRaw.append(0)
				xRaw.append(self.xScale * 2*(pos[0] - self.w/2)/float(self.w))
				yRaw.append(self.yScale * 2*(self.h/2-pos[1])/float(self.h))

def process_trajectory():

	# loop over the data ms by ms
	tProc = []
	xProc = []
	yProc = []
	for i in xrange(0,tRaw[-1]+1):

		# check if this ms exists
		if i in tRaw:

			# indices for interpolation
			idxLow  = tRaw.index(i)
			idxHigh = idxLow + 1

			# store data
			tProc.append(tRaw[idxLow])
			xProc.append(xRaw[idxLow])
			yProc.append(yRaw[idxLow])

		else:
			tProc.append(i)
			xProc.append((xRaw[idxLow]*(tRaw[idxHigh]-i)+xRaw[idxHigh]*(i-tRaw[idxLow]))/(tRaw[idxHigh]-tRaw[idxLow]))
			yProc.append((yRaw[idxLow]*(tRaw[idxHigh]-i)+yRaw[idxHigh]*(i-tRaw[idxLow]))/(tRaw[idxHigh]-tRaw[idxLow]))

	# write the txt file
	with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
		for i in xrange(0,len(tProc)):
	 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(tProc[i]), xProc[i], yProc[i], 1.5, 1.0, 0.0, 0.0, 0.0))

# drawing variables
global done, draw
done = False
draw = False

# filter events
global prev, prevPos, prevTime
prev = False

# reference time
global refTime

# trajectory
global tRaw, xRaw, yRaw
global tProc, xProc, yProc
tRaw = []
xRaw = []
yRaw = []

# run the program
s = Starter(800, 800, 2.5, 2.5)
s.mainLoop(1000)

# process the results
print "Processing the trajectory..."
process_trajectory()

# render and simulate the scene
os.system("roslaunch dvs_simulator_py custom_render.launch")
os.system("roslaunch dvs_simulator_py custom_simulate.launch")

# get the current rosbag file
curDir = os.getcwd()
os.chdir(os.getcwd() + '/src/rpg_davis_simulator/datasets/rosbags/')
bagFile = max(glob.glob('*.bag'), key=os.path.getctime)
bagFilePath = os.getcwd() + '/' + bagFile
os.chdir(curDir)

# split the name and create the aedat filename
bagSplit = bagFile.split('.')
aedatFile = bagSplit[0] + '.aedat'

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
        x = '{0:07b}'.format(128-e.x)
        y = '{0:07b}'.format(128-e.y)
        p = '1' if e.polarity else '0'
        address = "0" + y + x + p # TODO: Polarity is INVERTED in .aedat files

        # write the event using big endian format
        file.write("%s" % struct.pack('>I', int(address, 2)))
        file.write("%s" % struct.pack('>I', int(ts)))

bag.close()
