import os
import sys
import shutil
import numpy as np
from pygame import *
from dataset import *
from pygamehelper import *
from pygame.locals import *
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


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
				tRaw.append((time-refTime) * 2)
				xRaw.append(self.xScale * 2*(pos[0] - self.w/2)/float(self.w))
				yRaw.append(self.yScale * 2*(self.h/2-pos[1])/float(self.h))

				# time
				print 'Time: {0}\r'.format((time-refTime) * 2 * 10**(-3)),
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


def process_trajectory(altitude):

	global tProc, xProc, yProc
	global wx, wy, D

	# fit a spline for interpolation
	xSpline = interp1d(tRaw, xRaw, kind='cubic')
	ySpline = interp1d(tRaw, yRaw, kind='cubic')

	# loop over the data ms by ms
	tProc = []
	xProc = []
	yProc = []
	for i in xrange(0,tRaw[-1]+1):
		tProc.append(i)
		xProc.append(xSpline(i))
		yProc.append(ySpline(i))

	# write the txt file
	with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
		for i in xrange(0,len(tProc)):
	 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(tProc[i]), xProc[i], yProc[i], altitude, 1.0, 0.0, 0.0, 0.0))


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
#s = Starter(800, 800, 2.5, 2.5)
#s.mainLoop(1000)

# process the results
altitude = 0.5
print "Processing the trajectory..."
#process_trajectory(altitude)

xProc = []
yProc = []
tProc = []
#th = np.linspace(np.pi, -np.pi, 4000)
#for i in xrange(0,len(th)):
#	xProc.append(np.cos(th[i]))
#	yProc.append(np.sin(th[i]))
#	tProc.append(i)
xProc = np.linspace(-1, 1, 4000)
yProc = np.linspace(-1, 1, 4000)
for i in xrange(0,len(xProc)):
	#yProc.append(np.sin(2*np.pi*i/2000))
	tProc.append(i)

# write the txt file
with open(os.getcwd() + "/src/rpg_davis_simulator/datasets/scenes/customTraj.txt", "w") as text_file:
	for i in xrange(0,len(tProc)):
 		text_file.write("%i %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (int(tProc[i]), xProc[i], yProc[i], altitude, 1.0, 0.0, 0.0, 0.0))

# render and simulate the scene
run_simulator()
data = dataset('/media/fedepare/Datos/Ubuntu/Projects/TF_Continuous')

# generate aedat file
#data.generate_aedat()

# generate images
imgCnt = data.generate_images(1000)

# generate groundtruth
data.generate_groundtruth(tProc, xProc, yProc, altitude, imgCnt)

# clean data generated
path = 'src/rpg_davis_simulator/datasets/full_datasets'
if os.path.exists(path):
	shutil.rmtree(path)

path = 'src/rpg_davis_simulator/datasets/rosbags'
if os.path.exists(path):
	shutil.rmtree(path)
