#!/usr/bin/env python

import os
import shutil

while True:
	entry = raw_input("Are you sure? [y/n] ")
	if entry == 'y' or entry == 'Y':

		# clean the aedat directory
		path = 'aedat'
		if os.path.exists(path):
			shutil.rmtree(path)

		# delete the renderized scenes
		path = 'src/rpg_davis_simulator/datasets/full_datasets'
		if os.path.exists(path):
			shutil.rmtree(path)

		# delete the rosbags
		path = 'src/rpg_davis_simulator/datasets/rosbags'
		if os.path.exists(path):
			shutil.rmtree(path)
			
		break
	elif entry == 'n' or entry == 'N':
		break
	else:
		print "Try again."