#!/usr/bin/env python

""" 
Author: F. Paredes Valles
Created: 27-06-2017
"""

import os
import csv
import glob
import time
import numpy as np
import multiprocessing
from dataset_bagfiles import *
from joblib import Parallel, delayed


def generate_images(i):

	# generate images
	data.generate_images(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		accumTime = 2500, 
		imtype    = imtype, 
		expScale  = 0.00005, 
		expTime   = 'us',
		datasetFolder = datasetFolder)

	# print the directory when done computing
	print(dirList[i])


# initialize directories and class
global data
pathTo   = '/media/fedepare/Datos/Ubuntu/Projects/NewDeepDVS'
pathFrom = '/media/fedepare/Datos/Ubuntu/Projects/bagfiles'
data = dataset(pathTo)

# list with directories in pathFrom
global dirList, dirListLen
dirList = os.listdir(pathFrom)
dirListLen = len(dirList)

# initialize new directory if needed
global imtype
imtype = 'split_normal'
datasetFolder = 'images_' + imtype
cnt = 2
while True:
	if os.path.exists(pathTo + '/' + datasetFolder):
		datasetFolder = 'images_' + imtype + '_' + str(cnt)
		cnt += 1
	else: break

# get the images in parallel from the bagfiles
num_cores = multiprocessing.cpu_count()
Parallel(n_jobs=num_cores)(delayed(generate_images)(i) for i in xrange(0, dirListLen))

# # create .csv file for Keras
# folders = [name for name in os.listdir(pathTo + '/' + datasetFolder)]
# ofile  = open(pathTo + '/' + datasetFolder + '/data_file.csv', "wb")
# writer = csv.writer(ofile, delimiter=',', quoting=csv.QUOTE_ALL)

# for i in xrange(0,len(folders)):
# 	folderName = folders[i].split('_')
# 	if len(folderName) == 1:
# 		writer.writerow(['train', folders[i]])
# 	elif folderName[1] != '0':
# 		writer.writerow(['test', folders[i]])