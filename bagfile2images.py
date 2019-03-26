#!/usr/bin/env python

import os
import csv
import glob
import time
import numpy as np
import multiprocessing
from dataset_bagfiles import *
from joblib import Parallel, delayed


def generate_images(i):

	# generate images of different type
	data.generate_images(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		accumTime = accumTime,           # delta_t
		imtype    = imtype, 
		expScale  = expScale,
		datasetFolder = foldername, # name of the dataset
		blur = False, 
		imageResMs=10)
	print(dirList[i])


def generate_images_cnst_variance(i):

	# generate images with constant variance
	data.generate_images_cnst_variance(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i],
		datasetFolder = foldername, # name of the dataset
		variance_obj  = 0.125)                   # final variance
	print(dirList[i])


def generate_aedat(i):

	# generate .aedat files
	data.generate_aedat(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = foldername)
	print(dirList[i])


def generate_csv(i):

	# generate .csv files
	data.generate_csv(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = foldername)
	print(dirList[i])


def generate_csv_64(i):

	# generate .csv files
	data.generate_csv_64(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = foldername)
	print(dirList[i])


def generate_csv_32(i):

	# generate .csv files
	data.generate_csv_32(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = foldername)
	print(dirList[i])


'''
imtype:
	- normal: gray canvas with events colored depending on polarity.
	- normal_mono: black canvas with events as white pixels.
	- split_normal: black canvas with events as white pixels.
					ON/OFF events stored in different images.
	- temporal: images with exponential decay.
	- split_temporal: images with exponential decay.
					  ON/OFF events stored in different images.
  	- temp_mono: same as temporal but all events are white pixels.
'''


# initialize directories and class
global data, pathTo, pathFrom, foldername
pathTo     = 'generated_datasets/images' # directory to generate the dataset
pathFrom   = 'generated_datasets/fede'   # directory with bagfiles

data = dataset(pathTo)

# list with directories in pathFrom
global dirList, dirListLen
dirList    = os.listdir(pathFrom)
dirList = [d for d in os.listdir(pathFrom) if os.path.isdir(os.path.join(pathFrom, d))]
dirListLen = len(dirList)

# initialize new directory if needed
global imtype, expScale, accumTime
imtype    = 'temporal'	# (normal, normal_mono, split_normal, temporal, split_temporal, temp_mono)
expScale  = 0.000025 # used with temporal images
accumTime = 1000	# used with normal images

if imtype == 'temporal' or imtype == 'split_temporal' or imtype == 'temp_mono':
	foldername = imtype + '_' + str(int(expScale*1000000))
else:
	foldername = imtype + '_' + str(accumTime)

# get the images in parallel from the bagfiles
num_cores = max(multiprocessing.cpu_count()-1, 1)
Parallel(n_jobs=num_cores)(delayed(generate_images)(i) for i in xrange(0, dirListLen))

data.generate_data_set_csv(datasetFolder=pathTo+'/'+foldername)
