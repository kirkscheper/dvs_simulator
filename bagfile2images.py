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
		accumTime = 1000,           # delta_t
		imtype    = imtype, 
		expScale  = 0.00005, 
		expTime   = 'us',
		datasetFolder = foldername, # name of the dataset
		blur = False)
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
pathTo     = '/media/fedepare/Datos/Ubuntu/Projects/NewDeepDVS' # directory to generate the dataset
pathFrom   = '/media/fedepare/Datos/Ubuntu/Projects/bagfiles'   # directory with bagfiles
foldername = 'deleteNOWWWWWWWW'                         # dataset name
data = dataset(pathTo)

# list with directories in pathFrom
global dirList, dirListLen
dirList    = os.listdir(pathFrom)
dirListLen = len(dirList)

# initialize new directory if needed
global imtype
imtype = 'temp_mono'

# get the images in parallel from the bagfiles
num_cores = multiprocessing.cpu_count()
Parallel(n_jobs=num_cores)(delayed(generate_images)(i) for i in xrange(0, dirListLen))