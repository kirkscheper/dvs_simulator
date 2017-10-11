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

def new_dataset(pathTo, datasetFolder, imtype):
	cnt = 2
	while True:
		if os.path.exists(pathTo + '/' + datasetFolder):
			datasetFolder = 'images_' + imtype + '_' + str(cnt)
			cnt += 1
		else: break

	return datasetFolder


def generate_images(i):

	# do not use test info
	folder = dirList[i].split('_')
	if folder[0] != 'test':

		# generate images
		data.generate_images(
			pathFrom  = pathFrom, 
			bagFile   = dirList[i], 
			accumTime = 1000, 
			imtype    = imtype, 
			expScale  = 0.00005, 
			expTime   = 'us',
			datasetFolder = 'images_split_normal_1000',
			blur = False)

		# print the directory when done computing
		print(dirList[i])


def generate_images_cnst_variance(i):

	# do not use test info
	folder = dirList[i].split('_')
	if folder[0] != 'test':

		# generate images
		data.generate_images_cnst_variance(
			pathFrom  = pathFrom, 
			bagFile   = dirList[i],
			datasetFolder = 'images_split_variance',
			variance_obj  = 0.125)

		# print the directory when done computing
		print(dirList[i])

def generate_aedat(i):

	# do not use test info
	folder = dirList[i].split('_')

	# generate images
	data.generate_aedat(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = 'datasets')

	# print the directory when done computing
	print(dirList[i])


def generate_csv(i):

	# do not use test info
	folder = dirList[i].split('_')

	# generate images
	data.generate_csv(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = 'datasets_csv')

	# print the directory when done computing
	print(dirList[i])


def generate_csv_64(i):

	# do not use test info
	folder = dirList[i].split('_')

	# generate images
	data.generate_csv_64(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = 'datasets_csv_64')

	# print the directory when done computing
	print(dirList[i])


def generate_csv_32(i):

	# do not use test info
	folder = dirList[i].split('_')

	# generate images
	data.generate_csv_32(
		pathFrom  = pathFrom, 
		bagFile   = dirList[i], 
		datasetFolder = 'datasets_csv_32')

	# print the directory when done computing
	print(dirList[i])


def generate_statistics(i):

	# do not use test info
	folder = dirList[i].split('_')
	if folder[0] != 'test':

		# generate images
		eventRate = data.data_statistics(pathFrom = pathFrom, bagFile = dirList[i])

		# print the directory when done computing
		print(dirList[i])
		print(eventRate)


def generate_meanflow_events(i):

	# do not use test info
	folder = dirList[i].split('_')
	if len(folder) == 1:

		# generate images
		data.generate_meanflow_events(
			pathFrom  = pathFrom, 
			bagFile   = dirList[i], 
			accumTime = 1000, 
			imtype    = imtype, 
			expScale  = 0.00005, 
			expTime   = 'us',
			datasetFolder = 'delete',
			blur = False)


# initialize directories and class
global data
pathTo   = '/media/fedepare/Datos/Ubuntu/Projects/NewDeepDVS'
pathFrom = '/media/fedepare/Datos/Ubuntu/Projects/bagfiles'
data = dataset(pathTo)

# list with directories in pathFrom
global dirList, dirListLen
dirList = os.listdir(pathFrom)
dirList = ['circle_natural_125', 'circle_natural_150']
dirListLen = len(dirList)

# initialize new directory if needed
global imtype
imtype = 'split_normal'
#datasetFolder = new_dataset(pathTo, datasetFolder, imtype)

# get the images in parallel from the bagfiles
num_cores = multiprocessing.cpu_count()
Parallel(n_jobs=num_cores)(delayed(generate_images_cnst_variance)(i) for i in xrange(0, dirListLen))