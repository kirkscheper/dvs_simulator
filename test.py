import os
from os import listdir
from os.path import isfile, join

accumTime = 1000
cnt = 0
readFlag = False
path = 'aedat/images/3planesDVS-20170504-181213_1000us/'
while True:

	# create the name of the file
	file = '3planesDVS-20170504-181213_1000us_' + str(cnt) + '.png'

	# check wether it is included in the directory
	if os.path.isfile(path + file):

		# update the flag
		readFlag  = True
		time = cnt * accumTime
		print time / 1000.0

		


	# if the file is not in the directory after reading some files, break
	elif readFlag == True:
		break

	# update the counter
	cnt += 1
