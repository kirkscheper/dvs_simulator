#!/usr/bin/env python

""" 
clean.py is a Python script that deletes the files that have been 
generated using the rpg_davis_simulator [1] and the run.py script after it.

[1] rpg_davis_simulator: https://github.com/uzh-rpg/rpg_davis_simulator

Author: F. Paredes Valles
Created: 04-05-2017
"""

import os
import shutil

path = 'aedat'
if os.path.exists(path):
	shutil.rmtree(path)

path = 'src/rpg_davis_simulator/datasets/full_datasets'
if os.path.exists(path):
	shutil.rmtree(path)

path = 'src/rpg_davis_simulator/datasets/rosbags'
if os.path.exists(path):
	shutil.rmtree(path)