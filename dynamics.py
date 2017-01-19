#!/usr/bin/env python

import numpy as np
import scipy as sp

newfile = open("/Users/chandrayeebasu/IdeaProjects/opends_project2/carData_track1.txt")
fileread = newfile.readlines()
count = 0

for i in range(1,len(fileread)):
	data = str.split(fileread[i],":")
	if (count!=0):
		p_time = time
		p_speed = speed
	time = data[0]
	posx = data [1]
	posy = data[2]
	posz = data[3]
	rotx = data[4]
	roty = data[5]
	rotz = data[6]
	rotw = data[7]
	speed = data[8]
	steering = data[9]
	gas = data[10]
	brake = data[11]
	engineOn = data[12]
	
	u = np.array([steering, gas])
	
	#if np.float(gas) > 0:
	if (count!=0): 
		speedchange = np.double(np.float(speed) - np.float(p_speed))
		timechange = int(time) - int(p_time)
		print("speed change: " + str(speedchange) + " @gas: " + gas + " @timechange " + str(timechange))
	
	count = count + 1