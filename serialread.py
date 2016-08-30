# this is a python script that plots the last 50 batches of data received through a serial port

import numpy                            # enriched mathematical features
import matplotlib.pyplot as plt	        # plotting	
import serial			           	 	# access to serial port
from drawnow import *		     	    # live plotting

CO = []		# array for storing data received through serial port
k = 0		# counter
plt.ion()	# tell matplotlib we are gonna do a live plotting session

arduinoStuff = serial.Serial('/dev/ttyACM0', 57600);	# open serial communication at
														# port named /dev/ttyACM0 and baud rate 57600

# this function plots stuff
def COplot():
	plt.title('Carbon Monoxide live stream')
	plt.grid(True)
	plt.ylim(4,6)
	plt.ylabel('CO in parts per billion')
	plt.plot(CO, 'r-', label = 'CO')
	plt.legend(loc = 'upper left')

#this functions run infinitely
while (True):
	
	# do nothing if you don't have data coming in
	while(arduinoStuff.inWaiting == 0): 
		pass

	string_line = arduinoStuff.readline()	# you have data !! let's read it
	string_array = string_line.split(' ')	# split sentence into words
	
	to_be_appended = float(string_array[2])	# this word has some valuable data, convert it to a number
	CO.append(to_be_appended)				# append that number
	
	drawnow(COplot)				# drawnow magic
	plt.pause(.000001)			# wait a bit

	k = k+1					# counts how many batches of data were received
	if (k>50)				# after the first 50 batches
		CO.pop(0)			# remove first batch