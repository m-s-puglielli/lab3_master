"""
Hayden Liao  &&  Maximilian Puglielli
February 24th, 2020 @ 14:20
CS-425-A: Introduction to Robotics
Lab #3: Computer Vision & General Suffering
"""

import socket
from time import *
from pynput import keyboard
import sys
import threading
import enum
import urllib.request
import cv2
import numpy
import copy
from random import randint

socketLock = threading.Lock()
imageLock = threading.Lock()

# SET THIS TO THE RASPBERRY PI's IP ADDRESS
IP_ADDRESS = "192.168.1.103"

class States(enum.Enum):
	SEARCH			= enum.auto()
	TURN_LEFT		= enum.auto()
	TURN_RIGHT		= enum.auto()
	MOVE_FORWARD	= enum.auto()

class StateMachine(threading.Thread):

	def __init__(self):
		# NOTE: MUST call this to make sure we setup the thread correctly
		threading.Thread.__init__(self)

		# CONFIGURATION PARAMETERS
		global IP_ADDRESS
		self.IP_ADDRESS = IP_ADDRESS
		self.CONTROLLER_PORT = 5001
		self.TIMEOUT = 10   # If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
		self.STATE = States.SEARCH
		self.RUNNING = True
		self.DIST = False
		self.video = ImageProc()

		# START VIDEO
		self.video.start()

		# CONNECT TO THE MOTORCONTROLLER
		try:
			with socketLock:
				self.sock = socket.create_connection((self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
				self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
			print("Connected to RP")
		except Exception as e:
			print("ERROR with socket connection", e)
			sys.exit(0)

		# CONNECT TO THE ROBOT
		# The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!
		with socketLock:
			self.sock.sendall("i /dev/ttyUSB0".encode())
			print("Sent command")
			result = self.sock.recv(128).decode()
			print(result)
			if result != "i /dev/ttyUSB0":
				self.RUNNING = False

		self.sensors = Sensing(self.sock)
		# START READING DATA
		self.sensors.start()

		# COLLECT EVENTS UNTIL RELEASED
		self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
		self.listener.start()

	# BEGINNING OF THE CONTROL LOOP
	def run(self):
		while(self.RUNNING):
			sleep(0.01)
			if self.STATE == States.SEARCH:
				pass
				#rnd = randint(1, 3)
				#if rnd == 1:
				#	self.STATE = States.TURN_LEFT
				#elif rnd == 2:
				#	self.STATE = States.TURN_RIGHT
				#elif rnd == 3:
				#	self.STATE = States.MOVE_FORWARD
				#else:
				#	print("ERROR: rnd is corrupted")

			elif self.STATE == States.TURN_LEFT:
				self.sock.sendall("a drive_direct(500, -500)".encode())
				discard = self.sock.recv(128).decode()
				sleep(0.05)
				self.sock.sendall("a drive_straight(0)".encode())
				discard = self.sock.recv(128).decode()
				self.STATE = States.SEARCH

			elif self.STATE == States.TURN_RIGHT:
				self.sock.sendall("a drive_direct(-500, 500)".encode())
				discard = self.sock.recv(128).decode()
				sleep(0.05)
				self.sock.sendall("a drive_straight(0)".encode())
				discard = self.sock.recv(128).decode()
				self.STATE = States.SEARCH

			elif self.STATE == States.MOVE_FORWARD:
				self.sock.sendall("a drive_direct(500, 500)".encode())
				discard = self.sock.recv(128).decode()
				sleep(0.5)
				self.sock.sendall("a drive_straight(0)".encode())
				discard = self.sock.recv(128).decode()
				self.STATE = States.SEARCH

			else:
				print("ERROR: self.STATE is corrupted")

	# END OF CONTROL LOOP


		# STOP ANY OTHER THREADS TALKING TO THE ROBOT
		self.sensors.RUNNING = False
		self.video.RUNNING = False

		# WAIT FOR THREADS TO FINISH
		sleep(1)

		# NEED TO DISCONNECT
		"""
		The c command stops the robot and disconnects.
		The stop command will also reset the Create's mode to a battery safe PASSIVE.
		It is very important to use this command.
		"""
		with socketLock:
			self.sock.sendall("c".encode())
			print(self.sock.recv(128).decode() + "\n")
			self.sock.close()

		# If the user didn't request to halt, we should stop listening anyways
		self.listener.stop()

#		self.sensors.join()
#		self.video.join()

	def on_press(self, key):
		# NOTE: DO NOT attempt to use the socket directly from here
		try:
			print('alphanumeric key {0} pressed'.format(key.char))
		except AttributeError:
			print('special key {0} pressed'.format(key))

	def on_release(self, key):
		# NOTE: DO NOT attempt to use the socket directly from here
		print('{0} released'.format(key))
		if key == keyboard.Key.esc:
			# STOP LISTENER
			self.RUNNING			= False
			self.sensors.RUNNING	= False
			self.video.RUNNING		= False
			return False

# END OF STATEMACHINE



class Sensing(threading.Thread):
	def __init__(self, socket):
		# NOTE: MUST call this to make sure we setup the thread correctly
		threading.Thread.__init__(self)
		self.RUNNING = True
		self.sock = socket

	def run(self):
		while self.RUNNING:
			sleep(0.1)
			# This is where I would get a sensor update
			# Store it in this class
			# You can change the polling frequency to optimize performance, don't forget to use socketLock
			with socketLock:
				self.sock.sendall("a distance".encode())
#				print("distance = {0}\n".format(self.sock.recv(128).decode()))

# END OF SENSING



class ImageProc(threading.Thread):

	def __init__(self):
		# NOTE: MUST call this to make sure we setup the thread correctly
		threading.Thread.__init__(self)
		global IP_ADDRESS
		self.IP_ADDRESS = IP_ADDRESS
		self.PORT = 8081
		self.RUNNING = True
		self.latestImg = []
		self.feedback = []
		# cone in the evening
		self.thresholds = {'low_hue':       127, 'high_hue':          14,
							'low_saturation': 0, 'high_saturation': 255,
							'low_value':      0, 'high_value':       255}

		# tennisball in the morning
		self.thresholds = {'low_hue':       30, 'high_hue':          56,
							'low_saturation': 28, 'high_saturation': 160,
							'low_value':      84, 'high_value':       255}


	def run(self):
		url = "http://"+self.IP_ADDRESS+":"+str(self.PORT)
		stream = urllib.request.urlopen(url)
		while(self.RUNNING):
#			sleep(0.5)
			bytes = b''
			while self.RUNNING:
				# Image size is about 40k bytes, so this loops about 5 times
				bytes += stream.read(8192)
				a = bytes.find(b'\xff\xd8')
				b = bytes.find(b'\xff\xd9')
				if a > b:
					bytes = bytes[b+2:]
					continue
				if	a != -1 and\
					b != -1:
					jpg = bytes[a:b+2]
#					bytes = bytes[b+2:]
#					print("found image", a, b, len(bytes))
					break
			img = cv2.imdecode(numpy.frombuffer(jpg, dtype=numpy.uint8),cv2.IMREAD_COLOR)
			# Resize to half size so that image processing is faster
			img = cv2.resize(img, ((int)(len(img[0])/4),(int)(len(img)/4)))

			with imageLock:
				# Make a copy not a reference
				self.latestImg = copy.deepcopy(img)

			# Pass by reference for all non-primitve types in Python
			self.doImgProc(img)

			# After image processing you can update here to see the new version
			with imageLock:
				self.feedback = copy.deepcopy(img)

			# erode and dilate the processed image
			self.dilate_big(img, 2, 2)
			self.dilate_big(img, 2, 2)
			self.erode_big(img, 2, 2)
			self.erode_big(img, 2, 2)
			self.erode(img, 2)

			# after eroding the image you can see the update in feedback_filtered
			with imageLock:
				self.feedback_filtered = copy.deepcopy(img)

	def draw_box(self, original):
		pass

	def setThresh(self, name, value):
		self.thresholds[name] = value

	# if a pixel is not interesting, make all surrounding pixels not interesting
	# white, (255,255,255) is "interesting", black is not
	def erode(self, original, scale):
		imgToModify = original

		for y in range(1, len(original)-1, scale):
			for x in range(1, len(original)-1, scale):
				if	original[y][x][0] == 0 and\
					original[y][x][1] == 0 and\
					original[y][x][2] == 0:
					for i in range(3):
						imgToModify [y - 1]	[x - 1]	[i] = 0
						imgToModify [y - 1]	[x]		[i] = 0
						imgToModify [y - 1]	[x + 1]	[i] = 0
						imgToModify [y]		[x - 1]	[i] = 0
						imgToModify [y]		[x + 1]	[i] = 0
						imgToModify [y + 1]	[x - 1]	[i] = 0
						imgToModify [y + 1]	[x]		[i] = 0
						imgToModify [y + 1]	[x + 1]	[i] = 0
		self.feedback_filtered = imgToModify


	def erode_big(self, original, scale, how_big):
		imgToModify = original

		for y in range(how_big, len(original)-how_big, scale):
			for x in range(how_big, len(original)-how_big, scale):
				if	original[y][x][0] == 0 and\
					original[y][x][1] == 0 and\
					original[y][x][2] == 0:
					for i in range(3):
						for j in range(y-how_big, y+how_big):
							for k in range(x-how_big, x+how_big):
								imgToModify[j][k][i] = 0
		self.feedback_filtered = imgToModify


	# if a pixel is interesting, make all surrounding pixels interesting
	# white, (255,255,255) is "interesting", black is not
	def dilate(self, original, scale):
		imgToModify = original

		for y in range(1, len(original)-1, scale):
			for x in range(1, len(original)-1, scale):
				if	original[y][x][0] == 255 and\
					original[y][x][1] == 255 and\
					original[y][x][2] == 255:
					for i in range(3):
						imgToModify [y - 1]	[x - 1]	[i] = 255
						imgToModify [y - 1]	[x]		[i] = 255
						imgToModify [y - 1]	[x + 1]	[i] = 255
						imgToModify [y]		[x - 1]	[i] = 255
						imgToModify [y]		[x + 1]	[i] = 255
						imgToModify [y + 1]	[x - 1]	[i] = 255
						imgToModify [y + 1]	[x]		[i] = 255
						imgToModify [y + 1]	[x + 1]	[i] = 255
		self.feedback_filtered = imgToModify

	def dilate_big(self, original, scale, how_big):
		imgToModify = original

		for y in range(how_big, len(original)-how_big, scale):
			for x in range(how_big, len(original)-how_big, scale):
				if	original[y][x][0] == 255 and\
					original[y][x][1] == 255 and\
					original[y][x][2] == 255:
					for i in range(3):
						for j in range(y-how_big, y+how_big):
							for k in range(x-how_big, x+how_big):
								imgToModify[j][k][i] = 255
		self.feedback_filtered = imgToModify

		# if no pixels are changed
		#	set all pink pixels to interesting



	def doImgProc(self, imgToModify):
#		pixel = self.latestImg[120,160]
#		print("pixel (160, 120) is ",pixel, "in B,G,R order.")

#		hsv_img = cv2.cvtColor(self.latestImg, cv2.COLOR_BGR2HSV)
		hsv_img = cv2.cvtColor(imgToModify, cv2.COLOR_BGR2HSV)

		for y in range(len(hsv_img)):
			for x in range(len(hsv_img[0])):
				if self.thresholds['low_hue'] >= self.thresholds['high_hue']:
					if	(self.thresholds['low_hue']			<= hsv_img[y][x][0] or	hsv_img[y][x][0] <= self.thresholds['high_hue']) and\
						self.thresholds['low_saturation']	<= hsv_img[y][x][1] and	hsv_img[y][x][1] <= self.thresholds['high_saturation'] and\
						self.thresholds['low_value']		<= hsv_img[y][x][2] and	hsv_img[y][x][2] <= self.thresholds['high_value']:
						imgToModify[y][x][0] = 255
						imgToModify[y][x][1] = 255
						imgToModify[y][x][2] = 255
					else:
						imgToModify[y][x][0] = 0
						imgToModify[y][x][1] = 0
						imgToModify[y][x][2] = 0
				else:
					if	self.thresholds['low_hue']			<= hsv_img[y][x][0] and hsv_img[y][x][0] <= self.thresholds['high_hue'] and\
						self.thresholds['low_saturation']	<= hsv_img[y][x][1] and hsv_img[y][x][1] <= self.thresholds['high_saturation'] and\
						self.thresholds['low_value']		<= hsv_img[y][x][2] and hsv_img[y][x][2] <= self.thresholds['high_value']:
						imgToModify[y][x][0] = 255
						imgToModify[y][x][1] = 255
						imgToModify[y][x][2] = 255
					else:
						imgToModify[y][x][0] = 0
						imgToModify[y][x][1] = 0
						imgToModify[y][x][2] = 0

		# CIRCLE TRACKING
		gray = cv2.cvtColor(imgToModify, cv2.COLOR_BGR2GRAY)
		circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)
		if circles is not None:
			circles = np.round(circles[0, :]).astype("int")
			for (x, y, r) in circles:
				cv2.circle(imgToModify, (x, y), r, (0, 255, 0), 4)
				cv2.rectangle(imgToModify, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)


				# CONE DETECTION
				#	if image saturation & value are within their respective upper and lower bounds, and
				#	if image hue is within its respective upper and lower bounds, or
				#	low_hue is greater than high_hue and either
				#		the image's hue is less than high_hue (which is the new low bound) or
				#		the image's hue is greater than low_hue (which is the new high bound)
#				if	((self.thresholds['low_hue'] >= self.thresholds['high_hue'] and (self.thresholds['low_hue'] <= hsv_img[y][x][0] or hsv_img[y][x][0] <= self.thresholds['high_hue'])) or\
#					(self.thresholds['low_hue']			<= hsv_img[y][x][0] and hsv_img[y][x][0] <= self.thresholds['high_hue'])) and\
#					self.thresholds['low_saturation']	<= hsv_img[y][x][1] and hsv_img[y][x][1] <= self.thresholds['high_saturation'] and\
#					self.thresholds['low_value']		<= hsv_img[y][x][2] and hsv_img[y][x][2] <= self.thresholds['high_value']:
#					imgToModify[y][x][0] = 255
#					imgToModify[y][x][1] = 255
#					imgToModify[y][x][2] = 255
#
#				else:
#					imgToModify[y][x][0] = 0
#					imgToModify[y][x][1] = 0
#					imgToModify[y][x][2] = 0
#
#				if self.thresholds['low_blue'] <= self.latestImg[y,x][0] and self.latestImg[y,x][0] <= self.thresholds['high_blue']:
#					imgToModify[y,x][0] = 255
#					imgToModify[y,x][1] = 0
#					imgToModify[y,x][2] = 0
#				if self.thresholds['low_green'] <= self.latestImg[y,x][1] and self.latestImg[y,x][1] <= self.thresholds['high_green']:
#					imgToModify[y,x][0] = 0
#					imgToModify[y,x][1] = 255
#					imgToModify[y,x][2] = 0
#				if self.thresholds['low_red'] <= self.latestImg[y,x][2] and self.latestImg[y,x][2] <= self.thresholds['high_red']:
#					imgToModify[y,x][0] = 0
#					imgToModify[y,x][1] = 0
#					imgToModify[y,x][2] = 255

# END OF IMAGEPROC



if __name__ == "__main__":

	cv2.namedWindow("Original Image View", flags=cv2.WINDOW_AUTOSIZE)
	cv2.moveWindow("Original Image View", 21, 21)

	cv2.namedWindow('Binary View')
	cv2.moveWindow('Binary View', 300, 21)

	cv2.namedWindow('Filtered View')
	cv2.moveWindow('Filtered View', 600, 21)

	cv2.namedWindow('sliders')
	cv2.moveWindow('sliders', 900, 21)

	sm = StateMachine()
	sm.start()

	# Probably safer to do this on the main thread rather than in ImgProc init
	cv2.createTrackbar('low_hue',			'sliders', sm.video.thresholds['low_hue'],			255, lambda x: sm.video.setThresh('low_hue',			x))
	cv2.createTrackbar('high_hue',			'sliders', sm.video.thresholds['high_hue'],			255, lambda x: sm.video.setThresh('high_hue',			x))
	cv2.createTrackbar('low_saturation',	'sliders', sm.video.thresholds['low_saturation'],	255, lambda x: sm.video.setThresh('low_saturation',		x))
	cv2.createTrackbar('high_saturation',	'sliders', sm.video.thresholds['high_saturation'],	255, lambda x: sm.video.setThresh('high_saturation',	x))
	cv2.createTrackbar('low_value',			'sliders', sm.video.thresholds['low_value'],		255, lambda x: sm.video.setThresh('low_value',			x))
	cv2.createTrackbar('high_value',		'sliders', sm.video.thresholds['high_value'],		255, lambda x: sm.video.setThresh('high_value',			x))

	while len(sm.video.latestImg) == 0:
		sleep(1)

	while(sm.RUNNING):
		with imageLock:
			cv2.imshow("Original Image View", sm.video.latestImg)
			cv2.imshow("Binary View",sm.video.feedback)
			cv2.imshow("Filtered View", sm.video.feedback_filtered)
		cv2.waitKey(5)

	cv2.destroyAllWindows()

	sleep(1)

#	sm.join()
