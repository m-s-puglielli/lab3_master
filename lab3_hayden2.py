"""
Maximilian Puglielli  &&  Hayden Liao
February 24th, 2020 @ 14:20
CS-425-A: Introduction to Robotics
Lab #3: Computer Vision
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

socketLock = threading.Lock()
imageLock = threading.Lock()

# SET THIS TO THE RASPBERRY PI's IP ADDRESS
IP_ADDRESS = "192.168.1.103"

class States(enum.Enum):
    LISTEN = enum.auto()

class StateMachine(threading.Thread):

    def __init__(self):
        # NOTE: MUST call this to make sure we setup the thread correctly
        threading.Thread.__init__(self)
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10   # If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.LISTEN
        self.RUNNING = True
        self.DIST = False
        self.video = ImageProc()
        # START VIDEO
        self.video.start()

        # CONNECT TO THE MOTORCONTROLLER
        try:
            with socketLock:
                self.sock = socket.create_connection( (self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to RP")
        except Exception as e:
            print("ERROR with socket connection", e)
            sys.exit(0)

        # CONNECT TO THE ROBOT
        """ The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!"""
        with socketLock:
            self.sock.sendall("i /dev/ttyUSB0".encode())
            print("Sent command")
            result = self.sock.recv(128)
            print(result)
            if result.decode() != "i /dev/ttyUSB0":
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
            sleep(0.1)
            if self.STATE == States.LISTEN:
                pass
            # TODO: Work here

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
            print(self.sock.recv(128))
            self.sock.close()

        # If the user didn't request to halt, we should stop listening anyways
        self.listener.stop()

#        self.sensors.join()
#        self.video.join()

    def on_press(self, key):
        # NOTE: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        # NOTE: DO NOT attempt to use the socket directly from here
        print('{0} released'.format(key))
        if key == keyboard.Key.esc or key == keyboard.Key.ctrl:
            # STOP LISTENER
            self.RUNNING = False
            self.sensors.RUNNING = False
            self.video.RUNNING = False
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
                print(self.sock.recv(128))

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
        self.thresholds = {'low_red':0,'high_red':0, 'low_green':0, 'high_green':0, 'low_blue':0, 'high_blue':0}

    def run(self):
        url = "http://"+self.IP_ADDRESS+":"+str(self.PORT)
        stream = urllib.request.urlopen(url)
        while(self.RUNNING):
            # sleep(0.5)
            bytes = b''
            while self.RUNNING:
                # Image size is about 40k bytes, so this loops about 5 times
                bytes += stream.read(8192)
                a = bytes.find(b'\xff\xd8')
                b = bytes.find(b'\xff\xd9')
                if a>b:
                    bytes = bytes[b+2:]
                    continue
                if a!=-1 and b!=-1:
                    jpg = bytes[a:b+2]
#                    bytes = bytes[b+2:]
#                    print("found image", a, b, len(bytes))
                    break
            img = cv2.imdecode(numpy.fromstring(jpg, dtype=numpy.uint8),cv2.IMREAD_COLOR)
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

    def setThresh(self, name, value):
        self.thresholds[name] = value

    def doImgProc(self, imgToModify):
        # pixel = self.latestImg[120,160]
        # print("pixel (160, 120) is ",pixel, "in B,G,R order.")
        hsv_img = cv2.cvtColor(self.latestImg, cv2.COLOR_BGR2HSV)

        for y in range(len(self.latestImg)):
            for x in range(len(self.latestImg[0])):
                if self.latestImg[y,x][0] >= self.thresholds['low_blue'] and self.latestImg[y,x][0] <= self.thresholds['high_blue']:
                    imgToModify[y,x][2] = 0
                    imgToModify[y,x][1] = 0
                    imgToModify[y,x][0] = 255
                if self.latestImg[y,x][1] >= self.thresholds['low_green'] and self.latestImg[y,x][1] <= self.thresholds['high_green']:
                    imgToModify[y,x][2] = 0
                    imgToModify[y,x][1] = 255
                    imgToModify[y,x][0] = 0
                if self.latestImg[y,x][2] >= self.thresholds['low_red'] and self.latestImg[y,x][2] <= self.thresholds['high_red']:
                    imgToModify[y,x][2] = 255
                    imgToModify[y,x][1] = 0
                    imgToModify[y,x][0] = 0
        # TODO: Work here

# END OF IMAGEPROC


if __name__ == "__main__":

    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("Create View", 21, 21)

    cv2.namedWindow('sliders')
    cv2.moveWindow('sliders', 680, 21)

    cv2.namedWindow('Create View2')
    cv2.moveWindow('Create View2', 500, 21)

    sm = StateMachine()
    sm.start()

    # Probably safer to do this on the main thread rather than in ImgProc init
    cv2.createTrackbar('low_red', 'sliders', sm.video.thresholds['low_red'], 255,
                      lambda x: sm.video.setThresh('low_red', x) )
    cv2.createTrackbar('high_red', 'sliders', sm.video.thresholds['high_red'], 255,
                     lambda x: sm.video.setThresh('high_red', x) )
    cv2.createTrackbar('low_green', 'sliders', sm.video.thresholds['low_green'], 255,
                      lambda x: sm.video.setThresh('low_green', x) )
    cv2.createTrackbar('high_green', 'sliders', sm.video.thresholds['high_green'], 255,
                     lambda x: sm.video.setThresh('high_green', x) )
    cv2.createTrackbar('low_blue', 'sliders', sm.video.thresholds['low_blue'], 255,
                      lambda x: sm.video.setThresh('low_blue', x) )
    cv2.createTrackbar('high_blue', 'sliders', sm.video.thresholds['high_blue'], 255,
                     lambda x: sm.video.setThresh('high_blue', x) )


    while len(sm.video.latestImg) == 0:
        sleep(1)

    while(sm.RUNNING):
        with imageLock:
            cv2.imshow("Create View",sm.video.latestImg)
            cv2.imshow("Create View2",sm.video.feedback)
        cv2.waitKey(5)

    cv2.destroyAllWindows()

    sleep(1)

#    sm.join()
