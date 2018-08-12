# import the necessary packages
from scipy.spatial import distance as dist
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils import perspective
from imutils import contours
import time
import cv2
import imutils
import numpy as np
import math
import serial
import os
import getopt
import sys
from socket import *
from socket import error as socket_error

import global_variable as gv



def set_cam():
	try:
	    gv.camera = PiCamera()
	    gv.camera.resolution = (640, 480)
	    
	    gv.camera.framerate = 90
	    
	    gv.camera.exposure_mode = "off"
	    gv.camera.awb_mode = "off"
	    gv.camera.awb_gains = [0.9,0]
	    gv.camera.brightness = 50
	    gv.camera.saturation = 100
	    gv.camera.contrast = 0
	    
	    gv.rawCapture = PiRGBArray(gv.camera, size=(640, 480))
	    print("Camera initialised")
	except:
	    print("Error: could not initialise camera")


def set_cam_vars():

    #log = np.append(log, str(Kp)+" | "+str(Kd)+" | "+str(Kd)+"\n")

    # initialise the storage vector for the dots
    gv.laserdot = np.array([[0,0],[0,0]])
    gv.backdot = [0,0]
    gv.frontdot = [0,0]
    gv.validdots = False

    # Coordinate system
    gv.xzero = 320
    gv.yzero = 200

    # Debug lines
    gv.axle_line = np.array([[gv.xzero + gv.axle_pos , 0],[gv.xzero + gv.axle_pos, 480]])
    gv.centre_line_x = np.array([[gv.xzero, 0],[gv.xzero, 480]])
    gv.centre_line_y = np.array([[0, gv.yzero],[640, gv.yzero]])

    # allow camera to warm up
    gv.cXs = [0, 0]
    gv.cYs = [0, 0]
    gv.dots = 0
    gv.dots_prev = 0
    gv.centroid_x_prev = 0
    gv.centroid_y_prev = 0

def set_vars():

	gv.e_angle_prev=0
	gv.e_angle_sum=0 

	gv.dr = 200
	gv.axle_pos = -160
	gv.pixpermm = 4
	gv.dr = gv.dr * gv.pixpermm


def k_values():

	a = 0

def set_ser():
	# setupt the serial connection
	gv.ser = serial.Serial(
	    port='/dev/ttyACM0',
	    baudrate = 57600,
	    parity=serial.PARITY_NONE,
	    stopbits=serial.STOPBITS_ONE,
	    bytesize=serial.EIGHTBITS,
	    timeout=1)
	
	print("serial: "+str(gv.ser.name))

def set_log():
	gv.file = open("/home/pi/live/data/data.csv","w").close()      
	gv.file = open("/home/pi/live/data/data.csv","a")

def setup():
	set_cam()
	set_ser()
	k_values()
	set_vars()
	set_cam_vars()
	set_log()



