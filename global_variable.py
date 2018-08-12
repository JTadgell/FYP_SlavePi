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

e_angle = 0
e_angle_prev = 0
e_angle_sum = 0

image = 0

c_angle = 0
t_angle = 0

c = 0
m = 0

orth_dist = 0


kv = 0
ki = 0
kp = 0

dr	= 0 		# distance along the laser line to aim for (mm)
axle_pos = 0	# axle position in pixels from centrepoint
pixpermm = 0	# mm to pixels


laserdot = 0
backdot = 0
frontdot = 0
validdots = 0

xzero = 0
yzero = 0

axle_line = 0
centre_line_x = 0
centre_line_y = 0

cXs = 0
cYs = 0
dots = 0
dots_prev = 0

#controid_x     	- local in functions now
centroid_x_prev = 0
#controid_y			- local in functions now
centroid_y_prev = 0

camera = 0
rawCapture = 0
ser = 0
file = 0

debugger = 0
logging = 0

leftMotorSpeed = 0
rightMotorSpeed = 0

xdiff = 0
ydiff = 0

thresh = 0

end = 0
start = 0