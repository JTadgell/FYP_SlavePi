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

from functions import PID, capture, write_data, draw_lines, send_result
from setup import setup

import global_variable as gv


# should we use the debugger or log data?
gv.debugger = True
gv.logging = True

#=============================================================================
# STANDARD STARTUP LOOP
#
# startup initial k values, serial connection, camera + rawcapture, logging array
#
#=============================================================================

setup()

# setup the camera with the required settings




#=============================================================================
# MAIN FOR LOOP
#
# cycle though -> check error (this uses the camera)
#                   -> update PID?
#              -> apply PID
#              -> send new values
# 
#
#=============================================================================

time.sleep(0.1)


for i in range(-3,5):

    gv.counter = 0
    gv.Kp = 10**i
    gv.Kd = 0
    gv.Ki = 0

    gv.file.write("\n"+str(gv.Kp)+" | "+str(gv.Kd)+" | "+str(gv.Kd)+"\n")

    gv.start = time.time()

    for frame in gv.camera.capture_continuous(gv.rawCapture, format="bgr", use_video_port=True):

        gv.counter = gv.counter + 1
        success = capture(frame)

        if success:
            PID()
            send_result()
        gv.end = time.time()

        if gv.debugging:
            draw_lines()
        if gv.logging:
            write_data()

        gv.start = gv.end

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        gv.rawCapture.truncate(0)
        if gv.counter==140:
            break

        
    #np.savetxt("data.csv", log, delimiter=",")
    gv.ser.write("2")
    time.sleep(5)
    camera.close
