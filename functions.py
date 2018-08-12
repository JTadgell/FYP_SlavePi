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


def PID():
       
	gv.e_angle_sum = gv.e_angle_sum + gv.e_angle
	motorspeed = (gv.Kp * gv.e_angle) + gv.Kd * (gv.e_angle - gv.e_angle_prev) + gv.Ki * (gv.e_angle_sum)
	gv.e_angle_sum = gv.e_angle_sum + gv.e_angle
	gv.e_angle_prev = gv.e_angle

	gv.rightMotorSpeed = 300 - motorspeed
	gv.leftMotorSpeed = 300 + motorspeed 
	if (gv.rightMotorSpeed>500):
	    gv.rightMotorSpeed=500
	elif (gv.rightMotorSpeed<100):
	    gv.rightMotorSpeed=100
	if (gv.leftMotorSpeed>500):
	    gv.leftMotorSpeed=500
	elif (gv.leftMotorSpeed<100):
	    gv.leftMotorSpeed=100
	left=int(gv.leftMotorSpeed)
	right=int(gv.rightMotorSpeed)

	tempx=len(str(left))
	tempy=len(str(right))
	if tempx==1:
	    valx="000"+str(left)
	if tempx==2:
	    valx="00"+str(left)
	if tempx==3:
	    valx="0"+str(left)
	if tempx==4:
	    valx=str(left)
	if tempy==1:
	    valy="000"+str(right)
	if tempy==2:
	    valy="00"+str(right)
	if tempy==3:
	    valy="0"+str(right)
	if tempy==4: 
	    valy=str(right)
	gv.val = "1" + valx + valy


def capture(frame):
    success1=0
    gv.image = frame.array
    rotmatrix = cv2.getRotationMatrix2D((640,0),-0.2,1)
    gv.image = cv2.warpAffine(gv.image,rotmatrix,(640,480))
    #start  calculations
    b,g,r = cv2.split(gv.image)
    #cv2.imshow("Red", r)
    #blurred = cv2.blur(r, (2, 2))
    #blurred = cv2.blur(blurred, (3, 3))
    #blurred = cv2.blur(blurred, (5, 5))
    
    #cv2.imshow("Blur", blurred)
    max_intensity=np.amax(r)
    min_intensity=np.amin(r)
    thresh_value, gv.thresh = cv2.threshold(r,max_intensity-(max_intensity-min_intensity)/2, 255, cv2.THRESH_BINARY)
    cnts = cv2.findContours(gv.thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0]


    # cnts should run for every dot detected
    # will highlight where the dots are

    for c in cnts:   
        if cv2.contourArea(c) > 25:
                cArea = cv2.contourArea(c)
                gv.dots = gv.dots + 1
        else:
            continue
                
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)

        max_x = int(max(box[0][0],box[1][0],box[2][0],box[3][0]) + 10)
        if max_x > 640:
            max_x = 640
        min_x = int(min(box[0][0],box[1][0],box[2][0],box[3][0]) - 10)
        if min_x < 0:
            min_x = 0
        max_y = int(max(box[0][1],box[1][1],box[2][1],box[3][1]) + 10)
        if max_y > 480:
            max_y = 480
        min_y = int(min(box[0][1],box[1][1],box[2][1],box[3][1]) - 10)
        if min_y < 0:
            min_y = 0
            
        box_vals = gv.thresh[min_y:max_y,min_x:max_x]

        # Calculate the x centroid
        sum_x = box_vals.sum(axis=0);
        x_vals = np.arange(len(sum_x))
        xi = sum_x*x_vals
        sum_xi = np.sum(xi)
        sum_i = np.sum(sum_x)
        rel_centroid_x = sum_xi/sum_i
        xcoords = range(int(min_x),int(max_x)+1)
        centroid_x = xcoords[int(rel_centroid_x)]

        # Calculate the y centroid
        sum_y = box_vals.sum(axis=1);
        y_vals = np.arange(len(sum_y))
        yi = sum_y*y_vals #y*i
        sum_yi = np.sum(yi)
        rel_centroid_y = sum_yi/sum_i
        ycoords = range(int(min_y),int(max_y)+1)
        centroid_y = ycoords[int(rel_centroid_y)]


        if gv.dots<3:
            gv.laserdot[gv.dots-1][0] = centroid_x
            gv.laserdot[gv.dots-1][1] = centroid_y
        cv2.drawContours(gv.image, [box.astype("int")], -1, (0, 255, 0), 2)
        cv2.circle(gv.image, (centroid_x, centroid_y), 4, (255, 0, 0), -1)

    


    #OUT OF CNTLOOP
    if gv.dots == 2:
        if(gv.dots_prev!=2):
            print("Two dots successfully found")
        if gv.laserdot[0][0] < gv.laserdot[1][0]:
            gv.frontdot[0] = gv.laserdot[0][0]
            gv.frontdot[1] = gv.laserdot[0][1]
            gv.backdot[0] = gv.laserdot[1][0]
            gv.backdot[1] = gv.laserdot[1][1]
        else:
            gv.frontdot[0] = gv.laserdot[1][0]
            gv.frontdot[1] = gv.laserdot[1][1]
            gv.backdot[0] = gv.laserdot[0][0]
            gv.backdot[1] = gv.laserdot[0][1]
        if gv.debugger:
            cv2.putText(gv.image, "front dot", (gv.frontdot[0], gv.frontdot[1]),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
            cv2.putText(gv.image, "back dot", (gv.backdot[0],gv.backdot[1]),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
            print("frontdot:"+ str(gv.frontdot))
            print("backdot:"+ str(gv.backdot))
        # Convert the y coordinates to a coordinate system where the centre line is zero
        gv.frontdot[1] = gv.frontdot[1] - gv.yzero
        gv.backdot[1] = gv.backdot[1] - gv.yzero

        # Convert the x coordinates to a coordinate system where no-man's land is zero
        gv.frontdot[0] = gv.frontdot[0] - gv.xzero
        gv.backdot[0] = gv.backdot[0] - gv.xzero
        
            
        if (gv.frontdot[0] > 0) or (gv.backdot[0] < 0):
            if(gv.dots_prev!=2):
                print("Interference, dots found in the same sector")
              
        else:
            # If valid dots have been found, set valid dots to tru
            gv.validdots = True
            
            # Find the differences in x and y directions for each dot
            gv.xdiff = gv.backdot[0] - gv.frontdot[0]
            gv.ydiff = gv.backdot[1] - gv.frontdot[1]

            # Calculate the current angle to the laser line
            gv.c_angle = round(math.degrees(math.acos(gv.xdiff/math.sqrt((gv.xdiff**2) + (gv.ydiff**2)))))
            if gv.ydiff<=0:
                gv.m=abs(gv.ydiff)/gv.xdiff
                gv.m=-gv.m
            else:
                gv.m = gv.ydiff/gv.xdiff    
            # Calculate the orthogonal distance to the laser line from the axle
             # Calculate the gradient between the lines
            gv.c = gv.backdot[1] - gv.m*gv.backdot[0] # Calculate the offset of the lines from the centreline
            gv.orth_dist = gv.m*(gv.axle_pos) + gv.c
            gv.orth_dist = round(gv.orth_dist)
            
            #print (gv.orth_dist)
            gv.t_angle = round(math.degrees(math.atan(-gv.orth_dist/gv.dr)))

            if gv.ydiff > 0:
                gv.c_angle = -1*gv.c_angle #Angle is negative if yf<yb

            
            gv.e_angle = gv.t_angle - gv.c_angle

            success1 = 1

    else:

        gv.ser.write("2")
        #np.append(log, "reset")
        gv.file.write("reset\n")
        print("didn't find 2 dots")
        success1 = 0
               
    if gv.debugger:
        draw_lines(0)     

    gv.dots_prev = gv.dots
    gv.dots = 0
    
    return success1

def send_result():
    gv.ser.write(gv.val)


def write_data():
    gv.file.write(str(gv.e_angle)+'|'+str(gv.leftMotorSpeed)+'|'+str(gv.rightMotorSpeed)+'\n')



def draw_lines(display):

    gv.end = time.time()

    if gv.dots == 2:
        print("xdiff:" +str(gv.xdiff)+ "   ydiff:" +str(gv.ydiff)+ "   m:"+ str(gv.m) +"   c:"+str(gv.c))
        print ("Orthogonal distance (pixels): %d"%gv.orth_dist)
        print ("Current angle: %3d"%gv.c_angle)
        print ("Target angle: %4d"%gv.t_angle)
        print ("Error angle: %5d"%gv.e_angle)
    print("Time taken: "+ str(gv.end - gv.start))

    if display == 1:    
        cv2.circle(gv.image, (int(gv.xzero + gv.axle_pos), int(gv.yzero + gv.orth_dist)), 4, (255, 255, 0), -1)
        cv2.drawContours(gv.image, [gv.axle_line.astype("int")], -1, (0, 255, 0), 1)
        cv2.drawContours(gv.image, [gv.centre_line_x.astype("int")], -1, (0, 255, 255), 1)
        cv2.drawContours(gv.image, [gv.centre_line_y.astype("int")], -1, (0, 255, 255), 1)
        cv2.putText(gv.image, "(x) ->", (gv.xzero+3,gv.yzero-3),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
        cv2.putText(gv.image, "(y)\/", (gv.xzero+3,gv.yzero+15),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
        cv2.imshow("Thresh", gv.thresh)
        cv2.imshow("Frame", gv.image)