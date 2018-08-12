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


# set up the serial connection
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate = 57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1)

# open up a file to store the logging data
file = open("/home/pi/test/data.csv","w").close()      
file = open("/home/pi/test/data.csv","a")

e_angle_prev=0
e_angle_sum=0

def values(x,y):
    tempx=len(str(x))
    tempy=len(str(y))
    if tempx==1:
        valx="000"+str(x)
    if tempx==2:
        valx="00"+str(x)
    if tempx==3:
        valx="0"+str(x)
    if tempx==4:
        valx=str(x)
    if tempy==1:
        valy="000"+str(y)
    if tempy==2:
        valy="00"+str(y)
    if tempy==3:
        valy="0"+str(y)
    if tempy==4:
        valy=str(y)
    val="1"+valx+valy
    print val
    ser.write(val)
    return 0

def PID(e_angle,Kp,Kd,Ki):
        global e_angle_prev
        global e_angle_sum
        
        e_angle_sum = e_angle_sum+e_angle
        motorspeed=(Kp* e_angle) +Kd*(e_angle-e_angle_prev)+Ki*(e_angle_sum)
        e_angle_prev=e_angle

        rightMotorSpeed = 300 - motorspeed
        leftMotorSpeed = 300 + motorspeed 
        if (rightMotorSpeed>500):
            rightMotorSpeed=500
        elif (rightMotorSpeed<100):
            rightMotorSpeed=100
        if (leftMotorSpeed>500):
            leftMotorSpeed=500
        elif (leftMotorSpeed<100):
            leftMotorSpeed=100
        left=int(leftMotorSpeed)
        right=int(rightMotorSpeed)
        #print"left motorspeed %i ---- right motorspeed %i ---- error angle %f" % (left,right,e_angle)
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
        val="1"+valx+valy
        print val
        ser.write(val)
        file.write(str(e_angle)+'|'+str(leftMotorSpeed)+'|'+str(rightMotorSpeed)+'\n')
       

#Beginnning of Computer Vision 

try:
    camera = PiCamera()
    camera.resolution = (640, 480)
    
    camera.framerate = 90
    
    camera.exposure_mode = "off"
    camera.awb_mode = "off"
    camera.awb_gains = [0.9,0]
    camera.brightness = 50
    camera.saturation = 100
    camera.contrast = 0
    
    rawCapture = PiRGBArray(camera, size=(640, 480))
    print("Camera initialised")
except:
    print("Error: could not initialise camera")
    
debugger = False    
    
dr = 200 # mm, distance along the laser line to aim for
axle_pos = -160 # pixels, axle position in pixels from centrepoint
pixpermm = 4 # 1mm = 4 pixels
dr = dr * pixpermm
for i in range(-3,5):
    start_global = time.time()
    counter=0
    Kp=1*10**i
    Kd=0
    Ki=0
    file.write("\n"+str(Kp)+" | "+str(Kd)+" | "+str(Kd)+"\n")
    # initialise the storage vector for the dots
    laserdot = np.array([[0,0],[0,0]])
    backdot = [0,0]
    frontdot = [0,0]
    validdots = False

    # Coordinate system
    xzero = 320
    yzero = 200

    # Debug lines
    axle_line = np.array([[xzero + axle_pos , 0],[xzero + axle_pos, 480]])
    centre_line_x = np.array([[xzero, 0],[xzero, 480]])
    centre_line_y = np.array([[0, yzero],[640, yzero]])

    # allow camera to warm up
    time.sleep(0.1)

    cXs = [0, 0]
    cYs = [0, 0]
    dots = 0
    dots_prev = 0
    centroid_x_prev = 0
    centroid_y_prev = 0



    start = time.time()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        rotmatrix = cv2.getRotationMatrix2D((640,0),-0.2,1)
        image = cv2.warpAffine(image,rotmatrix,(640,480))
        #start  calculations
        b,g,r = cv2.split(image)
        #cv2.imshow("Red", r)
        #blurred = cv2.blur(r, (2, 2))
        #blurred = cv2.blur(blurred, (3, 3))
        #blurred = cv2.blur(blurred, (5, 5))
        
        #cv2.imshow("Blur", blurred)
        max_intensity=np.amax(r)
        min_intensity=np.amin(r)
        thresh_value,thresh = cv2.threshold(r,max_intensity-(max_intensity-min_intensity)/2, 255, cv2.THRESH_BINARY)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        for c in cnts:
            
            if cv2.contourArea(c) > 25:
                    cArea = cv2.contourArea(c)
                    dots = dots + 1
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
                
            box_vals = thresh[min_y:max_y,min_x:max_x]

            # Calculate the x centroid
            sum_x = box_vals.sum(axis=0); #sum all the columns along x
            x_vals = np.arange(len(sum_x)) #x index of array
            xi = sum_x*x_vals #x*i
            sum_xi = np.sum(xi)
            sum_i = np.sum(sum_x) #sum intensities
            rel_centroid_x = sum_xi/sum_i #Calculate the centroid relative to the box
            xcoords = range(int(min_x),int(max_x)+1) #Translate the box coordinates to the frame

            centroid_x = xcoords[int(rel_centroid_x)] #Find the centroid relative to the frame

            # Calculate the y centroid
            sum_y = box_vals.sum(axis=1); #sum all the rows along y
            y_vals = np.arange(len(sum_y)) #y index of array
            yi = sum_y*y_vals #y*i
            sum_yi = np.sum(yi)
            rel_centroid_y = sum_yi/sum_i
            ycoords = range(int(min_y),int(max_y)+1)

            centroid_y = ycoords[int(rel_centroid_y)]

            if dots<3:
                laserdot[dots-1][0] = centroid_x
                laserdot[dots-1][1] = centroid_y
            cv2.drawContours(image, [box.astype("int")], -1, (0, 255, 0), 2)
            cv2.circle(image, (centroid_x, centroid_y), 4, (255, 0, 0), -1)
        #OUT OF CNTLOOP
        if dots == 2:
            if(dots_prev!=2):
                print("Two dots successfully found")
            if laserdot[0][0] < laserdot[1][0]:
                frontdot[0] = laserdot[0][0]
                frontdot[1] = laserdot[0][1]
                backdot[0] = laserdot[1][0]
                backdot[1] = laserdot[1][1]
            else:
                frontdot[0] = laserdot[1][0]
                frontdot[1] = laserdot[1][1]
                backdot[0] = laserdot[0][0]
                backdot[1] = laserdot[0][1]
            if debugger:
                cv2.putText(image, "front dot", (frontdot[0],frontdot[1]),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
                cv2.putText(image, "back dot", (backdot[0],backdot[1]),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
                print("frontdot:"+ str(frontdot))
                print("backdot:"+ str(backdot))
            # Convert the y coordinates to a coordinate system where the centre line is zero
            frontdot[1] = frontdot[1] - yzero
            backdot[1] = backdot[1] - yzero

            # Convert the x coordinates to a coordinate system where no-man's land is zero
            frontdot[0] = frontdot[0] - xzero
            backdot[0] = backdot[0] - xzero
            
                

            if (frontdot[0] > 0) or (backdot[0] < 0):
                if(dots_prev!=2):
                    print("Interference, dots found in the same sector")
                  
            else:
                # If valid dots have been found, set valid dots to tru
                validdots = True
                
                # Find the differences in x and y directions for each dot
                xdiff = backdot[0] - frontdot[0]
                ydiff = backdot[1] - frontdot[1]

                # Calculate the current angle to the laser line
                c_angle = round(math.degrees(math.acos(xdiff/math.sqrt((xdiff**2) + (ydiff**2)))))
                if ydiff<=0:
                    m=abs(ydiff)/xdiff
                    m=-m
                else:
                    m = ydiff/xdiff    
                # Calculate the orthogonal distance to the laser line from the axle
                 # Calculate the gradient between the lines
                c = backdot[1] - m*backdot[0] # Calculate the offset of the lines from the centreline
                orth_dist = m*(axle_pos) + c
                orth_dist = round(orth_dist)
                
                #print (orth_dist)
                t_angle = round(math.degrees(math.atan(-orth_dist/dr)))

                if ydiff > 0:
                    c_angle = -1*c_angle #Angle is negative if yf<yb

                
                e_angle = t_angle - c_angle
                PID(e_angle,Kp,Kd,Ki)
                if debugger:
                    print("xdiff:" +str(xdiff)+ "   ydiff:" +str(ydiff)+ "   m:"+ str(m) +"   c:"+str(c))
                    cv2.circle(image, (int(xzero + axle_pos), int(yzero + orth_dist)), 4, (255, 255, 0), -1)
                    print ("Orthogonal distance (pixels): %d"%orth_dist)
                    print ("Current angle: %3d"%c_angle)
                    print ("Target angle: %4d"%t_angle)
                    print ("Error angle: %5d"%e_angle)
                #PID(e_angle)
        else:
            ser.write("2")
    	    file.write("reset\n")
                
        dots = 0
            
        if debugger:
            cv2.drawContours(image, [axle_line.astype("int")], -1, (0, 255, 0), 1)
            cv2.drawContours(image, [centre_line_x.astype("int")], -1, (0, 255, 255), 1)
            cv2.drawContours(image, [centre_line_y.astype("int")], -1, (0, 255, 255), 1)
            cv2.putText(image, "(x) ->", (xzero+3,yzero-3),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
            cv2.putText(image, "(y)\/", (xzero+3,yzero+15),cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
            cv2.imshow("Thresh", thresh)
            cv2.imshow("Frame", image)
            end=time.time()
            print(end-start)
            start=end
        end=time.time()
        print(end-start)
        start=end
        counter=counter+1
        print counter
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
    	   break
        rawCapture.truncate(0)
        if counter==140:
           print(time.time()-start_global)
           break
    ser.write("2")
    time.sleep(5)
    camera.close
