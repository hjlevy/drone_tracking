# code to analyze x y distance to yellow target from downloaded video
# uses image processing to create bounding box on yellow feature
# plotting technique can't be used real time
# can use this to calibrate the yellow for an experiment

import numpy as np
import cv2
import os
import time
import csv
import os.path
from os import path
import matplotlib
import tkinter
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd


def animate():
    data = pd.read_csv('distance.csv')
    xc = data['dx']
    yc = data['dy']

    ax.cla()
    ax.plot(xc,yc,'.')
    ax.set_xlim(-1,1)
    ax.set_ylim(-2,4)
    ax.set_xlabel('Change in X')
    ax.set_ylabel('Change in Y')


def nothing(x):
    pass

# initializing plot
fig, ax = plt.subplots()
ax.set_xlabel('Change in X')
ax.set_ylabel('Change in Y')
ax.set_xlim(-2,4)
ax.set_ylim(-2,4)

plt.ion()
plt.show()


os.chdir('videos') 

# deleting data from before
filename = "distance.csv"
if path.exists(filename):
    f = open(filename,"w+")
    f.close()

# creating file
fieldnames = ["dx","dy"]
with open('distance.csv','w') as newFile:
    writer = csv.DictWriter(newFile, fieldnames=fieldnames)
    writer.writeheader()

cap = cv2.VideoCapture('yellow_test2.mp4')

if (cap.isOpened()==False):
    print("Error opening video stream or file")

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 24, 180, nothing)
cv2.createTrackbar("L-S", "Trackbars", 51, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 122, 255, nothing)
cv2.createTrackbar("U-H", "Trackbars", 90, 180, nothing)
cv2.createTrackbar("U-S", "Trackbars", 182, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)
cnt_found = False

while(cap.isOpened()):
    ret, frame = cap.read()
    
    width = cap.get(3)
    height = cap.get(4)

    # used to slow down to set trackbars
    time.sleep(0.1) 

    #using hsv color space to detect colors
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L-H","Trackbars")
    l_s = cv2.getTrackbarPos("L-S","Trackbars")
    l_v = cv2.getTrackbarPos("L-V","Trackbars")
    u_h = cv2.getTrackbarPos("U-H","Trackbars")
    u_s = cv2.getTrackbarPos("U-S","Trackbars")
    u_v = cv2.getTrackbarPos("U-V","Trackbars")

    lower_yellow = np.array([l_h, l_s, l_v])
    upper_yellow = np.array([u_h, u_s, u_v])

    # lower_yellow = np.array([24,51,122])
    # upper_yellow = np.array([90,182,255])

    mask = cv2.inRange(hsv, lower_yellow,upper_yellow)
    # removing mask noise
    kernel= np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel)

    #contours detection
    if int(cv2.__version__[0]) > 3:
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    else:
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    wmax = 0
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        # cv2.drawContours(frame, [cnt], 0, (0, 0, 0), 5)

        # wamt to only analyze largest contour
        if w>wmax:
            wmax = w

            # want to save center coordinate of bounding box
            x_c = x+w/2
            y_c = y+h/2

            # calibration of real distances
            calx = 0.295/w
            caly = 0.23/h

            # center of camera pos
            x_cam_c = width/2
            y_cam_c = height/2
            dx = (x_c - x_cam_c)*calx
            dy = -(y_c - y_cam_c)*caly


        cnt_found = True 

    if cnt_found : 
        with open('distance.csv', 'a') as newFile:
            writer = csv.DictWriter(newFile,fieldnames = fieldnames)
            info = {
                "dx": dx,
                "dy": dy
            }
            writer.writerow(info)  
        animate()

    cv2.namedWindow("Frame",cv2.WINDOW_NORMAL)
    cv2.imshow("Frame", frame)
    # fig.canvas.draw()
    cv2.namedWindow("Mask",cv2.WINDOW_NORMAL)
    cv2.imshow("Mask", mask)

    # if esc key is pressed leave loop 
    if cv2.waitKey(1) == 27:
        break


cap.release()
cv2.destroyAllWindows()




