# code to analyze x y z distance to red target from downloaded video
# uses image processing to create bounding box on red feature

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

# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')

def live_plotter(x_vec,y1_data, z_data, line1, identifier='',pause_time=0.1):
    if line1==[]:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111,projection='3d')
        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec,y1_data,z_data,'.',alpha=0.8)        
        #update plot label/title
        plt.ylabel('dy')
        plt.xlabel('dx')
        ax.set_zlabel('dz')
        ax.set_zlim(-1,1)
        plt.title('Title: {}'.format(identifier))
        plt.show()
    
    # after the figure, axis, and line are created, we only need to update the y-data
    
    line1.set_data(x_vec,y1_data)
    line1.set_3d_properties(z_data,'z')
    # adjust limits if new data goes beyond bounds
    # if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
    #     plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
    
    plt.ylim([-1,1])
    plt.xlim([-1,1])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)
    
    # return line so we can update it again in the next iteration
    return line1


def nothing(x):
    pass

def depth_calc(w, dpx, W, p):
    # w = width of target in pixels, W = width of camera frame in pixels, p is pixel to meter conversion factor
    cam_angle = 110*np.pi/180
    # alpha = 2*np.arctan(w/W*np.tan(cam_angle/2))
    # L = w*p/(2*np.tan(alpha/2))
    w = p*w #width of target in meters
    beta = np.arctan(2*(w/2+dpx)/W*np.tan(cam_angle/2))
    L = (w/2+dpx)*p/np.tan(beta)
    return L


os.chdir('drone_tracking/videos')

# deleting data from before
filename = "distance.csv"
if path.exists(filename):
    f = open(filename,"w+")
    f.close()

# creating file
fieldnames = ["dx","dy","dz"]
with open('distance.csv','w') as newFile:
    writer = csv.DictWriter(newFile, fieldnames=fieldnames)
    writer.writeheader()

cap = cv2.VideoCapture('red_highup.mp4')

if (cap.isOpened()==False):
    print("Error opening video stream or file")

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 125, 180, nothing)
cv2.createTrackbar("L-S", "Trackbars", 85, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 94, 255, nothing)
cv2.createTrackbar("U-H", "Trackbars", 180, 180, nothing)
cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)
cnt_found = False

line1 = []
while(cap.isOpened()):
    ret, frame = cap.read()
    
    width = cap.get(3)
    height = cap.get(4)

    # used to slow down to set trackbars
    # time.sleep(0.1) 

    #using hsv color space to detect colors
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L-H","Trackbars")
    l_s = cv2.getTrackbarPos("L-S","Trackbars")
    l_v = cv2.getTrackbarPos("L-V","Trackbars")
    u_h = cv2.getTrackbarPos("U-H","Trackbars")
    u_s = cv2.getTrackbarPos("U-S","Trackbars")
    u_v = cv2.getTrackbarPos("U-V","Trackbars")

    lower_red = np.array([l_h, l_s, l_v])
    upper_red = np.array([u_h, u_s, u_v])

    # lower_red = np.array([125,85,95])
    # upper_red = np.array([180,255,255])

    mask = cv2.inRange(hsv, lower_red,upper_red)
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
            dz = depth_calc(w,dx/calx,width,calx)


        cnt_found = True 

    if cnt_found : 
        with open('distance.csv', 'a') as newFile:
            writer = csv.DictWriter(newFile,fieldnames = fieldnames)
            info = {
                "dx": dx,
                "dy": dy,
                "dz": dz
            }
            writer.writerow(info) 

        data = pd.read_csv('distance.csv')
        xc = data['dx']
        yc = data['dy']
        zc = data['dz']
        # line1 = live_plotter(xc,yc,zc,line1)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    # if esc key is pressed leave loop 
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()




