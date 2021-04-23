# code to analyze x y distance to yellow target from downloaded video
# want to improve contour technique by finding centroid of the blob of white pixels
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

def find_farthest_white(img, target):
    nonzero = cv2.findNonZero(img)
    distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
    dist_x = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 )
    dist_y = np.sqrt((nonzero[:,:,1] - target[1]) ** 2 )
    x_max = np.amax(dist_x)
    y_max = np.amax(dist_y)
    #farthest_index = np.argmax(distances)
    # farthest_dist = np.amax(distances)
    #return nonzero[farthest_index]
    return [x_max,y_max]

def find_farthest_white_circ(img, target):
    nonzero = cv2.findNonZero(img)
    distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
    #farthest_index = np.argmax(distances)
    #return nonzero[farthest_index]
    farthest_dist = np.amax(distances)
    return farthest_dist
    

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

# cv2.namedWindow("Trackbars")
# cv2.createTrackbar("L-H", "Trackbars", 24, 180, nothing)
# cv2.createTrackbar("L-S", "Trackbars", 51, 255, nothing)
# cv2.createTrackbar("L-V", "Trackbars", 122, 255, nothing)
# cv2.createTrackbar("U-H", "Trackbars", 90, 180, nothing)
# cv2.createTrackbar("U-S", "Trackbars", 182, 255, nothing)
# cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)
cnt_found = False

while(cap.isOpened()):
    ret, frame = cap.read()
    
    width = cap.get(3)
    height = cap.get(4)

    # used to slow down to set trackbars
    time.sleep(0.1) 

    #using hsv color space to detect colors
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # l_h = cv2.getTrackbarPos("L-H","Trackbars")
    # l_s = cv2.getTrackbarPos("L-S","Trackbars")
    # l_v = cv2.getTrackbarPos("L-V","Trackbars")
    # u_h = cv2.getTrackbarPos("U-H","Trackbars")
    # u_s = cv2.getTrackbarPos("U-S","Trackbars")
    # u_v = cv2.getTrackbarPos("U-V","Trackbars")

    # lower_yellow = np.array([l_h, l_s, l_v])
    # upper_yellow = np.array([u_h, u_s, u_v])

    lower_yellow = np.array([24,51,122])
    upper_yellow = np.array([90,182,255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # removing mask noise
    kernel= np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel)

    #creating binary image using cv2 threshold
    ret, thresh = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)

    # print(np.sum(thresh))
    #only plot the centroid when enough white pixels detected in frame -> object there
    if np.sum(thresh)>100000:
        # calculate moments of binary image
        M = cv2.moments(thresh)

        # calculate x,y coordinate of center
        M00 =  M["m00"]
        if M00 == 0:
            M00 = 0.001

        cX = int(M["m10"] / M00)
        cY = int(M["m01"] / M00)

        # plotting center of blob
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # # plotting circle defined by farthest white point from centroid
        # r = find_farthest_white_circ(thresh, (cX,cY))

        #finding dimensions of bounding rectangle by finding farthest white point from center in x and y
        [xr, yr] = find_farthest_white(thresh, (cX,cY))
        xr = int(xr)
        yr = int(yr)

        # only plot if the xr and yr isn't bigger than the entire frame
        if xr < width/2 and yr< height/2:
            # # plotting circle 
            # cv2.circle(frame, (cX, cY), int(r), (0, 0, 255), 2)

            # plotting rectangle
            cv2.rectangle(frame, (cX-xr, cY-yr), (cX+xr, cY+yr), (0,0,255), 2)

            # calibration of real distances
            calx = 0.295/(2*xr)
            caly = 0.23/(2*yr)

            # center of camera pos
            x_cam_c = width/2
            y_cam_c = height/2
            dx = (cX - x_cam_c)*calx
            dy = -(cY - y_cam_c)*caly

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
    fig.canvas.draw()
    cv2.namedWindow("Mask",cv2.WINDOW_NORMAL)
    cv2.imshow("Mask", mask)

    # if esc key is pressed leave loop 
    if cv2.waitKey(1) == 27:
        break


cap.release()
cv2.destroyAllWindows()




