# frame target analysis of distance to yellow target
# use keyboard keys to navigate to the target
# will plot the distance to target and show a live stream of drone flying
# see QWERTY_CONTROL_KEYS for keyboard mapping system
# Press "esc" key when you want to stop streaming
# ADDED: Optimized contour function

import csv
import cv2
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import traceback
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import os.path
from os import path

import time
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxAltitude, MaxTilt, MaxDistance, NoFlyOverMaxDistance
from olympe.messages.ardrone3.SpeedSettings import Outdoor
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages import gimbal

# for keyboard
from pynput.keyboard import Listener, Key, KeyCode
from collections import defaultdict
from enum import Enum

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = "192.168.42.1"
# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')

# definining keyboard controls
class Ctrl(Enum):
    (
        QUIT,
        TAKEOFF,
        LANDING,
        MOVE_LEFT,
        MOVE_RIGHT,
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_UP,
        MOVE_DOWN,
        TURN_LEFT,
        TURN_RIGHT,
    ) = range(11)


QWERTY_CTRL_KEYS = {
    Ctrl.QUIT: Key.esc,
    Ctrl.TAKEOFF: "t",
    Ctrl.LANDING: "l",
    Ctrl.MOVE_LEFT: "h",
    Ctrl.MOVE_RIGHT: "k",
    Ctrl.MOVE_FORWARD: "u",
    Ctrl.MOVE_BACKWARD: "j",
    Ctrl.MOVE_UP: Key.up,
    Ctrl.MOVE_DOWN: Key.down,
    Ctrl.TURN_LEFT: Key.left,
    Ctrl.TURN_RIGHT: Key.right,
}

AZERTY_CTRL_KEYS = QWERTY_CTRL_KEYS.copy()
AZERTY_CTRL_KEYS.update(
    {
        Ctrl.MOVE_LEFT: "q",
        Ctrl.MOVE_RIGHT: "d",
        Ctrl.MOVE_FORWARD: "z",
        Ctrl.MOVE_BACKWARD: "s",
    }
)

## KEYBOARD CLASS
class KeyboardCtrl(Listener):
    def __init__(self, ctrl_keys=None):
        self._ctrl_keys = self._get_ctrl_keys(ctrl_keys)
        self._key_pressed = defaultdict(lambda: False)
        self._last_action_ts = defaultdict(lambda: 0.0)
        super().__init__(on_press=self._on_press, on_release=self._on_release)
        self.start()

    def _on_press(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = True
        elif isinstance(key, Key):
            self._key_pressed[key] = True
        if self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]:
            return False
        else:
            return True

    def _on_release(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = False
        elif isinstance(key, Key):
            self._key_pressed[key] = False
        return True

    def quit(self):
        return not self.running or self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]

    def _axis(self, left_key, right_key):
        return 100 * (
            int(self._key_pressed[right_key]) - int(self._key_pressed[left_key])
        )

    def roll(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_LEFT],
            self._ctrl_keys[Ctrl.MOVE_RIGHT]
        )

    def pitch(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_BACKWARD],
            self._ctrl_keys[Ctrl.MOVE_FORWARD]
        )

    def yaw(self):
        return self._axis(
            self._ctrl_keys[Ctrl.TURN_LEFT],
            self._ctrl_keys[Ctrl.TURN_RIGHT]
        )

    def throttle(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_DOWN],
            self._ctrl_keys[Ctrl.MOVE_UP]
        )

    def has_piloting_cmd(self):
        return (
            bool(self.roll())
            or bool(self.pitch())
            or bool(self.yaw())
            or bool(self.throttle())
        )

    def _rate_limit_cmd(self, ctrl, delay):
        now = time.time()
        if self._last_action_ts[ctrl] > (now - delay):
            return False
        elif self._key_pressed[self._ctrl_keys[ctrl]]:
            self._last_action_ts[ctrl] = now
            return True
        else:
            return False

    def takeoff(self):
        return self._rate_limit_cmd(Ctrl.TAKEOFF, 2.0)

    def landing(self):
        return self._rate_limit_cmd(Ctrl.LANDING, 2.0)

    def _get_ctrl_keys(self, ctrl_keys):
        # Get the default ctrl keys based on the current keyboard layout:
        if ctrl_keys is None:
            ctrl_keys = QWERTY_CTRL_KEYS
            try:
                # Olympe currently only support Linux
                # and the following only works on *nix/X11...
                keyboard_variant = (
                    subprocess.check_output(
                        "setxkbmap -query | grep 'variant:'|"
                        "cut -d ':' -f2 | tr -d ' '",
                        shell=True,
                    )
                    .decode()
                    .strip()
                )
            except subprocess.CalledProcessError:
                pass
            else:
                if keyboard_variant == "azerty":
                    ctrl_keys = AZERTY_CTRL_KEYS
        return ctrl_keys

# STREAMING CLASS 
class StreamingExample():

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)

        # max distance setting minimum is 10 meters
        self.drone(MaxDistance(10)).wait()
        self.drone(NoFlyOverMaxDistance(1)).wait()
        self.drone(Outdoor(0)).wait()

        # temporary directory
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        # self.h264_frame_stats = []
        # self.h264_stats_file = open(
        #     os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        # self.h264_stats_writer = csv.DictWriter(
        #     self.h264_stats_file, ['fps', 'bitrate'])
        # self.h264_stats_writer.writeheader()
        # self.frame_queue = queue.Queue()
        # self.flush_queue_lock = threading.Lock()
        self.line1 = []
        self.position = []

    
    def live_plotter(self,x_vec,y1_data, z_data, line1, identifier='',pause_time=0.1):
        if line1==[]:
            # this is the call to matplotlib that allows dynamic plotting
            plt.ion()
            fig = plt.figure(figsize=(13,6))
            ax = fig.add_subplot(111,projection='3d')
            # create a variable for the line so we can later update it
            self.line1, = ax.plot(x_vec,y1_data,z_data,'.',alpha=0.8)        
            #update plot label/title
            plt.ylabel('dy')
            plt.xlabel('dx')
            ax.set_zlabel('dz')
            ax.set_zlim(-1,1)
            plt.title('Title: {}'.format(identifier))
            plt.show()
    
        # after the figure, axis, and line are created, we only need to update the y-data
        
        self.line1.set_data(x_vec,y1_data)
        self.line1.set_3d_properties(z_data,'z')
        # adjust limits if new data goes beyond bounds
        # if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        #     plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
        
        plt.ylim([-1,1])
        plt.xlim([-1,1])
        # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
        plt.pause(pause_time)
        
        # return line so we can update it again in the next iteration
        return self.line1

    def start(self):
        # Connect the the drone
        self.drone.connect()

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.tempd, 'h264_data.264'),
            h264_meta_file=os.path.join(self.tempd, 'h264_metadata.json'),
            # Here, we don't record the (huge) raw YUV video stream
            # raw_data_file=os.path.join(self.tempd,'raw_data.bin'),
            # raw_meta_file=os.path.join(self.tempd,'raw_metadata.json'),
        )

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb
        )

        #point gimbal downwards
        cameraAction = self.drone(gimbal.set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="none",
            yaw=0.0,
            pitch_frame_of_reference="relative",
            pitch=-90,
            roll_frame_of_reference="none",
            roll=0.0,
        )).wait()
        time.sleep(5)

        #making sure gimbal moved
        if not cameraAction.success():
            assert False, "Cannot set gimbal position target"

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

        # Start video streaming
        self.drone.start_video_streaming()

    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnect()
        # self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        window_name = "Drone Tracking Target"
        # cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        start = time.time()
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        # converting from rgb to hsv
        hsv = cv2.cvtColor(cv2frame, cv2.COLOR_BGR2HSV)
        gray_image = cv2.cvtColor(cv2frame, cv2.COLOR_BGR2GRAY)
        
        # IF RED TARGET
        # lower_red = np.array([125,85,95])
        # upper_red = np.array([180,255,255])

        # # creating a mask red-> white, anything else ->black
        # mask = cv2.inRange(hsv, lower_red,upper_red)

        # IF YELLOW TARGET
        lower_yellow = np.array([24,51,122])
        upper_yellow = np.array([90,182,255])

        # creating a mask yellow-> white, anything else ->black
        mask = cv2.inRange(hsv, lower_yellow,upper_yellow)

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
            cv2.circle(cv2frame, (cX, cY), 5, (0, 0, 255), -1)

            # # circle radius defined by farthest white point from centroid
            # r = find_farthest_white_circ(thresh, (cX,cY))

            #finding dimensions of bounding rectangle by finding farthest white point from center in x and y
            [xr, yr] = self.find_farthest_white(thresh, (cX,cY))
            xr = int(xr)
            yr = int(yr)

            # only plot if the xr and yr isn't bigger than the entire frame
            if xr < width/2 and yr< height/2:
                # # plotting circle 
                # cv2.circle(frame, (cX, cY), int(r), (0, 0, 255), 2)

                # plotting rectangle
                cv2.rectangle(cv2frame, (cX-xr, cY-yr), (cX+xr, cY+yr), (0,0,255), 2)

                # calibration of real distances
                calx = 0.295/(2*xr)
                caly = 0.23/(2*yr)

                # center of camera pos
                x_cam_c = width/2
                y_cam_c = height/2
                dx = (cX - x_cam_c)*calx
                dy = -(cY - y_cam_c)*caly
                dz = self.depth_calc(2*xr,dx/calx,width,calx)

                cnt_found = True

        end = time.time()

        if cnt_found :    
            # seeing if center of bounding box getting updated with variable position
            self.position.append((cX,cY))

            # saving x y and z distances in a csv file
            with open('distance.csv', 'a') as newFile:
                writer = csv.DictWriter(newFile,fieldnames = ["dx","dy","dz"])
                info = {
                    "dx": dx,
                    "dy": dy,
                    "dz": dz
                }
                writer.writerow(info) 

            #plotting x,y,z distance to target
            data = pd.read_csv('distance.csv')
            xc = data['dx']
            yc = data['dy']
            zc = data['dz']
            self.line1 = self.live_plotter(xc,yc,zc,self.line1)
            # self.line1 = self.live_plotter(dx,dy,dz,self.line1)
            
        
        # print("FRAME TIMER VALUE: %f" % (end-start))
        # Use OpenCV to show this frame
        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, cv2frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...


    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """
        pass
    
    def find_farthest_white(self,img, target):
        nonzero = cv2.findNonZero(img)
        distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
        dist_x = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 )
        dist_y = np.sqrt((nonzero[:,:,1] - target[1]) ** 2 )
        x_max = np.amax(dist_x)
        y_max = np.amax(dist_y)
        #farthest_index = np.argmax(distances)
        #return nonzero[farthest_index]
        return [x_max,y_max]

    def depth_calc(self, w, dpx, W, p):
        # w = width of target in pixels, W = width of camera frame in pixels, p is pixel to meter conversion factor
        cam_angle = 110*np.pi/180
        # alpha = 2*np.arctan(w/W*np.tan(cam_angle/2))
        # L = w*p/(2*np.tan(alpha/2))
        w = p*w #width of target in meters
        beta = np.arctan(2*(w/2+dpx)/W*np.tan(cam_angle/2))
        L = (w/2+dpx)*p/np.tan(beta)
        return L

    def postprocessing(self):
        # Convert the raw .264 file into an .mp4 file
        h264_filepath = os.path.join(self.tempd, 'h264_data.264')
        mp4_filepath = os.path.join(self.tempd, 'h264_data.mp4')
        subprocess.run(
            shlex.split('ffmpeg -i {} -c:v copy -y {}'.format(
                h264_filepath, mp4_filepath)),
            check=True
        )

    def fly_by_keys(self):

        # quit keys by esc key
        # while esc not pressed, continue to read csv file to see where drone target is 
        control = KeyboardCtrl()

        current_row = 1
        while not control.quit():
            data = pd.read_csv('distance.csv')

            # seeing if next data point available in csv
            # if it is available -> move the drone
            if len(data.index)> current_row:
                current_row = len(data.index) 

                # getting last row of csv
                bottom = data.tail(1)
                x = bottom["dx"]
                x = x.iloc[0]
                y = bottom["dy"]
                y = y.iloc[0]
                z = bottom["dz"]
                z = z.iloc[0]

                print('distance in meters away from target is :')
                print(x,y,z)  

            if control.takeoff():
                self.drone(TakeOff())
            elif control.landing():
                print("Landing...")
                self.drone(Landing())
                print("Landed\n")
            if control.has_piloting_cmd():
                self.drone(
                    PCMD(
                        1,
                        control.roll(),
                        control.pitch(),
                        control.yaw(),
                        control.throttle(),
                        timestampAndSeqNum=0,
                    )
                )
            else:
                self.drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))
            time.sleep(0.05)            


if __name__ == "__main__":
    streaming_example = StreamingExample()
    # Start the video stream
    streaming_example.start()
    # Stop the video stream
    
    streaming_example.fly_by_keys()

    # record for 10 seconds
    # time.sleep(10)

    streaming_example.stop()
    # print(streaming_example.position)
    # Recorded video stream postprocessing
    streaming_example.postprocessing()
