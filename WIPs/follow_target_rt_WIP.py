# Code attempting to move drone in response to frame target analysis
# moving in show_yuv frame ... might not be the correct method

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

import keyboard

import os.path
from os import path

import time
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxAltitude, MaxTilt, MaxDistance, NoFlyOverMaxDistance
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages import gimbal



olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = "192.168.42.1"
# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')


class StreamingExample(threading.Thread):

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.line1 = []

        super().__init__()
        super().start()
    
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
        self.drone.start_piloting()
        self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()    
        time.sleep(1)
        self.drone(moveBy(0,0,-0.5,0)).wait()
        time.sleep(1)

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
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
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
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame) 



    def flush_cb(self):
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        # frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        # info = h264_frame.info()
        # frame_ts = info["ntp_raw_timestamp"]
        # if not bool(info["h264"]["is_sync"]):
        #     if len(self.h264_frame_stats) > 0:
        #         while True:
        #             start_ts, _ = self.h264_frame_stats[0]
        #             if (start_ts + 1e6) < frame_ts:
        #                 self.h264_frame_stats.pop(0)
        #             else:
        #                 break
        #     self.h264_frame_stats.append((frame_ts, frame_size))
        #     h264_fps = len(self.h264_frame_stats)
        #     h264_bitrate = (
        #         8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
        #     self.h264_stats_writer.writerow(
        #         {'fps': h264_fps, 'bitrate': h264_bitrate})

    def depth_calc(self, w, dpx, W, p):
        # w = width of target in pixels, W = width of camera frame in pixels, p is pixel to meter conversion factor
        cam_angle = 110*np.pi/180
        # alpha = 2*np.arctan(w/W*np.tan(cam_angle/2))
        # L = w*p/(2*np.tan(alpha/2))
        w = p*w #width of target in meters
        beta = np.arctan(2*(w/2+dpx)/W*np.tan(cam_angle/2))
        L = (w/2+dpx)*p/np.tan(beta)
        return L

    def show_yuv_frame(self, window_name, yuv_frame):
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

        lower_red = np.array([125,85,95])
        upper_red = np.array([180,255,255])

        # creating a mask red-> white, anything else ->black
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
        cnt_found = False
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            cv2frame = cv2.rectangle(cv2frame,(x,y),(x+w,y+h),(0,255,0),2)

            # want to only analyze largest contour
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
                dz = self.depth_calc(w,dx/calx,width,calx)

            cnt_found = True 
        end = time.time()

        if cnt_found :    

            with open('distance.csv', 'a') as newFile:
                writer = csv.DictWriter(newFile,fieldnames = ["dx","dy","dz"])
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
            self.line1 = self.live_plotter(xc,yc,zc,self.line1)

            # only moving if not crazy amount

            if abs(dx) < 0.5 and abs(dy) < 0.5:
                print('moving!')
                self.drone(moveBy(dy,dx,0,0)).wait() 
            

        # print("TIMER VALUE: %f" % (end-start))
        # Use OpenCV to show this frame
        cv2.imshow(window_name, cv2frame)

    
        # fig.canvas.draw()
        cv2.waitKey(1)  # please OpenCV for 1 ms...

    def run(self):
        window_name = "Drone Tracking Target"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.show_yuv_frame(window_name, yuv_frame)
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
        cv2.destroyWindow(window_name)

    def fly(self):
        print("Taking off")
        self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        self.drone(moveBy(0,0,-0.5,0)).wait()
        time.sleep(10)

        print("Landing...")
        self.drone(Landing()).wait().success()
        print("Landed\n")


    def postprocessing(self):
        # Convert the raw .264 file into an .mp4 file
        h264_filepath = os.path.join(self.tempd, 'h264_data.264')
        mp4_filepath = os.path.join(self.tempd, 'h264_data.mp4')
        subprocess.run(
            shlex.split('ffmpeg -i {} -c:v copy -y {}'.format(
                h264_filepath, mp4_filepath)),
            check=True
        )

        # Replay this MP4 video file using the default video viewer (VLC?)
        # subprocess.run(
        #     shlex.split('xdg-open {}'.format(mp4_filepath)),
        #     check=True
        # )


if __name__ == "__main__":
    streaming_example = StreamingExample()
    # Start the video stream
    streaming_example.start()
    # Perform some live video processing while the drone is flying
    # streaming_example.fly()
    # Stop the video stream
    
    time.sleep(10)
    print("Landing...")
    streaming_example.drone(Landing()).wait().success()
    print("Landed\n")

    # # if spacebar pressed then land drone
    # while True:
    #     if keyboard.is_pressed(' '):
    #         print("Landing...")
    #         drone(Landing()).wait().success()
    #         print("Landed\n")

    streaming_example.stop()
    # Recorded video stream postprocessing
    streaming_example.postprocessing()
