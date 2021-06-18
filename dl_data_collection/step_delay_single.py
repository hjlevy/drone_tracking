# this code saves a csv file of velocity data in order to determine time delay of step signal
# using piloting_pcmd

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

import os.path
from os import path

import olympe
import time
from olympe.messages.ardrone3.PilotingState import PilotedPOI, FlyingStateChanged, SpeedChanged, AttitudeChanged
from olympe.messages.ardrone3.Piloting import TakeOff, StartPilotedPOI, Landing, StopPilotedPOI, moveBy
from olympe.messages import gimbal

DRONE_IP = "192.168.42.1"
# DRONE_IP = "101.202.0.1"

class Step_Response(threading.Thread):

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        self.record = True
        # Connect the the drone
        self.drone.connect()
        self.drone.start_piloting()
        self.testname = 'left'
        self.pulse = 1
        self.sample = 0.01

        self.state = 0
        super().__init__()
        super().start()
    
    def start(self):

        # creating file 
        dur = str(self.pulse)
        dur = dur.replace('.','')
        fs = str(self.sample)
        fs = fs.replace('.','')
        filename = self.testname + "_vel_" + dur + '_smp_' + fs + "_st" + ".csv"
        fieldnames = ["time","vel_x","vel_y","vel_z", "roll", "pitch", "yaw" , "st"]
        with open(filename,'w') as newFile:
            writer = csv.DictWriter(newFile, fieldnames=fieldnames)
            writer.writeheader()

    def stop(self):
        # Properly stop the piloting and disconnect
        self.drone.stop_piloting()
        self.drone.disconnect()

    def fly(self):
        print("Taking off")
        self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()

        dt = self.pulse
        if self.testname == 'forward':
            # moving forward 50% 
            self.drone.piloting_pcmd(0, 50, 0, 0, dt)
            self.state = 1
            # self.drone(moveBy(0, 1, 0, 0))
            time.sleep(dt)

        elif self.testname == 'backward':
            # moving bkward 50% 
            self.drone.piloting_pcmd(0, -50, 0, 0, dt)
            self.state = 1

            time.sleep(dt)

        elif self.testname == 'right':
            # moving right 50%
            self.drone.piloting_pcmd(50, 0, 0, 0, dt)
            self.state = 1

            time.sleep(dt)

        elif self.testname == 'left':
            # moving left 50% 
            self.drone.piloting_pcmd(-50, 0, 0, 0, dt)
            self.state = 1

            time.sleep(dt) 

        
        elif self.testname == 'up':
            # moving up 50% 
            self.drone.piloting_pcmd(0, 0, 0, 50, dt)
            self.state = 1

            time.sleep(dt) 

        
        elif self.testname == 'down':
            # moving down 50% 
            self.drone.piloting_pcmd(0, 0, 0, -50, dt)
            self.state = 1

            time.sleep(dt) 
        #drone.piloting_pcmd(roll=0, pitch=50, yaw=0, gaz=0, piloting_time=1)   
        
        print("Landing...")
        # self.drone(Landing()).wait().success()
        self.drone(Landing())
        time.sleep(3)
        print("Landed\n")

    def wait_test(self):
        time.sleep(10)
        print('finished waiting\n')

    def run(self):
        self.fly()
        # self.wait_test()
        self.record = False

    def collect_data(self):
        t_sample = self.sample
        t = 0
        print('Collecting Data! \n')
        print(self.record)
        while self.record:
            time.sleep(t_sample)
            dict = self.drone.get_state(SpeedChanged)
            at_dict = self.drone.get_state(AttitudeChanged)
            # print("Speed changed : ", [t,dict['speedX'],dict['speedY'],dict['speedZ']])
            dur = str(self.pulse)
            dur = dur.replace('.','')
            fs = str(self.sample)
            fs = fs.replace('.','')
            filename = self.testname + "_vel_" + dur + '_smp_' + fs + "_st" + ".csv"
            fieldnames = ["time","vel_x","vel_y","vel_z", "roll", "pitch", "yaw", "st"]
            with open(filename, 'a') as newFile:
                writer = csv.DictWriter(newFile,fieldnames = fieldnames)
                info = {
                    "time": t,
                    "vel_x": dict['speedX'],
                    "vel_y": dict['speedY'],
                    "vel_z": dict['speedZ'],
                    "roll": at_dict['roll'],
                    "pitch": at_dict['pitch'],
                    "yaw": at_dict['yaw'],
                    "st": self.state
                }
                writer.writerow(info) 
            t = t + t_sample


if __name__ == "__main__":
    step_resp = Step_Response()
    step_resp.start()
    step_resp.collect_data()
    step_resp.stop()

    

