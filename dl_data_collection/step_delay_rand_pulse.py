# this code saves a csv file of velocity data in order to train deep learning algorithm
# code randomly sends an actuation of left/right/forward/back/up/down
# using piloting_pcmd
# press esc if want to land drone immediately

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
import random

import os.path
from os import path

import olympe
import time
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, SpeedChanged, AttitudeChanged
from olympe.messages.ardrone3.Piloting import TakeOff, Landing 

# for keyboard
from pynput.keyboard import Listener, Key, KeyCode
from collections import defaultdict
from enum import Enum

DRONE_IP = "192.168.42.1"
# DRONE_IP = "101.202.0.1"

#Keyboard class for landing drone
class Ctrl(Enum):
    (
        QUIT,
        LANDING
    ) = range(2)

QWERTY_CTRL_KEYS = {
    Ctrl.QUIT: Key.esc,
    Ctrl.LANDING: "l"
}

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
 
    def _rate_limit_cmd(self, ctrl, delay):
        now = time.time()
        if self._last_action_ts[ctrl] > (now - delay):
            return False
        elif self._key_pressed[self._ctrl_keys[ctrl]]:
            self._last_action_ts[ctrl] = now
            return True
        else:
            return False

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

# class captures velocity data while flying the drone
class Step_Response(threading.Thread):

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)

        # connect the the drone
        self.drone.connect()
        self.drone.start_piloting()

        # parameter for when to record data
        self.record = True

        # pulse time for pcmd and data sampling time
        self.pulse = 0.02
        self.sample = 0.01

        # initializing states
        self.state1 = 0 #forward-back
        self.state2 = 0 #left-right
        self.state3 = 0 #up-down

        super().__init__()
        super().start()
    
    def start(self):
        # creating csv file
        dur = str(self.pulse)
        dur = dur.replace('.','')
        fs = str(self.sample)
        fs = fs.replace('.','')
        filename = 'allvel_test_60s_pulse_' + dur + '_smp_' + fs + "_NED2" + ".csv"
        fieldnames = ["time","vel_x","vel_y","vel_z", "roll", "pitch", "yaw","st1","st2","st3"]
        with open(filename,'w') as newFile:
            writer = csv.DictWriter(newFile, fieldnames=fieldnames)
            writer.writeheader()

    def stop(self):
        # Properly stop the piloting and disconnect
        self.drone.stop_piloting()
        self.drone.disconnect()

    def fly(self):
        # quit keys by esc key
        # while esc not pressed, continue to read csv file to see where drone target is 
        control = KeyboardCtrl()

        print("Taking off")
        self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()

        #loop parameters
        N = 6 #num of actions
        dt = self.pulse
        timeout = time.time() + 60 #one minute from now

        #if esc is pressed or time has exceeded 1 minute
        while not control.quit() and time.time() <= timeout:
            n = random.randint(0,N)
            if n == 0:
                # moving forward 50% 
                # print('moving forward')
                self.drone.piloting_pcmd(0, 50, 0, 0, dt)
                time.sleep(dt)

                #saving state 
                self.state1 = self.state1 + 1
            elif n==1:
                # moving bkward 50% 
                # print('moving backward')
                self.drone.piloting_pcmd(0, -50, 0, 0, dt)
                time.sleep(dt)

                #saving state 
                self.state1 = self.state1 - 1
            elif n==2:
                # moving right 50%
                # print('moving right')
                self.drone.piloting_pcmd(50, 0, 0, 0, dt)
                time.sleep(dt)

                #saving state 
                self.state2 = self.state2 + 1
            elif n==3:
                # moving left 50%
                # print('moving left') 
                self.drone.piloting_pcmd(-50, 0, 0, 0, dt)
                time.sleep(dt)

                #saving state 
                self.state2 = self.state2 - 1
            elif n==4:
                # moving up 50%
                # print('moving up') 
                self.drone.piloting_pcmd(0, 0, 0, 50, dt)
                time.sleep(dt)

                #saving state 
                self.state3 = self.state3 - 1
            elif n==5:
                # moving down 50%
                # print('moving down') 
                self.drone.piloting_pcmd(0, 0, 0, -50, dt)
                time.sleep(dt)

                #saving state 
                self.state3 = self.state3 + 1
            else:
                # print('doing nothing')
                time.sleep(dt)
            
        print("Landing...")
        # self.drone(Landing()).wait().success()
        self.drone(Landing())
        time.sleep(3)
        print("Landed\n")

    def run(self):
        self.fly()
        self.record = False

    def collect_data(self):
        t_sample = self.sample
        t = 0
        print('Collecting Data! \n')
        print(self.record)
        while self.record:
            time.sleep(t_sample)
            # getting speed and velocity data
            dict = self.drone.get_state(SpeedChanged)
            at_dict = self.drone.get_state(AttitudeChanged)

            # # print for debugging 
            # print("Speed changed : ", [t,dict['speedX'],dict['speedY'],dict['speedZ']])

            # naming convension for file
            dur = str(self.pulse)
            dur = dur.replace('.','')
            fs = str(self.sample)
            fs = fs.replace('.','')
            filename = 'allvel_test_60s_pulse_' + dur + '_smp_' + fs + "_NED2" + ".csv"
            fieldnames = ["time","vel_x","vel_y","vel_z", "roll", "pitch", "yaw","st1","st2","st3"]
            
            # writing data to file
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
                    "st1": self.state1,
                    "st2": self.state2,
                    "st3": self.state3
                }
                writer.writerow(info) 
            t = t + t_sample


if __name__ == "__main__":
    step_resp = Step_Response()
    step_resp.start()
    step_resp.collect_data()
    step_resp.stop()

    

