# -*- coding: UTF-8 -*-
# testing takeoff and landing function

import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing

DRONE_IP = "192.168.42.1"
# DRONE_IP = "101.202.0.1"


def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
    time.sleep(10)
    assert drone(Landing()).wait().success()
    drone.disconnect()


if __name__ == "__main__":
    main()
