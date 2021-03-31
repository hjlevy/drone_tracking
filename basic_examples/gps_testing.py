# -*- coding: UTF-8 -*-
# testing moveby function

import olympe
import time
import numpy as np
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged, GpsLocationChanged, AltitudeChanged, SpeedChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged
from olympe.messages.ardrone3.GPSState import NumberOfSatelliteChanged


DRONE_IP = "192.168.42.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy = 'wait'))

    print("GPS position before take-off :", drone.get_state(HomeChanged))
    # print("GPS number of satellites changed: ", drone.get_state(NumberOfSatelliteChanged)["numberOfSatellite"])
    assert drone(TakeOff()).wait().success()
    drone.start_piloting()
    time.sleep(1)

    for i in np.arange(5):
        time.sleep(1)
        print("GPS positionchanged : ", drone.get_state(PositionChanged))
        print("GPS altitudechanged: ", drone.get_state(AltitudeChanged))
        print("GPS loc changed : ", drone.get_state(GpsLocationChanged))
        print("Speed changed : ", drone.get_state(SpeedChanged))

    print("MOVING FORWARD COMMAND START")
    drone.piloting_pcmd(0, 50, 0, 0, 0.2)
    for i in np.arange(5):
        time.sleep(1)
        print("GPS positionchanged : ", drone.get_state(PositionChanged))
        print("GPS altitudechanged: ", drone.get_state(AltitudeChanged))
        print("GPS loc changed : ", drone.get_state(GpsLocationChanged))
        print("Speed changed : ", drone.get_state(SpeedChanged))

    drone.stop_piloting()
    assert drone(Landing()).wait().success()
    drone.disconnect()
