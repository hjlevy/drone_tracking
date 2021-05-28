# -*- coding: UTF-8 -*-
# testing the minimum duration allowed by piloting pcmd to see a movement
# it is about 150ms - 200ms

import olympe
import time
from olympe.messages.ardrone3.PilotingState import PilotedPOI, FlyingStateChanged
from olympe.messages.ardrone3.Piloting import TakeOff, StartPilotedPOI, Landing, StopPilotedPOI

DRONE_IP = "192.168.42.1"


def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()	
    drone.start_piloting()
    
    #starting timer		
    start = time.time()
    print("MOVING FORWARD COMMAND START")
    drone.piloting_pcmd(0, 50, 0, 0, 0.2)
    time.sleep(0.2)
    #drone.piloting_pcmd(roll=0, pitch=50, yaw=0, gaz=0, piloting_time=1)

    #ending timer
    end = time.time()

    drone.stop_piloting()
    assert drone(Landing()).wait().success()
    drone.disconnect()
    print("TIMER VALUE: %f" % (end-start))


if __name__ == "__main__":
    main()
