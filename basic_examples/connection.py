# -*- coding: UTF-8 -*-

# Connecting to ANAFI drone
import olympe

DRONE_IP = "192.168.42.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone.disconnect()

