#Imports
import math 
import serial
import numpy as np
import time
import json
import cv2 as cv
import usb.core
import usb.util 
import imutils
import time 
import os
from kalmanFilter import kalmanFilter
from localization import setupWebcam, savePicture, extractCentroids, getReference, getAbsoluteAngle, computePosition
import multiprocessing
from multiprocessing import Lock, Process, Queue, current_process

# Initial conditions and variables declaration
r = 0.04 # wheel radius
x0 = 1 
y0 = 1
yaw0 = 0
pose0 = np.array([[x0, y0, yaw0]]).transpose()
Pk = np.identity(5) # TO COMPUTE
Q = np.identity(5) # TO COMPUTE
R = np.identity(3) # TO COMPUTE

def triangulation(queue, e, yaw):
    try:
        filename = savePicture()
        # set event to tell the readingOdometry to read the serial information
        e.set()
    except:
         print("Problem retrieving filename")
    else:
        centroids = extractCentroids(filename)
        xCenterM, yCenterM, yaw = computePosition(centroids, yaw)
        data = {"xCenterM": xCenterM[0], "yCenterM": yCenterM[0], "yaw": yaw[0]}
        print(data)
        queue.put(json.dumps(data))
        return queue

def readingOdometry(queue, e, ser):

    while (not e.is_set()):
        if ser.in_waiting > 0:
            odom = json.loads(ser.readline())
        else:
             pass
    # when the event is triggered, pass the state vector computed by odometry
    data = {"x": pose[0][0], "y": pose[1][0], "yaw": pose[2][0], "v": v, "gz": gz, "dT": dt, "ax": ax}
    queue.put(json.dumps(data))
    return  queue

def sensorFusion(pose, Pk, Q, R, ser):

    i = 0
    while(i < 10):
        e = multiprocessing.Event()
        queueBeac = Queue()
        queueOdom = Queue()
        pOdom = Process(target=readingOdometry, args=(queueOdom, e, ser))
        pBeac = Process(target=triangulation, args=(queueBeac, e, pose[2]))
        pOdom.start()
        pBeac.start()
        pOdom.join()
        pBeac.join()
        try:
            state = json.loads(queueOdom.get())
            state_vector = np.array([[state["x"], state["y"], state["yaw"], state["v"], state["gz"]]]).transpose()
            dT = state["dT"]
            ax = state["ax"]
            measurements = json.loads(queueBeac.get())
            measurements_vector = np.array([[measurements["xCenterM"], measurements["yCenterM"], measurements["yaw"]]]).transpose()
            
            print("state vector", state_vector)
            print("measurements vector", measurements_vector)
        
        except:
            print("not able to get state or measurement vector")
        else:
            pose, Pk = kalmanFilter(state_vector, measurements_vector, dT, ax, Pk, Q, R)
            print("kalman filter pose:", pose)


if __name__ == '__main__':
    
    # serial object instantiation
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    ser.flush()

    time.sleep(1)
    sensorFusion(pose0, Pk, Q, R, ser)



        

        
