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
from kalmanFilter import kalmanFilter
from localization import setupWebcam, savePicture, extractCentroids, getReference, getAbsoluteAngle, computePosition
from odometry import odometryGyroAccel
from gyro import angleComputation
import multiprocessing
from multiprocessing import Lock, Process, Queue, current_process

# Initial conditions and variables declaration
i = 0
j = 0
count = 0
r = 0.04 # wheel radius
x0 = 1 
y0 = 1
yaw0 = 0
yaw = 0
v = 0
omega = 0
gz = 0
ax = 0
dt = 0
Pk = np.identity(5) # TO COMPUTE
Q = np.identity(5) # TO COMPUTE
R = np.identity(3) # TO COMPUTE
pose = np.array([[x0, y0, yaw0]]).transpose()
absolutePose = {}
processAlive = False
kFilter = False

def triangulation(queue, e, yaw, webcam):
    try:
        filename = '4_4pi.jpg'
    except:
         print("Problem retrieving filename")
    else:
        centroids = extractCentroids(filename)
        xCenterM, yCenterM, yaw = computePosition(centroids, yaw)
        data = {"xCenterM": xCenterM, "yCenterM": yCenterM, "yaw": yaw}
        #print(data)
        queue.put(json.dumps(data))
        e.set()
        return queue

def mainLoop(queueMain, e, ser, kFilter, pose, v, gz, dt, ax, i, count):

    i = 0
    while(not e.is_set()):
        #print(1)
        #if (i== 5):
        #    e.set()
        time.sleep(0.2)
        #queueMain.put(i)
        i += 1
    return queueMain.put(i)

def triang():
    i = 0
    while i < 2:
        e = multiprocessing.Event()
        queueMain = Queue()
        queueBeac = Queue()
        pBeac = Process(target=triangulation, args=(queueBeac, e, yaw, webcam)) #update yaw
        pMain = Process(target=mainLoop, args=(queueMain, e, ser, kFilter, pose, v, gz, dt, ax, i, count))
        pMain.start()
        pBeac.start()
        pBeac.join()
        pMain.join()
        #sensor fusion
        q = json.loads(queueBeac.get())
        print("centrpids", q["yaw"])
        i += 1
    return queueMain

if __name__ == '__main__':
    
    ser = 0
    webcam = 0
    j = 0
    q = triang()
    print("waiting")
    while j < 100:
        
        if not q.empty():
            elem = q.get()
            print(j, elem)
        j += 1
        time.sleep(0.01)