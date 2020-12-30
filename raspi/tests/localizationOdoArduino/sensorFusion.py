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
# import pandas as pd
from csv import writer
from time import localtime, strftime
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

def append_list_as_row(file_name, list_of_elem):
    # open file in append mode
    with open(file_name, 'a+', newline='') as write_obj:
        # create a writer object from csv module
        csv_writer = writer(write_obj)
        # add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elem)

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
            line = ser.readline()
            try:
                odom = json.loads(line)
            except:
                print("[ERROR] JSON cannot decode string:", line)
        else:
             pass
    # when the event is triggered, pass the state vector computed by odometry
    try:
        data = {"x": odom["pos"][0], "y": odom["pos"][1],
                "yaw": odom["pos"][2], "v": odom["info"][0],
                "gz": odom["info"][1], "dT": odom["info"][2],
                "ax": odom["info"][3]}
    #      TODO change dictionnary accordingly in Arduino File
    except:
        print("did't manage to retrieve data")
        print(odom)
    else:
        queue.put(json.dumps(data))
        return  queue

def sensorFusion(pose, Pk, Q, R, ser):

    i = 0
    while(i < 1):
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
            # send pose back to Arduino that will reference its relative displacement to it
            #  
            # TO DO
            # add sensor fusion result and state to log file 
            # row_contents = [strftime("%H:%M:%S", localtime()),str(pose[0][0]),str(pose[1][0]),str(pose[2][0]),'state']
            # Append list as new line to log csv file
            # append_list_as_row('log.csv', row_contents)
        i += 1
    print("Finished sensorFusion")

if __name__ == '__main__':
    
    # serial object instantiation
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    ser.flush()

    # initialize csv table 
    # df = pd.DataFrame({'time':[],'x': [],'y': [],'yaw': [],'state': []})
    # df.to_csv('log.csv', index = False)
    time.sleep(1)
    sensorFusion(pose0, Pk, Q, R, ser)



        

        
