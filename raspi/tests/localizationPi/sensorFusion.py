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
from odometry import odometryGyroAccel, velocity
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

def triangulation(queue, yaw, webcam):
    filename = savePicture(webcam)
    centroids = extractCentroids(filename)
    xCenterM, yCenterM, yaw = computePosition(centroids, yaw)
    data = {"xCenterM": xCenterM, "yCenterM": yCenterM, "yaw": yaw}
    #print(data)
    queue.put(json.dumps(data))

def mainLoop(queue, ser, kFilter, pose, v, gz, dt, ax, i, count):
    if ser.in_waiting > 0:
        decoded = json.loads(ser.readline())
        gz = decoded["gyro"]
        ax = decoded["accel"]
        dt = float(decoded["dT"])/1000.0
        if (abs(decoded["motorSpeed"][0]-415)< 10):
            decoded["motorSpeed"][0] = 415
        if (abs(decoded["motorSpeed"][1]-415)< 10):
            decoded["motorSpeed"][1] = 415
        avSpeedR = -((float(decoded["motorSpeed"][0])-415.00)/415.00)*6.25 #rad/s
        avSpeedL = ((float(decoded["motorSpeed"][1])-415.00)/415.00)*6.25  #rad/s
        omega = [avSpeedL, avSpeedR]
        pose = odometryGyroAccel(gz, ax, r, pose, omega, dt)
        yaw = angleComputation(yaw, gz, dt)
        v = velocity(v, ax, dt)
        #jsonString = json.dumps(theta)
        #ser.write(jsonString)
        #if pose[0] > 3:
        #    ser.write(b"stop\n")
        data = {"x": pose[0], "y": pose[1], "yaw": pose[2], "v": v, "gz": gz, "dT": dt}
        #print(data)
        queue.put(json.dumps(data))
    else:
        pass 

def main():

    queueBeac = Queue()
    queueMain = Queue()
    pBeac = Process(target=triangulation, args=(queueBeac, yaw, webcam))
    pMain = Process(target=mainLoop, args=(queueMain, ser, kFilter, pose, v, gz, dt, ax, i, count))
    pMain.start()
    pMain.join()
    if(queueBeac.empty()):
        pBeac.start()
        measurements = json.loads(queueBeac.get(block=True, timeout=None))
        pBeac.join()
    if(len(measurements) > 0):
        queueMain = Queue()
        pMain.start()
        odometry = json.loads(queueMain.get(block=True, timeout=None))
        pMain.join()
        state_vector =  np.array([[odometry["x"], odometry["y"], odometry["yaw"], odometry["v"], odometry["gz"]]]).transpose()
        measurements_vector = np.array([[measurements["xCenterM"], measurements["yCenterM"], measurements["yaw"]]]).transpose()
        dT = odometry["dT"]
        kalmanFilter(state_vector, measurements_vector, dT, ax, Pk, Q, R)

    """
    pBeac.start()
    pMain.start()
    measurements = json.loads(queueBeac.get(block=True, timeout=None))
    pBeac.join()
    pMain.join()
    """

if __name__ == '__main__':
    
    # serial object instantiation
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    ser.flush()
    
    # webcam object creation and setup
    webcam = cv.VideoCapture(0) #ID 0
    webcam.set(cv.CAP_PROP_FRAME_WIDTH, 1920) 
    webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080) 
    time.sleep(0.1)
    
    main()

    webcam.release()








    """
    while i< 5:
        print(processAlive)
        if not processAlive:
            absolutePose = beaconsProcess(yaw, webcam, processAlive)
            i += 1
        if len(absolutePose) > 0:
            # absolute position available -> Kalman filter using previous step odometry estimate
            state_vector = np.array([[pose[0], pose[1], pose[2], v, gz]]).transpose()
            measurements_vector = np.array([[absolutePose["xCenterM"], absolutePose["yCenterM"], absolutePose["yaw"]]]).transpose()
            state = kalmanFilter(state_vector, measurements_vector, dt, ax, Pk, Q, R)
            print(state)
            kFilter = True
            
            if ser.in_waiting > 0:
                decoded = json.loads(ser.readline())
                gz = decoded["gyro"]
                ax = decoded["accel"]
                dt = float(decoded["dT"])/1000.0
                if (abs(decoded["motorSpeed"][0]-415)< 10):
                    decoded["motorSpeed"][0] = 415
                if (abs(decoded["motorSpeed"][1]-415)< 10):
                    decoded["motorSpeed"][1] = 415
                avSpeedR = -((float(decoded["motorSpeed"][0])-415.00)/415.00)*6.25 #rad/s
                avSpeedL = ((float(decoded["motorSpeed"][1])-415.00)/415.00)*6.25  #rad/s
                omega = [avSpeedL, avSpeedR]
                if kFilter:
                    # updated pose from Kalman filter available -> merge it in the odometry 
                    pose = np.array([[state[0], state[1], state[2]]]).transpose()
                    yaw =  state[2]
                    kFilter = False
                pose = odometryGyroAccel(gz, ax, r, pose, omega, dt)
                yaw = angleComputation(yaw, gz, dt)
                v = velocity(v, ax, dt)
                #jsonString = json.dumps(theta)
                #ser.write(jsonString)
                #if pose[0] > 3:
                #    ser.write(b"stop\n")
                print(pose)
                
        else:
            pass
            count += 1
        if (count > 200):
            webcam.release() 
        time.sleep(0.02)
        """
        

        
