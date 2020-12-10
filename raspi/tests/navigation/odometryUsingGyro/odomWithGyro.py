import serial
import json
import time
import math
#from odometry import odometry
from gyro import angleComputation
import numpy as np

def odometryGyro(gz, r, pose, omega, dt):
    # pose = [x, y, theta] with respect to arena reference frame [m, m, rad] as numpy array (column vector) 
    # omega = [wl, wr] [rad/s] as a list
    # gz: gyro rate [°/s]
    # r: wheel radius [m]
    # t: elapsed time between two measurements(inverse of loop frequency) [s]
    vl = omega[0]*r
    vr = omega[1]*r
    linearV = (vl+vr)/2
    #rotV = (vl-vr)/(2*l) 
    rotV = gz*math.pi/180
    dotVector = np.array([[linearV, 0, rotV]]).transpose()
    invRotMatrix = np.array([[math.cos(pose[2][0]), -math.sin(pose[2][0]), 0], [math.sin(pose[2][0]), math.cos(pose[2][0]), 0], [0, 0, 1]])
    newPose = pose + invRotMatrix.dot(dotVector)*dt
    return newPose 


# Assumption 50Hz signals 
l = 0.144
r = 0.04
theta = 0
pose = np.array([[1, 1, 0]]).transpose()
print(pose,theta)
time.sleep(3)

if __name__ == '__main__':
    
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    ser.flush()
    time_start = time.time()
    while True:
        if ser.in_waiting > 0:
            decoded = json.loads(ser.readline())
            gz = decoded["gyro"]
            if (abs(decoded["motorSpeed"][0]-415)< 10):
                decoded["motorSpeed"][0] = 415
            if (abs(decoded["motorSpeed"][1]-415)< 10):
                decoded["motorSpeed"][1] = 415
            avSpeedR = -((float(decoded["motorSpeed"][0])-415.00)/415.00)*6.25 #rad/s
            avSpeedL = ((float(decoded["motorSpeed"][1])-415.00)/415.00)*6.25  #rad/s
            omega = [avSpeedL, avSpeedR]
            time_end = time.time()
            t = time_end - time_start 
            pose = odometryGyro(gz, r, pose, omega, t)
            theta = angleComputation(theta, gz, t)
            print(pose, theta)
            time_start = time.time()
        else:
            time_start = time.time()
            pass
        time.sleep(0.02)
        
