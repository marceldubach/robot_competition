import serial
import json
import time
import math
from odometry import odometryGyro
from gyro import angleComputation
import numpy as np

# Assumption 50Hz signals 
r = 0.04
theta = 0
pose = np.array([[1, 1, 0]]).transpose()
t = 0.02
print(pose,theta)

if __name__ == '__main__':
    
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    ser.flush()
    #time_start = time.time()
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
            #time_end = time.time()
            #t = time_end - time_start 
            pose = odometryGyro(gz, r, pose, omega, t)
            theta = angleComputation(theta, gz, t)
            print(pose, theta)
            #time_start = time.time()
        else:
            #time_start = time.time()
            pass
        time.sleep(0.02)
        
