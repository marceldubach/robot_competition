import serial
import json
import time
from odometry import odometry
import numpy as np

l = 0.144
r = 0.04
pose = np.array([[1, 1, 0]]).transpose()
print(pose)
t = 0.1
time.sleep(1)

if __name__ == '__main__':
    
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    time_start = time.time()
    while True:
        if ser.in_waiting > 0:
            decoded = json.loads(ser.readline())
            #print(decoded)
            #print(decoded["motorSpeed"][0])
            #print(decoded["motorSpeed"][1])
            if (abs(decoded["motorSpeed"][0]-415)< 10):
                decoded["motorSpeed"][0] = 415
            if (abs(decoded["motorSpeed"][1]-415)< 10):
                decoded["motorSpeed"][1] = 415
            avSpeedR = -((float(decoded["motorSpeed"][0])-415.00)/415.00)*6.25 #rad/s
            avSpeedL = ((float(decoded["motorSpeed"][1])-415.00)/415.00)*6.25   #rad/s
            omega = [avSpeedL, avSpeedR]
            time_end = time.time()
            t = time_end - time_start 
            pose = odometry(l, r, pose, omega, t)
            print(pose)
            time_start = time.time()
            time.sleep(0.1)
