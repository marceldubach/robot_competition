import serial
import json
import time
import math
from odometry import odometryGyroAccel
from gyro import angleComputation
import numpy as np

# Assumption 50Hz signals 
r = 0.04
theta = 0
pose = np.array([[1, 1, 0]]).transpose()
#t = 0.02
print(pose,theta)

if __name__ == '__main__':
    
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    ser.flush()
    ser.write(b"start\n")
    #time_start = time.time()
    while True:
        if ser.in_waiting > 0:
            try:
                decoded = json.loads(ser.readline())

                gz = decoded["gyro"]
                ax = decoded["accel"]
                t = float(decoded["dT"])/1000.0
                if (abs(decoded["motorSpeed"][0]-415)< 10):
                    decoded["motorSpeed"][0] = 415
                if (abs(decoded["motorSpeed"][1]-415)< 10):
                    decoded["motorSpeed"][1] = 415
                avSpeedR = -((float(decoded["motorSpeed"][0])-415.00)/415.00)*6.25 #rad/s
                avSpeedL = ((float(decoded["motorSpeed"][1])-415.00)/415.00)*6.25  #rad/s
                omega = [avSpeedL, avSpeedR]
                #time_end = time.time()
                #t = time_end - time_start
                pose = odometryGyroAccel(gz, ax, r, pose, omega, t)
                theta = angleComputation(theta, gz, t)
                #jsonString = json.dumps(theta)
                #ser.write(jsonString)
                if pose[0] > 3:
                    ser.write(b"stop\n")
                print(pose)
            except:
                print("Didnt decode value")
                pass
            #time_start = time.time()
        else:
            pass
        time.sleep(0.02)
        
