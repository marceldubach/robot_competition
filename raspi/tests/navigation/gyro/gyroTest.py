import serial
import json
import time
from gyro import angleComputation
import numpy as np

yaw = 0
print(yaw)
time.sleep(3)

if __name__ == '__main__':
    
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    ser.flush()
    time_start = time.time()
    while True:

        if ser.in_waiting > 0:
            decoded = json.loads(ser.readline())
            gz = decoded["gyro"]
            time_end = time.time()
            t = time_end - time_start 
            yaw = angleComputation(yaw, gz, t)
            print(yaw)
            time_start = time.time()
            time.sleep(0.02)
