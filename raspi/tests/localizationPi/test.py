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
if __name__ == '__main__':
        
    # webcam object creation and setup
    webcam = cv.VideoCapture(0) #ID 0
    webcam.set(cv.CAP_PROP_FRAME_WIDTH, 1920) 
    webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080) 
    time.sleep(1)
    if not (webcam.isOpened()):
        print("Could not open video device")

    try:
        check, frame = webcam.read()
        if check:
            if not os.path.exists('images'):
                os.makedirs('images')
            cv.imwrite(filename='images/beacons.jpg', img=frame)
            #webcam.release()
            print("Image saved!")
            return filename
        else:
            savePicture(webcam)
    except:
        print("Problem saving image")
    
   










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
        

        
