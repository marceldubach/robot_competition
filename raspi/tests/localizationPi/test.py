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

    check, frame = webcam.read()
    cv.imwrite(filename='images/beacons.jpg', img=frame)
    webcam.release()
    print("Image saved!")
    print(check)
    
   