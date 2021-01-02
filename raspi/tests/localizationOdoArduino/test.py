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

if __name__ == '__main__':
    webcam = cv.VideoCapture(0) #ID 0
    webcam.set(cv.CAP_PROP_FRAME_WIDTH, 1920) 
    webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080) 
    time.sleep(0.3)
    i = 0
    while( i < 5):
        if not (webcam.isOpened()):
            print("Could not open video device")
        try:
            check, frame = webcam.read()
            if check:
                filename = 'img'+str(i)+'.jpg'
                cv.imwrite(filename, img=frame)
                print("Image saved!")
        except:
            print("Problem saving image")
            filename = 0
        i += 1
    webcam.release() 
