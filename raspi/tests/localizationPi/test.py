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
if __name__ == '__main__':
    yaw = np.pi
    filename = '4_4pi.jpg'
    centroids = extractCentroids(filename)
    xCenterM, yCenterM, yaw = computePosition(centroids, yaw)
    data = {"xCenterM": xCenterM, "yCenterM": yCenterM, "yaw": yaw}






   