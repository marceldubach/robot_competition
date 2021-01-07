import cv2 as cv
from picamera import PiCamera
import argparse
import os
import time

parser = argparse.ArgumentParser()


parser.add_argument("name",help="name the photo to save as .jpg")
args = parser.parse_args()
camera = PiCamera()
camera.rotation = 180

camera.capture(args.name+".jpg")
webcam = cv.VideoCapture(0) #ID 0
webcam.set(cv.CAP_PROP_FRAME_WIDTH, 1920) 
webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080) 
time.sleep(0.5)
if not (webcam.isOpened()):
    print("Could not open video device")

try:
    check, frame = webcam.read()
    cv.imwrite(filename='webcam.jpg', img=frame)
    webcam.release()
    print("Image saved!")

except:
    print("Problem saving image")


