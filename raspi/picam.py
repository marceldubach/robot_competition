
from picamera import PiCamera
import argparse

parser = argparse.ArgumentParser()

args = parser.add_argument()
parser.parseargs("name",help="name the photo to save as .jpg")
parser.parseargs("-c","--cam",help="camera type: set to 1 for normal camera",type=int)
camera = PiCamera()
camera.rotation = 180

cameraType = "noIR"
if args.cam == 1:
    cameraType = "normal"

camera.capture("images/"+cameraType+"/"+args.name+".jpg")
print("Saved image to "+"images/"+cameraType+"/"+args.Name+".jpg")
