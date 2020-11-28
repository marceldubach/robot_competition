
from picamera import PiCamera
import argparse
import os

parser = argparse.ArgumentParser()


parser.add_argument("name",help="name the photo to save as .jpg")
parser.add_argument("-c","--cam",help="camera type: add -c 1 to change to normal camera",type=int)
args = parser.parse_args()
camera = PiCamera()
camera.rotation = 180


cameraType = "noIR"
if args.cam:
    cameraType = "normal"

if not os.path.exists('images/'+cameraType):
    os.makedirs('images/'+cameraType)

camera.capture("images/"+cameraType+"/"+args.name+".jpg")
print("Saved image to "+"images/"+cameraType+"/"+args.name+".jpg")
