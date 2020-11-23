
from picamera import PiCamera
import argparse

parser = argparse.ArgumentParser()


parser.add_argument("name",help="name the photo to save as .jpg")
parser.add_argument("-c","--cam",help="camera type: add -c to change to normal camera",action="store_true")
args = parser.parse_args()
camera = PiCamera()
camera.rotation = 180


cameraType = "noIR"
if args.cam:
    cameraType = "normal"

camera.capture("images/"+cameraType+"/"+args.name+".jpg")
print("Saved image to "+"images/"+cameraType+"/"+args.Name+".jpg")
