
from picamera import PiCamera
from datetime import datetime

camera = PiCamera()
camera.rotation = 180

print("Hello world!")

now  = datetime.now()
date_time = now.strftime("%Y/%m/%d_%H/%M/%S")

import time
#camera.capture("image-"+date_time+".jpg")
print(date_time+".jpg")
camera.capture("images/foo.jpg")
camera.capture(date_time+".jpg")