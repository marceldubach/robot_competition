
from picamera import PiCamera
from datetime import datetime

camera = PiCamera()
camera.rotation = 0

print("Hello world!")

now  = datetime.now()
date_time = now.strftime("%Y/%m/%d_%H/%M/%S")

import time
#camera.capture("image-"+date_time+".jpg")
print(date_time+".jpg")
camera.capture("images/foo.jpg")

# this doesn't work for some stupid reason
# camera.capture("images/"+date_time+".jpg")