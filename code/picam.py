print("Hello world!")

from picamera import PiCamera
from datetime import datetime

camera = PiCamera()
camera.rotation = 180

now  = datetime.now()
date_time = now.strftime("%Y/%m/%d-%S:%M:%S")

import time
#camera.capture("image-"+date_time+".jpg")

camera.capture("foo.jpg")