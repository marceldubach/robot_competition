print("Hello world!")

from picamera import PiCamera

camera = PiCamera()

camera.capture('foo.jpg')

