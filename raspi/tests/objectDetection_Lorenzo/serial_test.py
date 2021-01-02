import serial
import time
import json
import numpy as np
import cv2 as cv

from picamera import PiCamera

from detection import detect_bottle
if __name__=='__main__':

    camera = PiCamera()
    camera.rotation = 180
    camera.resolution = (1280, 720)
    img = np.empty((720, 1280, 3))


    ser = serial.Serial('/dev/ttyACM0',9600,timeout=0.5)
    time.sleep(0.1)
    ser.flush()

    start_time = time.time()
    print("[main] Program started")
    running = True
    # Arduino replies to any commands. Give an initial command
    dt = 3
    t_old = time.time()-dt
    i = 0

    while(running):
        #serial_string = "{\\\"command\\\": [5, 10]}\}\n"
        #print(serial_string)
        #ser.write(b"{\\\"command\\\": [5, 10]}\}\n") #.encode('utf-8'))
        #ser.write(b"{\\\"command\\\":[10, 20]}")

        camera.capture("images/image"+str(i)+".jpg")
        i += 1
        #img = cv.imread("images/image"+str(i)+".jpg")
        img = cv.imread("images/old/2TileFromBottle.jpg") # TODO change this
        has_bottle, center, img_out = detect_bottle(img)
        #print(center)
        # convert center to motor commands here
        command_left = 450
        command_left = 450
        center_x2 = center[0]
        if (has_bottle):
            command_left = 450 + (640-center_x2) /640 * 400
            command_right = 450 + (center_x2-640)/640 * 400

        data = {}
        # this command should depend on the image detection!
        data["command"] = [command_left,command_right]
        data = json.dumps(data)
        ser.write(data.encode('ascii'))

        #time.sleep(2)

        if (ser.in_waiting):

            json_string= ser.readline().decode('utf-8').rstrip()
            # rstrip removes the '\n' at the end of the line

            print("[main] Received odometry value: " + json_string)
            t_old = time.time()

        end_time = time.time()
        print("[main] Elapsed time" + str(end_time-start_time))
        if(end_time-start_time>10):
            running = False
