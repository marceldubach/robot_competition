import serial
import time
import json
if __name__=='__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=0.5)
    time.sleep(0.1)
    ser.flush()

    start_time = time.time()
    print("Program started")
    running = True
    # Arduino replies to any commands. Give an initial command
    dt = 3
    t_old = time.time()-dt

    while(running):
        #serial_string = "{\\\"command\\\": [5, 10]}\}\n"
        #print(serial_string)
        #ser.write(b"{\\\"command\\\": [5, 10]}\}\n") #.encode('utf-8'))
        #ser.write(b"{\\\"command\\\":[10, 20]}")

        data = {}
        # this command should depend on the image detection!
        data["command"] = [10,30]
        data = json.dumps(data)
        ser.write(data.encode('ascii'))

        time.sleep(3)

        if (ser.in_waiting):

            json_string= ser.readline().decode('utf-8').rstrip()
            # rstrip removes the '\n' at the end of the line

            print("Received odometry value: " + json_string)
            t_old = time.time()

        end_time = time.time()
        print("Elapsed time" + str(end_time-start_time))
        if(end_time-start_time>10):
            running = False

