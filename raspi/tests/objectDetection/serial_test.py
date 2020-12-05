import serial
import time
import json
if __name__=='__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    time.sleep(0.1)
    ser.flush()

    start_time = time.time()
    print("Program started")
    running = True
    # Arduino replies to any commands. Give an initial command


    while(running):
        #serial_string = "{\\\"command\\\": [5, 10]}\}\n"
        #print(serial_string)
        #ser.write(b"{\\\"command\\\": [5, 10]}\}\n") #.encode('utf-8'))
        #ser.write(b"{\\\"command\\\":[10, 20]}")
        data = {}
        data["command"] = [10,30]
        data = json.dumps(data)
        ser.write(data.encode('ascii'))

        #print({"command": [10,20]})
        arduino_received = ser.readline().decode('utf-8').rstrip()
        com_left= ser.readline().decode('utf-8').rstrip()
        com_right= ser.readline().decode('utf-8').rstrip()
        json_string= ser.readline().decode('utf-8').rstrip()
        # rstrip removes the '\n' at the end of the line
        print("Arduino received: "+ arduino_received)
        print("Command left: "+com_left)
        print("Command right: " + com_right)
        print("Received odometry value: " + json_string)
        # try:
        #     potential = float(line)
        # except ValueError:
        #     print("Not a float - set potential to zero")
        #     potential = 0

        # if (potential > 500):
        #     ser.write("led1\n".encode('utf-8'))
        # else:
        #     ser.write("led2\n".encode('utf-8'))


        end_time = time.time()

        print("Elapsed time" + str(end_time-start_time))
        if(end_time-start_time>10):
            running = False

