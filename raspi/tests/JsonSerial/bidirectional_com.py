import serial
import time
import json

""" This scripts implements a bidirectional communication at LOW RATE (around 1 Hz)
    Run this script together with 'serial_com' on Arduino.
    
    28.Dec.2020
    
    TODO:
        - Add odometry integration on the Arduino
        - Add Localization update on python
        - Implement a Kalman Filter
        - Implement the state machine
        - Implement the gripper movement
        - Implement the position controller
        """
def get_time(time_start):
    return time.time() - time_start

def write_to_serial(serial, state, cmdLeft, cmdRight):
    message = {"state": state, "cmd": [cmdLeft, cmdRight]}
    serial.write(json.dumps(message).encode('ascii'))

if __name__=='__main__':
    t_max = 20
    # initalize time for display
    t_s = time.time()


    print("Start simulation. Duration: ", t_max ," seconds")
    t_s = time.time()
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout = 0.5)

    if (ser.isOpen()):
        print("{:6.2f}".format(get_time(t_s)) + " [SER] serial successfully initialized")
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [SER] connection failed")

    time.sleep(2)

    if (ser.in_waiting>0):
        line = ser.readline().decode('ascii').rstrip()
        print("{:6.2f}".format(get_time(t_s)) + " [SER] Arduino: ", line)
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [ERROR] Arduino is not responding")
    ser.flush()

    # initialize the state machine
    state = 1

    # generate commands to motors
    # TODO define commands as angle and distance!
    cmdLeft = 0
    cmdRight = 0

    # MAIN LOOP HERE
    while(time.time()-t_s < t_max):

        cmdLeft = 10
        cmdRight = 20

        write_to_serial(ser, state, cmdLeft, cmdRight)
        ser.flush()
        time.sleep(0.5)

        # ser.in_waiting>0 does not work properly

        if (ser.in_waiting>0): #if (len(line)>0):
            t0 = time.time()
            line = ser.readline().decode('ascii').rstrip()
            # line = ser.read(52).decode('utf-8').rstrip()
            #print("Time to read: ", time.time()-t0)

            #print(line)
            data = json.loads(line)
            print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", int(data["state"]), " pos", data["pos"])
            # print("{:6.2f}".format(get_time(t_s)) + " [SER] state: ",data["state"], " pos:", data["pos"])
        else:
            #print("{:6.2f}".format(get_time(t_s)) + " [SER] state: ",data["state"][0], " pos:", data["pos"][0])
            # print("{:6.2f}".format(get_time(t_s)) + " [SER] Error: Received string of length " + str(len(line)))
            print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")


        time.sleep(0.6) # doen't work for time sleep <0.6!

    print("Time elapsed. Program ending.")
    ser.close() # close serial   port at the end of the code