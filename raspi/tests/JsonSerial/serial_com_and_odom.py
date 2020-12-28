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



    # MAIN LOOP HERE
    while(time.time()-t_s < t_max):

        # updated inputs and state here
        # TODO do this properly
        if (time.time()-t_s < 10):
            state = 1
            cmdLeft = 50
            cmdRight = 200
        else:
            state = 0
            cmdLeft = 0
            cmdRight = 0

        write_to_serial(ser, state, cmdLeft, cmdRight)
        ser.flush()
        time.sleep(0.5)

        # ser.in_waiting>0 does not work properly

        if (ser.in_waiting>0): #if (len(line)>0):
            t0 = time.time()
            line = ser.readline().decode('ascii').rstrip()
            # line = ser.read(52).decode('utf-8').rstrip()
            #print("Time to read: ", time.time()-t0)
            ser.reset_input_buffer()
            #print(line)
            data = json.loads(line)
            print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", int(data["state"]), " pos", data["pos"])
            # print("{:6.2f}".format(get_time(t_s)) + " [SER] state: ",data["state"], " pos:", data["pos"])
        else:
            #print("{:6.2f}".format(get_time(t_s)) + " [SER] state: ",data["state"][0], " pos:", data["pos"][0])
            # print("{:6.2f}".format(get_time(t_s)) + " [SER] Error: Received string of length " + str(len(line)))
            print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")


        time.sleep(0.6) # doen't work for time sleep <0.6!

    # shut down the Robot
    state = 0;
    cmdLeft = 0;
    cmdRight = 0;
    write_to_serial(ser, state, cmdLeft, cmdRight)
    print("{:6.2f}".format(get_time(t_s)) + " Shut motors down")
    time.sleep(1)
    print("Time elapsed. Program ending.")
    ser.close() # close serial   port at the end of the code