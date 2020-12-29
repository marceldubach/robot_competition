import serial
import time
import json
import numpy as np

""" This scripts implements a bidirectional communication at ca. 10 Hz
    Run this script together with 'gripper_test' on Arduino.
    
    29.Dec.2020
    Done:
        - Bidirectional communication
        - Odometry implemented on Arduino
        - Implement the position controller
        - Implement servo control
    ToDo:
        - Implement the state machine (goal: by 29.12.20)
        - Add Localization update on python (camera)
        - Implement a Kalman Filter
        - Add Bottle Detection to the Script
        """
def get_time(time_start):
    return time.time() - time_start

def write_to_serial(serial, state, waypoint):
    message = {"state": state, "ref": [float(waypoint[0]), float(waypoint[1])]}
    # print(json.dumps(message))
    serial.write(json.dumps(message).encode('ascii'))

if __name__=='__main__':
    t_max = 20
    # initalize time for display
    t_s = time.time()


    print("Start simulation. Duration: ", t_max ," seconds")
    t_s = time.time()
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout = 0.5)

    if (ser.isOpen()):
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial successfully initialized")
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial connection failed")

    time.sleep(3)

    if (ser.in_waiting>0):
        line = ser.readline()
        print("{:6.2f}".format(get_time(t_s)) + " [SER] Arduino: ", line)
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [ERROR] Arduino is not responding")
    ser.flush()

    # predefined waypoints
    waypoints = np.array([[2,1],[2,2]])
    i_wp = 0 # iterator over waypoints
    wp = np.array([0.5,0.5])

    pose = np.empty(3) # estimated position
    # MAIN LOOP HERE
    while(time.time()-t_s < t_max):
        if (time.time()-t_s < 2):
            state = 0
        # TODO do this properly
        elif (time.time()-t_s < 20): # state = 1: track waypoints
            state = 2
            if i_wp<len(waypoints):
                wp = waypoints[i_wp]
                if (abs(wp[0]-pose[0])<0.2) and (abs(wp[1]-pose[1])<0.2):

                    print("{:6.2f}".format(get_time(t_s)) + " [P-CTRL] waypoint ", waypoints[i_wp], " reached!")
                    i_wp += 1
                    # print("{:6.2f}".format(get_time(t_s)) + " [P-CTRL] new waypoint: ", waypoints[i_wp])
            if i_wp==len(waypoints):
                wp = np.array([0.5,0.5])

        print("{:6.2f}".format(get_time(t_s)) + " [P-CTRL] waypoint is: ", wp)
        write_to_serial(ser, state, wp)
        ser.flush()
        time.sleep(0.1) # TODO replace this with while

        if (ser.in_waiting>0): #if (len(line)>0):
            t0 = time.time()
            line = ser.readline().decode('ascii').rstrip()
            # line = ser.read(52).decode('utf-8').rstrip()
            # print("Time to read: ", time.time()-t0)
            ser.reset_input_buffer()
            # print(line)
            try:
                data = json.loads(line)
                pose = np.array(data["pos"])
                print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", int(data["state"]), " pos: ", data["pos"], ", cmd:", data["cmd"],", ref:", data["ref"])
                # print("{:6.2f}".format(get_time(t_s)) + " [SER] state: ",data["state"], " pos:", data["pos"])
            except:
                print("[ERROR] cannot deserialize string from arduino")
                print(line)

        else:
            print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")

        # time.sleep(0.5) # doen't work for time sleep <0.6!

    # shut down the Robot
    state = 3
    wp_end = np.array([0.5,0.5])
    write_to_serial(ser, state, wp_end)
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Shutting motors down")
    time.sleep(1)
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Time elapsed. Program ending.")
    ser.close() # close serial   port at the end of the code