import serial
import time
import json
import numpy as np
import states
from multiprocessing import Lock, Process, Queue, current_process
import multiprocessing as mp



""" This scripts implements a bidirectional communication at ca. 10 Hz
    Run this script together with 'gripper_test' on Arduino.
    
    30.Dec.2020
    Done:
        - Bidirectional communication
        - Odometry implemented on Arduino
        - Implement the position controller
        - Implement servo control (with empty mechanism)
    ToDo:
        - Implement the state machine (goal: by 29.12.20)
        - Add Localization update on python (camera)
        - Implement a Kalman Filter
        - Add Bottle Detection to the Script
        """

def state_machine(t_s, t_max):
    # predefined waypoints


def get_time(time_start):
    return time.time() - time_start

def main(event, queue): # localisation
    # take image..(taking 0.3 seconds(
    event.set()




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

    time.sleep(4)
    if (ser.in_waiting>0):
        line = ser.readline()
        print("{:6.2f}".format(get_time(t_s)) + " [SER] Arduino: ", line)
        # if the received message is 'ready', then the arduino is in state 0 and well initialized
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [ERROR] Arduino is not responding")
    ser.flush()
    time.sleep(0.1)


    # MAIN LOOP HERE
    q_kalman = Queue()

    # beacon_start = mp.Event()
    beacon_completed = mp.Event()

    # bottle_detection_start = mp.Event()
    bottle_detection_completed = mp.Event()

    process_beacon = Process(target=main, args=(q_main,pose,event,,ser))
    process_beacon.start()






    waypoints = np.array([[2,1],[2,2]])
    i_wp = 0 # iterator over waypoints
    wp = np.array([0.5,0.5])

    # initial estimated position
    pose = np.array([0.5,0.5,0]) # estimated position
    state = 0 # state
    state_previous = 0

    while (time.time() - t_s < t_max):

        if (time.time() - t_s < 5):  # state = 1: track waypoints
            state = states.CATCH
            wp = np.array([0, 0])  # some random waypoint (doesn't matter)
        elif (time.time() - t_s < 15):
            state = states.EMPTY
            wp = np.array([0, 0])

            # if i_wp<len(waypoints):
            #     wp = waypoints[i_wp]
            #     if (abs(wp[0]-pose[0])<0.2) and (abs(wp[1]-pose[1])<0.2):
            #
            #         print("{:6.2f}".format(get_time(t_s)) + " [P-CTRL] waypoint ", waypoints[i_wp], " reached!")
            #         i_wp += 1
            #         # print("{:6.2f}".format(get_time(t_s)) + " [P-CTRL] new waypoint: ", waypoints[i_wp])
            # if i_wp==len(waypoints):
            #     wp = np.array([0.5,0.5])

        message = {}
        if (state != state_previous):
            message["state"] = state
            state_previous = state

        if (state == states.MOVING):
            wp = np.array([1, 1])  # need to define waypoints here
            message["ref"] = [float(wp[0], wp[1])]  # conversion to float is necessary!

        if (state == states.RETURN):
            wp = np.array([1, 1])  # need to define waypoints here
            message["ref"] = [float(wp[0], wp[1])]  # conversion to float is necessary!

        # write message to serial
        print("{:6.2f}".format(get_time(t_s)), "[SER] send: ", json.dumps(message))
        ser.write(json.dumps(message).encode('ascii'))

        ser.flush()
        time.sleep(0.09)  # TODO replace this with while

        if (ser.in_waiting > 0):  # if (len(line)>0):
            t0 = time.time()
            line = ser.readline().decode('ascii').rstrip()
            # line = ser.read(52).decode('utf-8').rstrip()
            # print("Time to read: ", time.time()-t0)
            ser.reset_input_buffer()
            # print(line)
            try:
                data = json.loads(line)
                pose = np.array(data["pos"])
                print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", int(data["state"]),
                      " pos: ", data["pos"], ", cmd:", data["cmd"], ", ref:", data["ref"],
                      " cnt: ", data["cnt"])
                # print("{:6.2f}".format(get_time(t_s)) + " [SER] state: ",data["state"], " pos:", data["pos"])
            except:
                print("[ERROR] cannot deserialize string from arduino. Received:", line)

        else:
            print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")

        if (process_beacon.)



    # time.sleep(0.1) # doen't work for time sleep <0.6!

    # shut the motor down
    state = states.FINISH
    wp_end = np.array([0.5,0.5])
    message = {"state": state}
    ser.write(json.dumps(message).encode('ascii'))
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Shutting motors down")
    time.sleep(1)
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Time elapsed. Program ending.")
    ser.close() # close serial   port at the end of the code