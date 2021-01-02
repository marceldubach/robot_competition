import serial
import time
import json
import numpy as np
import states
import localization
from multiprocessing import Lock, Process, Queue, current_process
import multiprocessing as mp
from localization import triangulation
from kalmanFilter import kalmanFilter


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
"""
def state_machine(t_s, t_max):
    # predefined waypoints
"""

def get_time(time_start):
    return time.time() - time_start

def main(event, queue): # localisation
    # take image..(taking 0.3 seconds(
    event.set()




if __name__=='__main__':
    t_max = 30
    # initalize time for display
    t_s = time.time()

    # initial state
    state = states.STARTING
    state_previous = 0
    n_bottles = 0

    Pk = np.array([[0.1, 0, 0.02, 0.02, 0],
                   [0, 0.1, 0.02, 0.02, 0],
                   [0.02, 0.02, 0.1, 0, 0.04],
                   [0.02, 0.02, 0, 0.05, 0],
                   [0, 0, 0.04, 0, 0.02]])
    Q = 0.01 * np.identity(5)
    R = np.array([[0.1, 0, 0],
                  [0, 0.1, 0],
                  [0, 0, 0.01]])
    x = np.zeros(5)
    dT = 0

    # initial estimated position
    pose = np.array([0.5,0.5,0]) # estimated position

    print("Start simulation. Duration: ", t_max ," seconds")
    t_s = time.time()
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout = 0.5)

    if (ser.isOpen()):
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial successfully initialized")
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial connection failed")

    webcam = localization.setupWebcam()

    time.sleep(3)
    if (ser.in_waiting>0):
        line = ser.readline().decode('ascii') # TODO check if this decode works
        print("{:6.2f}".format(get_time(t_s)) + " [SER] Arduino: ", line)
        state = states.MOVING
        # if the received message is 'ready', then the arduino is in state 0 and well initialized
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [ERROR] Arduino is not responding")
    ser.flush()
    time.sleep(0.1)

    """
        # MAIN LOOP HERE
        q_kalman = Queue()
    
        # beacon_start = mp.Event()
        beacon_completed = mp.Event()
    
        # bottle_detection_start = mp.Event()
        bottle_detection_completed = mp.Event()
    
        process_beacon = Process(target=main, args=(q_main,pose,event,,ser))
        process_beacon.start()
    """
    q_triang = mp.Queue()
    e_img_loc = mp.Event()
    e_location = mp.Event()

    p_triang = mp.Process(target=triangulation, args=(q_triang, e_img_loc, e_location, pose[2],webcam))
    p_triang.start()
    pose_update_available = False

    waypoints = np.array([[2,1],[2,2]]) #TODO write function to calculate waypoints
    i_wp = 0 # iterator over waypoints
    wp = waypoints[i_wp]

    while (time.time() - t_s < t_max):
        message = {}

        if (state == states.MOVING):  # state = 1: track waypoints
            if np.linalg.norm(pose[0:-1]-wp)<0.2:
                print("{:6.2f}".format(get_time(t_s)), " [MAIN] waypoint ", wp, " reached")
                i_wp += 1
                if i_wp>len(waypoints): # if all waypoints are reached, shutdown
                    state_previous = state
                    state = states.FINISH
                    print("{:6.2f}".format(get_time(t_s)), " [MAIN] All waypoints are reached")
                else:
                    wp = waypoints[i_wp]  # some random waypoint (doesn't matter)
            message["ref"] = [float(wp[0]), float(wp[1])]

        if (state != state_previous):
            message["state"] = state
            state_previous = state

        if (state == states.RETURN): # if state is returning, then send waypoints
            wp = np.array([1, 1])  # need to define waypoints here
            message["ref"] = [float(wp[0]), float(wp[1])]  # conversion to float is necessary!

        if (pose_update_available):
            message["pose"]  =[float(pose[0]),float(pose[1]),float(pose[2])]
            pose_update_available = False

        # write message to serial
        print("{:6.2f}".format(get_time(t_s)), "[SER] send: ", json.dumps(message))
        ser.write(json.dumps(message).encode('ascii'))

        ser.flush()
        time.sleep(0.09)  # TODO replace this with while

        # READ THE SERIAL INFORMATION FROM ARDUINO
        if (ser.in_waiting > 0):
            t0 = time.time()
            line = ser.readline().decode('ascii').rstrip()
            ser.reset_input_buffer()
            # print(line)
            try:
                data = json.loads(line)
                if "pos" in data:
                    pose = np.array(data["pos"])
                if "state" in data:
                    if state != data["state"]:
                        state_previous = state
                        state = data["state"]
                        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] state changed to ",state)
                if "nBot" in data:
                    if (n_bottles != data["nBot"]):
                        n_bottles = data["nBot"]
                        print("{:6.2f}".format(get_time(t_s)) + "Bottle catched! Robot contains now ",
                              n_bottles, " bottles")

                print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", state,
                      " pos: ", pose," ref:", data["ref"], " cmd: ", data["cmd"])
                # display detailed message:
                # print(", cmd:", data["cmd"], ", ref:", data["ref"]," cnt: ", data["cnt"])

            except:
                print("[ERROR] cannot deserialize string from arduino. Received:", line)

        else:
            print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")

        if (e_img_loc.is_set()):
            v = data["info"][0]
            omega = data["info"][1]
            x = np.array([pose[0],pose[1],pose[2],v, omega])
            dT = data["info"][2]

        if (e_location.is_set()):
            p_triang.join()

            x_update, Pk = kalmanFilter(x,q_triang.get(),dT,Pk,Q,R)
            if (x_update[0]!=-1) and (x_update[1]!=-1):
                pose[0] = x_update[0]
                pose[1] = x_update[1]
                pose[2] = x_update[2]
                pose_update_available = True

            q_triang = mp.Queue()
            e_img_loc = mp.Event()
            e_location = mp.Event()

            p_triang = mp.Process(target=triangulation, args=(q_triang, e_img_loc, e_location, pose[2], webcam))
            p_triang.start()


    # shut the motor down
    state = states.FINISH
    wp_end = np.array([0.5,0.5])
    message = {"state": state}
    ser.write(json.dumps(message).encode('ascii'))
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Shutting motors down")
    time.sleep(1)
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Time elapsed. Program ending.")
    ser.close() # close serial   port at the end of the code