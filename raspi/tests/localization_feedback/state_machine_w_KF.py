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
from utilities import detect_bottle
from picamera import PiCamera


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
def get_time(time_start):
    return time.time() - time_start


if __name__=='__main__':
    t_max = 180
    t_home = 120
    
    # initalize time for display
    t_s = time.time()

    # initial state
    state = states.STARTING
    state_previous = 0
    n_bottles = 0
    is_catching = False

    Pk = np.array([[0.5, 0, 0.02, 0.02, 0],
                   [0, 0.5, 0.02, 0.02, 0],
                   [0.02, 0.02, 0.1, 0, 0.04],
                   [0.02, 0.02, 0, 0.05, 0],
                   [0, 0, 0.04, 0, 0.02]])
    Q = 0.01*np.identity(5)
    R =  np.array([[0.5, 0, 0],
                  [0, 0.5, 0],
                  [0, 0, 0.05]])

    x = np.zeros(5)
    wp_bottle = np.array([0,0])
    wp_recycling_area = np.array([1,1])
    dT = 0

    # Picamera sensor matrix
    Z = np.array([[2714/2, 0, 640], [0, 2714/2, 360], [0, 0, 1]])
    Zi = np.linalg.inv(Z)
    x_c = 640
    y_c = 360
    r2 = Zi.dot([x_c, y_c, 1.0])

    # initial estimated position
    pose = np.array([1,1,0]) # estimated position

    data = ""

    print("Start simulation. Duration: ", t_max ," seconds")
    t_s = time.time()
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout = 0.5)

    if (ser.isOpen()):
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial successfully initialized")
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial connection failed")

    time.sleep(4)
    if (ser.in_waiting>0):
        try:
            line = ser.readline().decode('ascii') 
            print("{:6.2f}".format(get_time(t_s)) + " [SER] Arduino: ", line)
            state = states.MOVING
            # if the received message is 'ready', then the arduino is in state 0 and well initialized
        except:
             print("{:6.2f}".format(get_time(t_s)) + " [ERROR] Arduino is not responding")
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [ERROR] Arduino didn't send message")
    ser.flush()
    time.sleep(0.1)

    q_bottle = mp.Queue() # queue for frontal camera information
    q_triang = mp.Queue() # queue for triangulation
    e_bottle = mp.Event() # event when the frontal camera has finished
    e_img_loc = mp.Event() # event when an image is saved
    e_location = mp.Event() # event when triangulation has finished

    p_bottle = mp.Process(target=detect_bottle, args=(q_bottle, e_bottle, Zi, r2))
    p_triang = mp.Process(target=triangulation, args=(q_triang, e_img_loc, e_location, pose[2]))
    p_bottle.start()
    p_triang.start()
    pose_update_available = False
    bottle_detected = False

    pose_KF = np.empty(3)

    waypoints = np.array([[2,1],[4,4],[5,3], [1,1], [2,1]]) #TODO write function to calculate waypoints
    i_wp = 0 # iterator over waypoints
    wp = waypoints[i_wp]

    try:
        while (time.time() - t_s < t_max):

            message = {}
            if data == "":
                message["pose"] = [float(pose[0]), float(pose[1]), float(pose[2])]

            # Timeout expired: return to recycling station
            if (state == states.MOVING) and (time.time()- t_s > t_home):
                state_previous = state
                state = states.RETURN
                wp = wp_recycling_area
                message["ref"] = [float(wp[0]), float(wp[1])]

            # We have seen a bottle
            if (state == states.CATCH):
                message["ref"] = [float(wp_bottle[0]), float(wp_bottle[1])]

            # Back to recycling station
            if (state == states.RETURN):
                if (np.linalg.norm(pose[0:-1]-np.array([0.5,0.5]))<0.4):
                    state = states.EMPTY

            if (state == states.MOVING):  # state = 1: track waypoints
                if np.linalg.norm(pose[0:-1]-wp)<0.4:
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
                if (state== states.MOVING) and (state_previous == states.CATCH):
                    is_catching = False
                    n_bottles += 1
                message["state"] = state
                state_previous = state

            if (state == states.RETURN): # if state is returning, then send waypoints
                wp = wp_recycling_area # need to define waypoints here
                message["ref"] = [float(wp[0]), float(wp[1])]  # conversion to float is necessary!

            if (pose_update_available):
                message["pose"]  =[float(pose[0]),float(pose[1]),float(pose[2])]
                pose_update_available = False

            # write message to serial
            print("{:6.2f}".format(get_time(t_s)), "[SER] send: ", json.dumps(message))
            ser.write(json.dumps(message).encode('ascii'))

            ser.flush()
            while not (ser.in_waiting >0):
                time.sleep(0.05)  # TODO replace this with while

            # READ THE SERIAL INFORMATION FROM ARDUINO
            if (ser.in_waiting > 0):
                t0 = time.time()
                line = ser.readline().decode('ascii').rstrip()
                ser.reset_input_buffer()
                #ser.flush()
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
                          " pos: ", pose," ref:", data["ref"], " info:", data["info"])
                    # display detailed message:
                    # print(", cmd:", data["cmd"], ", ref:", data["ref"]," cnt: ", data["cnt"])

                except:
                    print("[ERROR] cannot deserialize string from arduino. Received:", line)

            else:
                print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")

            if (e_img_loc.is_set()):
                v = 0
                omega = 0
                dT = 0.02
                try:
                    measures = np.array(data["info"])
                    v,omega,dT = measures
                except:
                    print("Didn't manage to get info from arduino")
                pose_KF = pose
                if pose[2] > 2*np.pi:
                    pose_KF[2] -= 2*np.pi
                elif pose[2] < -2*np.pi:
                    pose_KF[2] += 2*np.pi
                x = np.array([pose_KF[0],pose_KF[1],pose_KF[2],v, omega])
                e_img_loc.clear()
                # print(x)

            if (e_location.is_set()):

                e_location.clear()

                measure = q_triang.get()
                print("measure:", measure)
                print("{:6.2f}".format(get_time(t_s)), "Time join start")
                p_triang.join()
                print("{:6.2f}".format(get_time(t_s)), "Time join end")

                if (measure[0]!=-1) and (measure[1]!=-1):
                    x_update, Pk = kalmanFilter(x,measure,dT,Pk,Q,R)
                    delta = pose - pose_KF
                    pose[0] = x_update[0] + delta[0]
                    pose[1] = x_update[1] + delta[1]
                    pose[2] = x_update[2] + delta[2]
                    pose_update_available = True
                    print("{:6.2f}".format(get_time(t_s)), "[KF] update position to ",pose)

                del q_triang
                del e_location
                del e_img_loc
                del p_triang

                q_triang = mp.Queue()
                e_location = mp.Event()
                e_img_loc = mp.Event()


            p_triang = mp.Process(target=triangulation, args=(q_triang, e_img_loc, e_location, pose[2]))
            p_triang.start()

        # frontal camera check
        if(e_bottle.is_set()):
            e_bottle.clear()
            bottle_pos = q_bottle.get()
            print("bottle position:", bottle_pos)
            p_bottle.join()
            if (bottle_pos[0] != -1 and bottle_pos[1] != -1):
                bottle_detected = True
                state_previous = state 
                state = states.CATCH
                distanceToBottle = bottle_pos[0]
                angle = bottle_pos[1]
                bottle_x = pose[0] + (distanceToBottle-0.1)*np.cos(pose[2]+angle)
                bottle_y = pose[1] + (distanceToBottle-0.1)*np.sin(pose[2]+angle)
                if not is_catching:
                    wp_bottle = np.array([bottle_x, bottle_y])
                    is_catching = True 
                # TODO do not chenge wp if not yet reached
            del q_bottle
            del e_bottle
            del p_bottle

            q_bottle = mp.Queue()
            e_bottle = mp.Event()
            p_bottle = mp.Process(target=detect_bottle, args=(q_bottle, e_bottle, Zi, r2))
            p_bottle.start()
            # TODO send bottle detected to arduino and commands

    finally:
        # shut the motor down
        state = states.FINISH
        wp_end = np.array([0.5,0.5])
        message = {"state": state}
        ser.write(json.dumps(message).encode('ascii'))
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Shutting motors down")
        time.sleep(1)
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Time elapsed. Program ending.")
        ser.close() # close serial port at the end of the code

        p_triang.join()
        p_bottle.join()



