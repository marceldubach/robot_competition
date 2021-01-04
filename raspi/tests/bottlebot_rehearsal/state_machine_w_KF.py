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
    Run this script together with 'bottlebot_complete' on Arduino.
    
    04.Jan.2020
"""
def get_time(time_start):
    return time.time() - time_start


if __name__=='__main__':
    t_max = 60
    t_home =30
    
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
    wp_end = np.array([0.75,0.75])
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
        #ser.reset_input_buffer()
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial successfully initialized")
        state = states.MOVING
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] serial connection failed")


    ser.flush()

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

    pose_KF = np.empty(3)

    waypoints = np.array([[4,4],[7,1],[7,4],[4,4],[4,5],[1,3],[2,1],[4,2]])
    i_wp = 0 # iterator over waypoints
    wp = waypoints[i_wp]

    while (time.time() - t_s < t_max):
        message = {}
        if data == "": # send the position from python to arduino in the first iteration
            message["pose"] = [float(pose[0]), float(pose[1]), float(pose[2])]

        # We have seen a bottle 
        if (state == states.CATCH):
            message["ref"] = [float(wp_bottle[0]), float(wp_bottle[1])]

        # Timeout expired: return to recycling station
        if (state == states.MOVING) and (time.time()- t_s > t_home):
            state_previous = state
            state = states.RETURN
            wp = wp_end
            message["ref"] = [float(wp[0]), float(wp[1])]

        # Empty the bottles in the recycling station
        if (state == states.RETURN):
            if (np.linalg.norm(pose[0:-1]-wp_end)<0.4):
                state = states.EMPTY

        if (state == states.MOVING):  # state = 1: track waypoints
            if np.linalg.norm(pose[0:-1]-wp)<0.4:
                print("{:6.2f}".format(get_time(t_s)), " [MAIN] waypoint ", wp, " reached")
                i_wp += 1
                """
                ATTENTION HERE
                """
                if i_wp>len(waypoints): # if all waypoints are reached, shutdown
                    i_wp = 0
                    wp = waypoints[i_wp]
                    """
                    state_previous = state
                    state = states.FINISH
                    print("{:6.2f}".format(get_time(t_s)), " [MAIN] All waypoints are reached")
                    """
                else:
                    wp = waypoints[i_wp]  # some random waypoint (doesn't matter)
            message["ref"] = [float(wp[0]), float(wp[1])]
            message["state"] = state
            # not updating previous state

        if (state != state_previous):
            if (state == states.MOVING) and (state_previous == states.CATCH):
                is_catching = False
                n_bottles += 1
                print("{:6.2f}".format(get_time(t_s)), " [MAIN] BOTTLE CATCHED! Robot contains ",
                      str(n_bottles), " bottles")
            message["state"] = state
            state_previous = state
    

        if (state == states.RETURN): # if state is returning, then send waypoints
            message["ref"] = [float(wp_end[0]), float(wp_end[1])]  # conversion to float is necessary!

        if (pose_update_available):
            message["pose"]  =[float(pose[0]),float(pose[1]),float(pose[2])]
            pose_update_available = False

        # write message to serial
        print("{:6.2f}".format(get_time(t_s)), "[SER] send: ", json.dumps(message))
        ser.write(json.dumps(message).encode('ascii'))

        ser.flush()

        while not (ser.in_waiting > 0):
            time.sleep(0.02)

        # READ THE SERIAL INFORMATION FROM ARDUINO
        if (ser.in_waiting > 0):
            t0 = time.time()

            try:
                line = ser.readline().decode('ascii').rstrip()
                ser.reset_input_buffer()
                data = json.loads(line)
                if "pos" in data:
                    pose = np.array(data["pos"])
                if "state" in data:
                    if (state != data["state"]) and ((state != states.RETURN) and (state!=states.EMPTY)):
                        state_previous = state
                        state = data["state"]
                        print("{:6.2f}".format(get_time(t_s)) + " [MAIN] state changed to ",state)
                if "nBot" in data:
                    if (n_bottles != data["nBot"]):
                        n_bottles = data["nBot"]
                        print("{:6.2f}".format(get_time(t_s)) + "Bottle catched! Robot contains now ",
                              n_bottles, " bottles")

                if "dist" in data:
                    print("{:6.2f}".format(get_time(t_s)), "Distances ", data["dist"])


                print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", state,
                      " pos: ", pose," ref:", data["ref"], " info:", data["info"])


            except:
                print("[ERROR] cannot deserialize string from arduino.")

        else:
            print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")

        # condition to read odometry when image is taken by webcam for localization
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
            x = np.array([pose_KF[0],pose_KF[1],pose_KF[2],v, omega])
            e_img_loc.clear()

        # condition to know that localization is ready for sensor fusion update
        if (e_location.is_set()):
            
            e_location.clear()
            
            measure = q_triang.get()
            print("measure:", measure)
            print("{:6.2f}".format(get_time(t_s)), "Time join start")
            p_triang.join()
            print("{:6.2f}".format(get_time(t_s)), "Time join end")

            if (e_img_loc.is_set()):
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

        # frontal camera check when not in RETURN state
        if(e_bottle.is_set()) and (state == states.MOVING):
            e_bottle.clear()
            bottle_pos = q_bottle.get()
            print("bottle position:", bottle_pos)
            p_bottle.join()
            if (bottle_pos[0] != -1 and bottle_pos[1] != -1):
                state_previous = state 
                state = states.CATCH
                if not (is_catching):
                    is_catching = True
                    distanceToBottle = bottle_pos[0]
                    angle = bottle_pos[1]
                    bottle_x = pose[0] + (distanceToBottle-0.1)*np.cos(pose[2]+angle)
                    bottle_y = pose[1] + (distanceToBottle-0.1)*np.sin(pose[2]+angle)
                    if bottle_x > 0.5 and bottle_x < 7.5 and bottle_y > 0.5 and bottle_y < 7.5:
                        wp_bottle = np.array([bottle_x, bottle_y])

            del q_bottle
            del e_bottle
            del p_bottle

            q_bottle = mp.Queue()
            e_bottle = mp.Event()
            p_bottle = mp.Process(target=detect_bottle, args=(q_bottle, e_bottle, Zi, r2))
            p_bottle.start()
            # TODO send bottle detected to arduino and commands

    # shut the motor down
    state = states.FINISH
    message = {"state": state}
    ser.write(json.dumps(message).encode('ascii'))
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Shutting motors down")
    while not (ser.in_waiting > 0):
            time.sleep(0.02)

    # READ THE SERIAL INFORMATION FROM ARDUINO
    if (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('ascii').rstrip()
            ser.reset_input_buffer()
            print(line)
        except:
            print("Nothing received as last message")
    
    print("{:6.2f}".format(get_time(t_s)) + " [MAIN] Time elapsed. Program ending.")
    ser.reset_input_buffer()
    ser.close() # close serial port at the end of the code

    # join the processes to close them correctly
    p_triang.join()
    p_bottle.join()