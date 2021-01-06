import serial
import time
import json
from numpy.linalg import norm
import numpy as np
import states
import localization
from multiprocessing import Lock, Process, Queue, current_process
from datetime import datetime
import os
import pandas as pd

import multiprocessing as mp
from localization import triangulation
from kalmanFilter import kalmanFilter, kf_get_param
from utilities import detect_bottle, ultrasound_dist_to_rel_pos, get_close_obstacles, waypoint_is_valid, is_obstacle, get_time
from picamera import PiCamera

# TODO: on arduino: after changing back from OBSTACLE to MOVING, the old reference is still given

""" This scripts implements a bidirectional communication at ca. 10 Hz
    Run this script together with 'bottlebot_complete' on Arduino.
    
    04.Jan.2020
"""

if __name__=='__main__':
    # display all np-floats with 2 decimals
    float_formatter = "{:.2f}".format
    np.set_printoptions(formatter={'float_kind': float_formatter})

    # initialize lists to generate logfile
    log_time = []
    log_pos = []
    log_cov = []
    log_ref = []
    log_info = []
    log_update = []

    ref = np.zeros(2)
    info = np.zeros(3)
    x_update = np.zeros(3)

    # define runtime (t_max), and time after which the robot returns to home (t_home)
    t_max = 300
    t_home = 300

    # initialize state of the robot
    state = states.STARTING
    state_previous = 0
    n_bottles = 0
    is_catching = False

    # initial estimated position
    pose = np.array([1,1,0]) # estimated position

    # get (initial) parameters of the Kalman Filter
    Pk, Q, R = kf_get_param()

    x = np.zeros(5)
    wp_bottle = np.array([1,1])
    wp_end = np.array([1,1])
    dT = 0

    data = "" # data string for serial communication

    # initalize time for display
    t_s = time.time()

    print("Start simulation. Duration: ", t_max ," seconds")
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

    p_bottle = mp.Process(target=detect_bottle, args=(q_bottle, e_bottle))
    p_triang = mp.Process(target=triangulation, args=(q_triang, e_img_loc, e_location, pose[2]))
    p_bottle.start()
    p_triang.start()
    pose_update_available = False

    pose_KF = np.empty(3)

    waypoints = [[6,1],[4,2],[5,4],[2,4],[7,3],[6,2],[6,4],[3,4],[6,4],[2,4]] #np.array()
    #i_wp = 0 # iterator over waypoints
    wp = np.array(waypoints[0])
    nav_tol = 0.4

    # tolerance on how close to track waypoints
    tracked_wp = []
    obst_list = []

    while (time.time() - t_s < t_max):
        message = {}
        if data == "": # send the position from python to arduino in the first iteration
            message["pose"] = [float(pose[0]), float(pose[1]), float(pose[2])]

        # We have seen a bottle 
        if (state == states.CATCH):
            message["ref"] = [float(wp_bottle[0]), float(wp_bottle[1])]

        # Calculate intermediate waypoint
        if (state_previous == states.OBSTACLE) and (state == states.MOVING):
            print("{:6.2f}".format(get_time(t_s)),"[MAIN] Obstacle avoided, redefine the tracked WP!")
            path_width = 0.4 # width of the tube along the desired direction in which there should be no obstacle

            # redefine the waypoint to track
            #1. find obstacles in vicinity
            obst_close = []
            des_angle = 0
            for o in obst_list:
                if norm(pose[0:-1]-o) < 1:
                    obst_close.append(o)

            if not obst_close:
                print("[OBST] WARNING: no obstacles found in vicinity... Leave waypoint unchanged")
            else:
                # there is an obstacle in vicinity to the desired path
                # 2. choose desired direction to current waypoint
                des_angle = np.arctan2(wp[0] - pose[0], wp[1] - pose[1]) # in [-pi, pi]
                c = np.cos(des_angle)
                s = np.sin(des_angle)
                path = [d*np.array([c, s]) for d in np.arange(0, 0.25, 1.1)]

                # 3. detect worst obstacle on the way
                minDistToObst = np.inf
                closestObst = None
                for obst in obst_close:
                    if (norm(obst-pose[0:2]) < minDistToObst):
                        minDistToObst = norm(obst-pose[0:2])
                        closestObst = obst

                if minDistToObst>path_width:
                    wp = pose[0:2] + path[-1]
                else:
                    # 4. set intermediate waypoint
                    obst_angle = np.arctan2(closestObst[0] - pose[0], closestObst[1] - pose[1]) # in  [-pi,pi]
                    R = np.array([[np.cos(obst_angle), -np.sin(obst_angle)], [np.sin(obst_angle), np.cos(obst_angle)]])

                    wp_CW = pose[0:2] + R.dot(np.array([0.75,0.75])) # clockwise
                    wp_CCW = pose[0:2] + R.dot(np.array([0.75,-0.75])) # counterclockwise

                    # random waypoint: turn by 90° or -90°
                    if (np.random.rand()>0.5):
                        new_wp = pose[0:2] + R.dot(np.array([-1,0]))
                    else:
                        new_wp = pose[0:2] + R.dot(np.array([1, 0]))

                    # TODO does not work at -pi!!
                    if (obst_angle>des_angle):
                        # set waypoint in clockwise direction along the path
                        if (waypoint_is_valid(wp_CW)):
                            angleToWP = obst_angle+np.pi/4
                            c = np.cos(angleToWP)
                            s = np.sin(angleToWP)
                            pathToNewWP = [d * np.array([c, s]) for d in np.arange(0, 0.25, 1.6)]
                            remaining_obst = get_close_obstacles(obst_close, pathToNewWP, 0.4)
                            if not remaining_obst:
                                # no obstacle blocks the waypoint!
                                new = wp_CW
                    else:
                        if (waypoint_is_valid(wp_CCW)):
                            angleToWP = obst_angle - np.pi / 4
                            c = np.cos(angleToWP)
                            s = np.sin(angleToWP)
                            pathToNewWP = [d * np.array([c, s]) for d in np.arange(0, 0.25, 1.6)]
                            remaining_obst = get_close_obstacles(obst_close, pathToNewWP, 0.4)
                            if not remaining_obst:
                                # no obstacle blocks the waypoint!
                                new = wp_CCW

                    #7. send waypoint and check if current waypoint has to be reached or bypassed
                    # TODO when to drop the current waypoint?
                    wp = new_wp
                    print("[MAiN] waypoint updated to:", wp)

        # Timeout expired: return to recycling station
        if (state != states.OBSTACLE) and (state!=states.EMPTY) and (time.time()- t_s > t_home):
            state_previous = state
            state = states.RETURN
            wp = wp_end
            message["ref"] = [float(wp[0]), float(wp[1])]

        # Empty the bottles in the recycling station
        if (state == states.RETURN):
            if (norm(pose[0:-1] - wp_end) < 0.4):
                state = states.EMPTY

        if (state == states.MOVING):  # state = 1: track waypoints
            if np.linalg.norm(pose[0:-1]-wp)<nav_tol:
                print("{:6.2f}".format(get_time(t_s)), " [MAIN] waypoint ", wp, " reached.")
                tracked_wp.append(wp)

                if waypoints: # there are still waypoints to track
                    # remove waypoint from the list and set it as current waypoint
                    wp = waypoints.pop(0)

                else:
                    # if there is still time, set some random waypoint
                    wp = np.random.randint(5,size=2)

            message["ref"] = [float(wp[0]), float(wp[1])]
            message["state"] = state
            # not updating previous state

        if (state != state_previous):
            if (state != states.CATCH) and (state_previous == states.CATCH):
                is_catching = False
                n_bottles += 1
                print("{:6.2f}".format(get_time(t_s)), " [MAIN] BOTTLE CATCHED! Robot contains ",
                      str(n_bottles), " bottles")
            message["state"] = state
            state_previous = state

        if (state == states.RETURN): # if state is returning, then send waypoints
            message["ref"] = [float(wp_end[0]), float(wp_end[1])]  # conversion to float is necessary!

        if (pose_update_available):
            message["pose"] = [float(pose[0]), float(pose[1]), float(pose[2])]
            pose_update_available = False

        # write message to serial
        #print("{:6.2f}".format(get_time(t_s)), "[SER] send: ", json.dumps(message))
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
                    pose = np.round(np.array(data["pos"]),2)
                    # round the pose to 2 decimals (pose may be sent via serial)
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
                if "info" in data:
                    info = np.array(data["info"])

                if "dist" in data: # exists only if Arduino is in obstacle avoidance
                    dist = np.array(data["dist"])
                    print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", state,
                          " pos: ", pose, " dist:", data["dist"], " info:", info)
                elif "ref" in data:
                    ref = np.round(np.array(data["ref"]),2)

                    print("{:6.2f}".format(get_time(t_s)) + " [SER] state:", state,
                          " pos: ", pose, " ref:", ref, " info:", info)

            except:
                print("[ERROR] cannot deserialize string from arduino.")

        else:
            print("{:6.2f}".format(get_time(t_s)) + " [SER] No message received :(")


        # CALCULATE OBSTACLE POSITONS:
        if (state == states.OBSTACLE):
            #print("[MAIN] try to append obstacles to list")
            min_obst_dist = 0.7 # take the same value as in Arduino code!
            radius_obstacle = 0.15 # radius of the obstacle size

            # calculate all frontal obstacles (sensors 2 to 5)
            for d,idx in zip(dist[1:7] ,range(0,7)):
                d = d/100 # rescale distance to meters
                if d<min_obst_dist:
                    c = np.cos(pose[2])
                    s = np.sin(pose[2])
                    R = np.array([[c, -s], [s, c]])
                    obstacle = pose[0:2]+R.dot(ultrasound_dist_to_rel_pos(d,idx))
                    already_obstacle = False
                    for obst in obst_list:
                        if (np.linalg.norm(obst-obstacle) < radius_obstacle):
                            already_obstacle = True

                    if not already_obstacle:
                        obst_list.append(obstacle)
                        print("obstacle appended at", obstacle)

                        # clear all waypoints close to that obstacle
                    for w in waypoints:
                        # clear precomputed waypoints that happen to be on obstacles
                        if np.linalg.norm(w-obstacle)<radius_obstacle:
                            waypoints.remove(w)
                    """
                    # clear bottle waypoint that happens to be on obstacles
                    if np.linalg.norm(wp_bottle-obstacle)<radius_obstacle:
                        wp_bottle.remove()
                    """

        # condition to read odometry when image is taken by webcam for localization
        if (e_img_loc.is_set()):
            v = 0
            omega = 0
            dT = 0.02
            try:
                measures = np.array(data["info"])
                v, omega, dT = measures
            except:
                print("Didn't manage to get info from arduino")
            pose_KF = pose
            x = np.array([pose_KF[0], pose_KF[1], pose_KF[2], v, omega])
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
                if not (is_catching):
                    is_catching = True
                    distanceToBottle = bottle_pos[0]
                    angle = bottle_pos[1]
                    bottle_x = pose[0] + (distanceToBottle)*np.cos(pose[2]+angle)
                    bottle_y = pose[1] + (distanceToBottle)*np.sin(pose[2]+angle)
                    bottle_ref = np.array([bottle_x, bottle_y])
                    if waypoint_is_valid(bottle_ref) and not is_obstacle(bottle_ref, obst_list):
                        state_previous = state 
                        state = states.CATCH
                        wp_bottle = np.round(bottle_ref,2)

            del q_bottle
            del e_bottle
            del p_bottle

            q_bottle = mp.Queue()
            e_bottle = mp.Event()
            p_bottle = mp.Process(target=detect_bottle, args=(q_bottle, e_bottle))
            p_bottle.start()
            # TODO send bottle detected to arduino and commands

        log_time.append(get_time(t_s))
        log_pos.append(pose)
        log_ref.append(ref)
        log_info.append(info)
        log_cov.append(Pk)
        log_update.append(x_update)




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

    print("Generating logfile...")

    log_data = {'time': log_time, 'pos': log_pos, 'ref': log_ref}


    dataframe = pd.DataFrame(log_data)
    if not os.path.exists('logs'):
        os.makedirs('logs')

    # log position data to .csv
    now = datetime.now()
    date_time = now.strftime("%m-%d-%Y_%H-%M-%S")
    logfile_name = os.path.basename(__file__)[:-3] + "_"+ date_time + ".csv"
    dataframe.to_csv("logs/"+logfile_name)

    # log obstacle to .csv
    log_obstacle_x = [o[0] for o in obst_list]
    log_obstacle_y = [o[1] for o in obst_list]
    log_obstacles = {'x': log_obstacle_x, 'y': log_obstacle_y}
    obstacle_logfilename = "obstacles_" + date_time + ".csv"
    obstacle_df = pd.DataFrame(log_obstacles)
    obstacle_df.to_csv('logs/'+obstacle_logfilename)

    print("Saved logfile to: logs/"+logfile_name)