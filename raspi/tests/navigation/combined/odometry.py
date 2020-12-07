# Imports
import numpy as np
import math

#Assumption: no wheel slip 
def odometry(l, r, pose, omega, dt):
    # pose = [x, y, theta] with respect to arena reference frame [m, m, rad] as numpy array (column vector) 
    # omega = [wl, wr] [rad/s] as a list
    # l: half of robot width or distance from wheel to central axis [m]
    # r: wheel radius [m]
    # t: elapsed time between two measurements(inverse of loop frequency) [s]
    vl = omega[0]*r
    vr = omega[1]*r
    linearV = (vl+vr)/2
    rotV = (vl-vr)/(2*l) 
    dotVector = np.array([[linearV, 0, rotV]]).transpose()
    invRotMatrix = np.array([[math.cos(pose[2][0]), -math.sin(pose[2][0]), 0], [math.sin(pose[2][0]), math.cos(pose[2][0]), 0], [0, 0, 1]])
    newPose = pose + invRotMatrix.dot(dotVector)*dt
    return newPose 