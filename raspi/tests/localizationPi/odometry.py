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
    xDot = np.array([[linearV, 0, rotV]]).transpose()
    invRotMatrix = np.array([[math.cos(pose[2][0]), -math.sin(pose[2][0]), 0], [math.sin(pose[2][0]), math.cos(pose[2][0]), 0], [0, 0, 1]])
    newPose = pose + invRotMatrix.dot(xDot)*dt
    return newPose 


def odometryGyro(gz, r, pose, omega, dt):
    # pose = [x, y, theta] with respect to arena reference frame [m, m, rad] as numpy array (column vector) 
    # omega = [wl, wr] [rad/s] as a list
    # gz: gyro rate [°/s]
    # r: wheel radius [m]
    # t: elapsed time between two measurements(inverse of loop frequency) [s]
    vl = omega[0]*r
    vr = omega[1]*r
    linearV = (vl+vr)/2
    rotV = gz*math.pi/180
    xDot = np.array([[linearV, 0, rotV]]).transpose()
    invRotMatrix = np.array([[math.cos(pose[2][0]), -math.sin(pose[2][0]), 0], [math.sin(pose[2][0]), math.cos(pose[2][0]), 0], [0, 0, 1]])
    newPose = pose + invRotMatrix.dot(xDot)*dt
    return newPose 

def odometryGyroAccel(gz, accel, r, pose, omega, dt):
    # pose = [x, y, theta] with respect to arena reference frame [m, m, rad] as numpy array (column vector) 
    # omega = [wl, wr] [rad/s] as a list
    # gz: gyro rate [°/s]
    # r: wheel radius [m]
    # t: elapsed time between two measurements(inverse of loop frequency) [s]
    vl = omega[0]*r
    vr = omega[1]*r
    linearV = (vl+vr)/2
    rotV = gz*math.pi/180
    xDot = np.array([[linearV, 0, rotV]]).transpose()
    xDotDot = np.array([accel, 0, 0]).transpose()
    invRotMatrix = np.array([[math.cos(pose[2][0]), -math.sin(pose[2][0]), 0], [math.sin(pose[2][0]), math.cos(pose[2][0]), 0], [0, 0, 1]])
    newPose = pose + invRotMatrix.dot(xDot)*dt + 0.5*invRotMatrix.dot(xDotDot)*(dt**2)
    return newPose 

def velocity(v, ax, dt):
    # update velocity integrating acceleration
    # v: linear velocity [m/s]
    # ax: accelerometer reading [m/s^2]
    # dt: delta time [s]
    v = v + ax*dt
    return v