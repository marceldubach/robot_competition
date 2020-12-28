#Imports
import math 
import numpy as np
import time
import cv2 as cv
import usb.core
import usb.util 
import imutils
import time 

def kalmanFilter(state_vector, measurements_vector, dT, ax, Pk, Q, R):
    """
    state_vector: column vector{
    x: x position in global reference [m]
    y: y position in global reference [m]
    yaw: absolute angle wrt global reference [rad]
    v: tangential velocity in robot's frame [m/s]
    omega: rotational velocity given by gyro [rad/s]
    }
    measurements_vector: column vector{
    x_beacons: absolute x position indicated by triangulation
    y_beacons: absolute y position indicated by triangulation
    yaw_beacons: absolute yaw angle 
    }
    dT: time elapsed between two measurements
    ax: accelerometer reading 
    Pk: state uncertainty at time k
    Q: environment uncertainty(untrucked noise)
    R: covariance of sensors noise 
    """
    yaw = state_vector[2]
    uk = ax
    A = np.array([[1, 0, 0, math.cos(yaw)*dT, 0], [0, 1, 0, math.sin(yaw)*dT, 0], [0, 0, 1, 0, dT], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])
    B = np.array([[0.5*math.cos(yaw)*(dT**2), 0.5*math.sin(yaw)*(dT**2), 0, dT, 0]]).transpose()
    C = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 1, 0, 0]])
    prediction = A.dot(state_vector) + B.dot(uk)
    Pk = (A.dot(Pk)).dot(A.transpose()) + Q
    K = (Pk.dot(C.transpose())).dot(np.linalg.inv((C.dot(Pk).dot(C.transpose()))+R))
    Pk = Pk - (K.dot(C)).dot(Pk)
    update = prediction + K.dot(measurements_vector - C.dot(prediction))
    return update , Pk