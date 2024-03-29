#Imports
import math 
import numpy as np
import time
import cv2 as cv
import usb.core
import usb.util 
import imutils
import time 

def kf_get_param():
    Pk = np.diag([0.5, 0.5, 0.4, 0.1, 0.05])
    Q = np.diag([0.2, 0.2, 0.5, 0.02, 0.1])
    R = np.diag([0.5, 0.5, 0.05])
    return Pk, Q, R

def kalmanFilter(prediction, measurements_vector, dT, Pk, Q, R):
    """
    prediction: column vector{
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
    Pk: covariance of state uncertainty at time k
    Q: covariance of process noise 
    R: covariance of measurements noise  
    """
    yaw = prediction[2]
    measured_angle = measurements_vector[2]

    # deal with singularities in angles (i.e 0.01 rad and 6.27 rad around 2pi rad)
    if abs(measured_angle - yaw) > np.pi:
        if measured_angle - yaw > 0:
            measurements_vector[2] -= 2*np.pi
        elif yaw - measured_angle > 0:
            prediction[2] -= 2*np.pi

    A = np.array([[1, 0, 0, math.cos(yaw)*dT, 0], [0, 1, 0, math.sin(yaw)*dT, 0], [0, 0, 1, 0, dT], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])
    C = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 1, 0, 0]])

    Pk = (A.dot(Pk)).dot(A.transpose()) + Q
    K = (Pk.dot(C.transpose())).dot(np.linalg.inv((C.dot(Pk).dot(C.transpose()))+R))
    Pk = Pk - (K.dot(C)).dot(Pk)
    update = prediction + K.dot(measurements_vector - C.dot(prediction))
    if update[2] < 0:
        update[2] += 2*np.pi
    return update , Pk