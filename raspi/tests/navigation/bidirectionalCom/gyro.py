# Imports
import numpy as np
import math

#Assumption 50Hz signals
def angleComputation(psi, gz, dt):
    # psi: yaw angle of previous iteration [rad]
    # gz: angular velocity rate with respect to z [°/sec] 
    # Transform gz from [°/s] in [rad/s]
    gz = math.pi*gz/180 
    newPsi = psi + gz*dt
    return newPsi
