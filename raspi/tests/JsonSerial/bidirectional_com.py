import serial
import time
import json

""" This scripts implements a bidirectional communication at LOW RATE (around 0.9 Hz)"""
def get_time(time_start):
    return time.time() - time_start

def write_to_serial(serial, state, cmdLeft, cmdRight):
    message = {"state": state, "cmd": [cmdLeft, cmdRight]}
    serial.write(json.dumps(message).encode('ascii'))

if __name__=='__main__':
    t_max = 20
    print("Simulation runs for ", t_max ," seconds")
    t_s = time.time()
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 0.6)

    if (ser.isOpen()):
        print("{:6.2f}".format(get_time(t_s)) + " [SER] serial successfully initialized")
    else:
        print("{:6.2f}".format(get_time(t_s)) + " [SER] connection failed")
    ser.flush()

    state = 1
    cmdLeft = 0
    cmdRight = 0

    t_s = time.time()
    sleeping = 0.5 # code does not work for time sleep below 0.5s

    # MAIN LOOP HERE
    while(time.time()-t_s < t_max):

        cmdLeft = 10
        cmdRight = 20
        write_to_serial(ser, state, cmdLeft, cmdRight)

        line = ser.readline().decode('ascii').rstrip()
        if (len(line)>0):
            print("{:6.2f}".format(get_time(t_s)) + " [SER] Received: ",line)
        else:
            print("{:6.2f}".format(get_time(t_s)) + " [SER] Error: Received string of length "+str(len(line)))

        time.sleep(sleeping)

    print("Time elapsed. Program ending.")
    ser.close() # close serial   port at the end of the code