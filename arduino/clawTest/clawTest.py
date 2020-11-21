#!/test for the claw mechanism 19.11.2020
import serial
import sys
import time

if __name__ == '__main__':

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    line = "abc"
    arg = sys.argv
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            
        if line == "end":
            print("End of execution")

        if arg[1] == "start":
            ser.write(b"GO\n")
            print("Initialized bottle catching")
        time.sleep(1)
