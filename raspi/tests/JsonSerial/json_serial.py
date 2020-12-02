import time
import json
import serial

ser = serial.Serial(port='dev/ttyS0', baudrate=9600)

print("start serial port connection...")
t_start = time.time()
while True:
    print("send request")
    ser.write("poll")
    rcv = ser.readLine()
    print(rcv)
    time.sleep(1)

    t_current = time.time()
    if (t_current-t_start)>10:
        print("exciting")
        break
        





