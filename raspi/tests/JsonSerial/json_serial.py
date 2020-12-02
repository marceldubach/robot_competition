import time
import json
from serial import Serial

ser = Serial('/dev/ttyAMA0', 9600, timeout=1)
print("using port: ", ser.name)
print("start serial port connection...")
t_start = time.time()
while True:
    print("send request")
    ser.write("poll".encode())
    rcv = ser.readLine()
    print(rcv)
    time.sleep(1)

    t_current = time.time()
    if (t_current-t_start)>10:
        print("exciting")
        break






