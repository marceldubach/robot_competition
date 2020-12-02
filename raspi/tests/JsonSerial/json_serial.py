import time
import json
from serial import Serial

ser = Serial('/dev/ttyAMA0', 9600, timeout=1)
print("using port: ", ser.name)
print("start serial port connection...")
if (ser.isOpen()):
    print("Serial is open")
else:
    print("Serial not open")
t_start = time.time()
i = 0
ser.flush()
while True:
    print("send request")
    ser.write(bytes("poll"))
    print("request sent")
    time.sleep(1)
    rcv = ser.readLine()
    print(rcv)
    time.sleep(1)
    i += 1
    print(i)
    t_current = time.time()
    if (i>5):
        print("exciting")
        break






