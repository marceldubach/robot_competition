"""
The purpose of this script is to test the bidirectional Serial connection
between Arduino and Raspberry Pi using Json Documents for exchange.

use it together with the Arduino script 'serial_test.ino' stored it in ...
created: 23.dec.2020 by Marcel Dubach
"""

import time
import json
from serial import Serial
import numpy as np

# set the same baudrate as in Arduino script!!!
ser = Serial('/dev/ttyAMA0', 9600, timeout=1)
print("[SER] selected port: ", ser.name)
if (ser.isOpen()):
    print("[SER] serial connection is open")
else:
    print("[SER] serial connection failed")

start_time = time.time()
ser.flush()
send_times = np.random.rand(10)+ np.arange(0,10)
print("send times:", send_times)
i = 0
while(time.time()<start_time+10):
    if ser.in_waiting>0:
        decoded = json.loads(ser.readline())
        print(decoded)

    if (send_times[i]<time.time()):

        ser.write(str(i*10).encode(encoding='UTF-8'))
        print("send delay:", time.time()-send_times[i])
        i+=1

ser.close()



