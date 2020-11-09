import serial
import time


if __name__=='__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    ser.flush()

    start_time = time.time()
    print("Start time:" + str(start_time))
    running = True

    while(running):
        if ser.in_waiting>0:
            line = ser.readline().decode('utf-8').rstrip()
            # rstrip removes the '\n' at the end of the line
            print("received some string")
            print(line)
            potential = float(line)
            if (potential > 500):
                ser.write(b"led\n")
            else:
                ser.write(b"led2\n")

        end_time = time.time()

        print("End time:" + str(end_time))
        if(end_time-start_time>10):
            running = False

