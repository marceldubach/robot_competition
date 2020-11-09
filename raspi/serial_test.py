import serial
import time


if __name__=='__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    ser.flush()

    start_time = time.time()
    print("Start time:" + str(start_time))
    running = True
    ser.write(b"led\n")

    while(running):
        line = ser.readline().decode('utf-8').rstrip()
        # rstrip removes the '\n' at the end of the line
        print("received some string")
        print(line)
        try:
            potential = float(line)
        except ValueError:
            print("Not a float")
            potential = 0

        if (potential > 500):
            ser.write(b"led1\n")
        else:
            ser.write(b"led2\n")

        time.sleep(0.1)
        end_time = time.time()

        print("End time:" + str(end_time))
        if(end_time-start_time>10):
            running = False

