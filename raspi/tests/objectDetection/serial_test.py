import serial
import time


if __name__=='__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    time.sleep(0.1)
    ser.flush()

    start_time = time.time()
    print("Program started")
    running = True
    # Arduino replies to any commands. Give an initial command


    while(running):
        ser.write("where\n".encode('utf-8'))
        line = ser.readline().decode('utf-8').rstrip()
        # rstrip removes the '\n' at the end of the line
        print("Received odometry value: "+line)
        # try:
        #     potential = float(line)
        # except ValueError:
        #     print("Not a float - set potential to zero")
        #     potential = 0

        # if (potential > 500):
        #     ser.write("led1\n".encode('utf-8'))
        # else:
        #     ser.write("led2\n".encode('utf-8'))

        time.sleep(0.2)
        end_time = time.time()

        print("Elapsed time" + str(end_time-start_time))
        if(end_time-start_time>10):
            running = False

