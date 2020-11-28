#Imports
import math 
import numpy as np
import cv2 as cv
import usb.core
import usb.util 
import imutils
import time 
import os

def imageAcquisition():
    #Find device 
    webcam = cv.VideoCapture(0) #ID 0

    if not (webcam.isOpened()):
        print("Could not open video device")

    try:
        check, frame = webcam.read()
        if not os.path.exists('images'):
            os.makedirs('images')
        cv.imwrite(filename='images/beacons.jpg', img=frame)
        webcam.release()
        print("Image saved!")
    except:
        print("Problem saving image")

def centroidsExtraction():
    #CHECK IF IMAGE ROTATION IS NEEDED

    # load image BGR
    img_BGR = cv.imread('images/beacons.jpg')  

    # convert to grayscale
    gray = cv.cvtColor(img_BGR, cv.COLOR_BGR2GRAY)

    # masking region of no interest -> everything outside sphere and inside certain radius 
    # detect circles (attention to parameters)
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 2.1, 500, minRadius = 300, maxRadius = 700) 

    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (xc, yc, r) in circles:
            # TO VERIFY
            # draw the inner circle on the BGR image and mask out inner region
            cv.circle(gray, (xc, yc), int(0.7*r), (0, 255, 0), -1)
            # extract mask
            ret,mask = cv.threshold(gray,10,255, cv.THRESH_BINARY) 
            # apply mask 
            result_mask = cv.bitwise_and(img_BGR, img_BGR, mask=mask)
            # draw the circle on the BGR image and mask out outer region
            cv.circle(gray, (xc, yc), r, (0, 255, 0), -1)
        # extract mask
        ret,mask = cv.threshold(gray,10,255, cv.THRESH_BINARY_INV)
        # apply mask 
        result_mask = cv.bitwise_and(result_mask, result_mask, mask=mask)
    else:
        # if no circle detected 
        result_mask = img_BGR
        
    # remove some noise 
    blurred = cv.GaussianBlur(result_mask, (5,5), 0)

    # Convert BGR to HSV
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([0,50,50])  
    upper_red = np.array([20,255,255])
    lower_red2 = np.array([160,50,50])  
    upper_red2 = np.array([180,255,255])

    # define range of yellow color in HSV
    lower_yellow = np.array([20,30,50])
    upper_yellow = np.array([60,255,255])

    # define range of green color in HSV
    lower_green = np.array([50,30,50])
    upper_green = np.array([100,255,255])

    # define range of blue color in HSV
    lower_blue = np.array([100,50,50])
    upper_blue = np.array([140,255,255])

    # Threshold the HSV image to get only yellow colors
    mask_Yellow = cv.inRange(hsv, lower_yellow, upper_yellow)

    # Threshold the HSV image to get only yellow colors
    mask_Red = cv.inRange(hsv, lower_red, upper_red)
    mask_Red2 = cv.inRange(hsv, lower_red2, upper_red2)

    # Threshold the HSV image to get only green colors
    mask_Green = cv.inRange(hsv, lower_green, upper_green)

    # Threshold the HSV image to get only blue colors
    mask_Blue = cv.inRange(hsv, lower_blue, upper_blue)

    # TO VERIFY
    # highlight only bright elements in result_mask
    thresh_res = cv.threshold(result_mask, 240, 255, cv.THRESH_BINARY)[1]

    # Bitwise-AND mask 
    res_Y = cv.bitwise_and(thresh_res,thresh_res, mask= mask_Yellow)

    # Bitwise-AND mask 
    res_R = cv.bitwise_and(thresh_res,thresh_res, mask= mask_Red)

    # Bitwise-AND mask 
    res_R2 = cv.bitwise_and(thresh_res,thresh_res, mask= mask_Red2)

    # Bitwise-AND mask 
    res_G = cv.bitwise_and(thresh_res,thresh_res, mask= mask_Green)

    # Bitwise-AND mask 
    res_B = cv.bitwise_and(thresh_res,thresh_res, mask= mask_Blue)

    res_Y = cv.cvtColor(res_Y,cv.COLOR_BGR2GRAY)
    res_R = cv.cvtColor(res_R,cv.COLOR_BGR2GRAY)
    res_R2 = cv.cvtColor(res_R2,cv.COLOR_BGR2GRAY)
    res_G = cv.cvtColor(res_G,cv.COLOR_BGR2GRAY)
    res_B = cv.cvtColor(res_B,cv.COLOR_BGR2GRAY)

    masks = np.array([res_Y, res_R, res_R2, res_G, res_B])
    colors = np.array(["yellow", "red", "red", "green", "blue"])
    centroids = []

    for mask, color in zip(masks, colors):

        # morphology operation on the mask
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((3,3),np.uint8))
        # find contours in the thresholded image
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv.moments(c) 
            if(M["m00"] > 100): 
                cX = int(M["m10"] / M["m00"]) 
                cY = int(M["m01"] / M["m00"]) 
                centroids.append([cX, cY, color])
    return centroids

# compute position
def computeC(centroids, img):
    if len(centroids) < 3:
        print("Not enough beacons")
        return 1
    else:
        if len(centroids) > 4:
            print("Too many beacons")
            return 1
        else:

            c1 = centroids[0]
            c2 = centroids[1]
            c3 = centroids[2]
            xc = np.shape(img)[1]/2
            yc = np.shape(img)[0]/2
            depAlpha = 0
            depBeta = 0
            if (c1[0]<xc and c1[1]<yc and c2[0]<xc and c2[1]<yc):
                pass
            elif (c1[0]<xc and c1[1]<yc):
                depBeta = 2*np.pi

            if (c2[0]<xc and c2[1]<yc and c3[0]<xc and c3[1]<yc):
                pass
            elif (c2[0]<xc and c2[1]<yc):
                depAlpha = 2*np.pi

            beta = np.arctan2((c1[1]-yc),(c1[0]-xc)) - np.arctan2((c2[1]-yc),(c2[0]-xc)) + depBeta
            alpha = np.arctan2((c2[1]-yc),(c2[0]-xc)) - np.arctan2((c3[1]-yc),(c3[0]-xc)) + depAlpha
            xA = 4
            yA = 4/math.tan(alpha)
            xB = 4/math.tan(beta)
            yB = 4
            rA = 4/math.sin(alpha)
            rB = 4/math.sin(beta)
            M = (xA-xB)/(yA-yB)
            N = (math.pow(rB,2) - math.pow(rA,2) - math.pow(xB,2) + math.pow(xA,2) - math.pow(yB, 2) + math.pow(yA,2))/(2*(yA-yB))
            A = math.pow(M,2)+1
            B = 2*yA*M - 2*M*N - 2*xA
            C = math.pow(xA,2) + math.pow(yA,2) + math.pow(N,2) - math.pow(rA,2) - 2*yA*N
            discriminant = abs(math.pow(B,2) - 4*A*C)
            x1 = (-B + math.sqrt(discriminant))/(2*A)
            y1 = N-x1*M
            x2 = (-B - math.sqrt(discriminant))/(2*A)
            y2 = N-x2*M
            if(x1>pow(10,-3) and x1<8 and y1>pow(10,-3) and y1<8):
                return x1,y1
            else:
                return x2,y2
