#Imports
import math 
import numpy as np
import time
import cv2 as cv
import imutils
import time
import json
import os

def triangulation(queue, e_img_loc, e_loc_finished, yaw):
    """
    Function called in the python process p_triang(), it sets up the webcam, checks if the 
    webcam has been well opened, takes a picture and then computes the robot's center of mass 
    position from the extracted beacon centroids.
    The event e_img_loc is set right after the picture has been taken to tell the main program 
    to stock the current odometry in a variable that will be used as prediction for the 
    Kalman filter, while the e_loc_finished event is set when the function completes in order
    to tell the main program that it can proceed with the sensor fusion and reinitialize the 
    process for the next image.
    The data containing the absolute pose of the robot is passed to the main program through 
    the queue.
    """
    filename = 0
    webcam = cv.VideoCapture(0) 
    webcam.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
    webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
    time.sleep(0.3)
    try:
        if (webcam.isOpened()):
            filename = savePicture(webcam)
            e_img_loc.set()
            centroids = extractCentroids(filename)
            xCenterM, yCenterM, yaw = computePosition(centroids, yaw)
            data = np.array([xCenterM, yCenterM, yaw])
            queue.put(data)
        else:
            raise ValueError("[TRIANG] Video Device is not open")

    except ValueError as ve:
        print(ve)

    finally:
        webcam.release()
        if (e_img_loc.is_set()):
            e_loc_finished.set()
        # if fails the queue will be empty
        return queue 

def savePicture(webcam):
    """
    Function that uses the object webcam as argument for taking and saving a picture 
    in the Raspberry Pi memory. The image is overwritten at each function call.
    """
    try:
        check, frame = webcam.read()
        if check:
            filename = 'img.jpg'
            time.sleep(0.1)
            cv.imwrite(filename, img=frame)
        else:
            filename = 0
    except:
        print("Problem saving image")
        filename = 0
    return filename


def extractCentroids(filename):
    """
    Reads the image associated to the filename, converts to grayscale and computes the masks 
    on the regions of no interest, converts the image in the HSV space and applies the inRange 
    threshold to extract targeted colors. IThen it calculates the contours around the color
    masks and if multiple clusters of the same color are found, it merges the clusters that 
    are closer to each other than 100 pixels and then it chooses the most important cluster 
    for such color. Finally, the centroids of the beacons are returned.
    """
    # load image BGR
    img_BGR = cv.imread(filename)  

    # rotate images
    img_BGR_flip = cv.flip(img_BGR,1)
    output = img_BGR_flip.copy()

    # convert to grayscale
    gray = cv.cvtColor(img_BGR_flip, cv.COLOR_BGR2GRAY)

    # masking region of no interest 
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1.4, 1000, minRadius = 400, maxRadius = 650)

    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        for (xc, yc, r) in circles:
            # draw the inner circle 
            cv.circle(gray, (xc, yc), int(0.6*r), (0, 255, 0), -1)
            # extract mask
            ret,mask = cv.threshold(gray,10,255, cv.THRESH_BINARY) 
            # apply mask 
            result_mask = cv.bitwise_and(img_BGR_flip, img_BGR_flip, mask=mask)
            # draw the outer circle 
            cv.circle(gray, (xc, yc), int(r), (0, 255, 0), -1)
            # extract mask
            ret,mask = cv.threshold(gray,10,255, cv.THRESH_BINARY_INV) 
            # apply mask 
            result = cv.bitwise_and(result_mask, result_mask, mask=mask)
    else:
        result = img_BGR_flip

    # remove some noise 
    blurred = cv.GaussianBlur(result, (5,5), 0)

    # convert BGR to HSV
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([0,130,120])  
    upper_red = np.array([15,255,255])
    lower_red2 = np.array([170,80,100])  
    upper_red2 = np.array([180,255,255])

    # define range of magenta color in HSV
    lower_magenta = np.array([120,160,100])
    upper_magenta = np.array([160,255,255])

    # define range of green color in HSV
    lower_green = np.array([45,150,70])
    upper_green = np.array([90,255,255])

    # define range of blue color in HSV
    lower_blue = np.array([108,130,70])
    upper_blue = np.array([110,255,255])

    # Threshold the HSV image to extract only magenta colors
    mask_Magenta = cv.inRange(hsv, lower_magenta, upper_magenta)

    # Threshold the HSV image to extract only red colors
    mask_Red = cv.inRange(hsv, lower_red, upper_red)
    mask_Red2 = cv.inRange(hsv, lower_red2, upper_red2)

    # Threshold the HSV image to extract only green colors
    mask_Green = cv.inRange(hsv, lower_green, upper_green)

    # Threshold the HSV image to extract only blue colors
    mask_Blue = cv.inRange(hsv, lower_blue, upper_blue)

    sum_Red = mask_Red + mask_Red2
    masks = np.array([mask_Magenta, sum_Red, mask_Green, mask_Blue])
    colors = np.array(["magenta", "red", "green", "blue"])
    centroids = []

    for mask, color in zip(masks, colors):
        
         # weaker detections need to be enhanced 
        if color != "magenta":
            mask = cv.dilate(mask,np.ones((5,5)),iterations = 1)
            
        # morphology operation on the mask
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5,5),np.uint8))
        kernel = np.ones((5,5))
        mask = cv.dilate(mask,kernel,iterations = 3)
        # find contours in the thresholded image
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # coordinates of centroid
        centroid_X = 0
        centroid_Y = 0
        # idx to choose most important
        idx = 0
        # lists to define cluster importance 
        mom0 = []
        mom10 = []
        mom01 = []
        
        if cnts != []:
            # loop over the contours
            for c in cnts:
                # compute the center of the contour
                M = cv.moments(c) 
                # reject noise and light artifacts
                if(M["m00"] > 100): 
                    cX = int(M["m10"] / M["m00"]) 
                    cY = int(M["m01"] / M["m00"]) 
                    # in the case of multiple close centroids for one color sum up
                    if(np.sqrt((centroid_X - cX)**2 + (centroid_Y - cY)**2) < 100):
                        centroid_X += cX
                        centroid_Y += cY
                        mom0[-1] += M["m00"]
                        mom10[-1] += M["m10"]
                        mom01[-1] += M["m01"]
                    else:
                        centroid_X = cX
                        centroid_Y = cY
                        mom0.append(M["m00"])
                        mom10.append(M["m10"])
                        mom01.append(M["m01"])
                    #get the index of the most important color cluster
                    idx = mom0.index(max(mom0))
                    cX = int(mom10[idx] / mom0[idx])
                    cY = int(mom01[idx] / mom0[idx]) 
                    if(color == 'red'):
                        clr = [255, 0, 0]
                    elif(color == 'green'):
                        clr = [0, 255, 0]
                    elif(color == 'blue'):
                        clr = [0, 0, 255]
                    elif(color == 'magenta'):
                        clr = [255, 0, 255]
            # do not count features inside or outside certain image radius 
            if(np.sqrt((cX-960)**2 + (cY-540)**2) < 280 or np.sqrt((cX-960)**2 + (cY-540)**2) > 580):
                pass
            else:
                text = "{} {}".format(color, "beacon")
                # draw the contour and center of the shape on the image
                cv.drawContours(output, [cnts[idx]], -1, clr, 2)
                cv.putText(output, text, (cX, cY - 50) ,cv.FONT_HERSHEY_SIMPLEX, 3, clr, 5)
                cv.circle(output, (cX, cY), 10,  (0, 0, 0), -1)
                cv.putText(output, "center", (cX + 20, cY + 20),cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 5)
                centroids.append([cX, cY, color])
    return centroids

def getReference(centroids):
    """
    Understand which beacon centroids are available and so choose 
    a suitable reference to be consistent 
    """
    red = 0
    magenta = 0
    blue = 0
    green = 0
    if centroids != []:
        for c in centroids:
            if (c[2] == "red"):
                red += 1
            elif(c[2] == "magenta"):
                magenta += 1
            elif(c[2] == "blue"):
                blue += 1
            elif(c[2] == "green"):
                green += 1
            
        if red + magenta + blue == 3:
            return "m"
        elif magenta + blue + green == 3:
            return "b"
        elif blue + green + red == 3:
            return "g"
        elif green + red + magenta == 3:
            return "r"
    else:
        return 0
    

def computePosition(centroids, yaw):
    
    # distance between camera and center of mass [m]
    l = 0.32
    # calculate the reference to which the angles are computed
    reference = getReference(centroids)
    if len(centroids) < 3:
        print("Not enough beacons")
        return -1, -1,  yaw 

    # if all beacons available, choose red, magenta, blue as triad
    if len(centroids) == 4:
        c1 = centroids[1]
        c2 = centroids[0]
        c3 = centroids[3]
    else:
    # if only 3 out of 4 beacons available
        if (reference == 'm'):
            c1 = centroids[1]
            c2 = centroids[0]
            c3 = centroids[2]
        elif (reference == 'b'):
            c1 = centroids[0]
            c2 = centroids[2]
            c3 = centroids[1]
        elif (reference == 'g'):
            c1 = centroids[2]
            c2 = centroids[1]
            c3 = centroids[0]
        else:
            c1 = centroids[2]
            c2 = centroids[1]
            c3 = centroids[0]
            
    xc = 960
    yc = 540
    depAlpha = 0
    depBeta = 0
    """
    If centroids are in the same quadrant, then the angle is computed with 
    respect to the same reference and the difference in phase is 0, otherwise 
    the difference in phase is 2pi because arctan2 gives an angle between [-pi, pi]
    """
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
    if (reference == 'm' and alpha < 55*np.pi/180 and beta < 55*np.pi/180):
        """
        The robot has climbed up the platform, not reliable to reference to the 
        recycling station reference == 'm', pick up 'g' as reference
        """
        reference = "g"
        
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
    discriminant = math.pow(B,2) - 4*A*C
    x1 = (-B + math.sqrt(discriminant))/(2*A)
    y1 = N-x1*M
    x2 = (-B - math.sqrt(discriminant))/(2*A)
    y2 = N-x2*M
    if(x1>0 and x1<8 and y1>0 and y1<8):
        # translate to true arena origin  
        if (reference == 'm'):
            x = x1
            y = y1
        elif (reference == 'b'):
            x = 8 - y1
            y = x1
        elif (reference == 'g'):
            x = 8 - abs(x1)
            y = 8 - abs(y1)
        else:
            x = y1
            y = 8 - x1
        # calculate position of center of mass and absolute angle
        xCenterM = x - l*np.cos(yaw)
        yCenterM = y - l*np.sin(yaw)
        angle = getAbsoluteAngle(centroids, xCenterM, yCenterM)
        if angle != -1:
            yaw = angle
        return xCenterM, yCenterM, yaw
    elif(x2>0 and x2<8 and y2>0 and y2<8):
        # translate to true arena origin  
        if (reference == 'm'):
            x = x2
            y = y2
        elif (reference == 'b'):
            x = 8 - y2
            y = x2
        elif (reference == 'g'):
            x = 8 - abs(x2)
            y = 8 - abs(y2)
        else:
            x = y2
            y = 8 - x2
        # calculate position of center of mass and absolute angle
        xCenterM = x - l*np.cos(yaw)
        yCenterM = y - l*np.sin(yaw)
        angle = getAbsoluteAngle(centroids, xCenterM, yCenterM)
        if angle != -1:
            yaw = angle
        return xCenterM, yCenterM, yaw
    else:
        return -1, -1,  yaw 

def getAbsoluteAngle(centroids, xCenterM, yCenterM):
    """
    Calculate the absolute orientation of the robot wrt
    to the global reference frame 
    """
    if len(centroids) < 3:
        return -1
    else:
        origin = []
        opposite = []
        xc = 960
        yc = 540

        for c in centroids[::-1]:
            if(c[2] == "magenta"):
                origin = c
                break
            elif(c[2] == "green"):
                opposite = c 
                break
        if origin:
            robotAngle = np.arctan((yCenterM)/(xCenterM))
            angle = np.arctan2((origin[1]-yc),(origin[0]-xc)) - np.arctan2((1080-yc),(960-xc))

        elif opposite:
            robotAngle = np.arctan((8-yCenterM)/(8-xCenterM))
            angle = np.arctan2((opposite[1]-yc),(opposite[0]-xc)) - np.arctan2((0-yc),(960-xc)) 
            
        rel_forward_dir = angle - np.pi/4
        absolute_angle = (robotAngle + rel_forward_dir)%(2*np.pi)
        return absolute_angle

