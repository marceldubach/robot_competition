import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from picamera import PiCamera


def HSV_mask(img,hsv_min, hsv_max):
    """Find a mask using thresholding on HSV image"""
    img_blurred = cv.GaussianBlur(img, (9, 9), 0)
    img_hsv = cv.cvtColor(img_blurred, cv.COLOR_BGR2HSV)
    erode_elem = cv.getStructuringElement(cv.MORPH_RECT, (20, 20))

    mask = cv.inRange(img_hsv, hsv_min, hsv_max)
    mask = cv.erode(mask, erode_elem)
    dilate_elem = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))
    mask = cv.dilate(mask, dilate_elem)

    return mask

def find_rectangles_around_brick(brick_mask):
    """returns rectangles of pointx (x,y) where x=(x1,x2) is (vertical, horizontal)"""
    rectangles = []
    height, width = brick_mask.shape
    delta = 20  # extend the bricks by some value
    b_contours, b_hierarchy = cv.findContours(brick_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for i in range(len(b_contours)):
        if (cv.arcLength(b_contours[i],True))>800: # take only long enough contours
            cnt = b_contours[i]
            # x1: from top to bottom, x2: from left to right,w: width, h: height of rectangle
            p2,p1,w,h = cv.boundingRect(cnt)
            if (p1-delta)>0:
                x1 = p1-delta
            else:
                x1 = 0
            if (p2-delta)>0:
                x2 = p2-delta
            else:
                x2 = 0
            if (p1+h+delta<height):
                y1 = p1+h+delta
            else:
                y1 = height
            if (p2+w+delta<width):
                y2= p2+w+delta
            else:
                y2 = width
            print("[DETECTION] Found a brick at:", np.array([x1,x2,y1,y2]))
            rectangles.append(np.array([x1,x2,y1,y2]))


    return rectangles

def corner_detection(img_grey):
    corners = cv.goodFeaturesToTrack(img_grey,10,0.05,10)
    found_corner = False

    if (len(corners)!=0):
        found_corner = True

    # Add back the correct y shift with respect to original image 
    for c in corners:
        c[0][1] += 250

    return found_corner, corners

def corner_in_rectangle(rect, c):

    x1, x2, y1, y2 = rect
    if (((x1 <= c[1]) and (y1 >= c[1])) and ((x2 <= c[0]) and (y2 >= c[0]))):
        # print("Rectangle coordinates: ", rect)
        # print("Corner coordinates", c)
        return True
    else:
        return False

def corner_in_beacon(mask, c):

    #mask[y][x]: value of point with coordinates y, x in the mask
    x = int(c[0])
    y = int(c[1])
    if (mask[y][x]) != 0:
        return True
    else:
        return False

def corner_is_outlier(corners, i,min_neighbours, max_distance):
    n = 0
    isOutlier = True

    c = corners[i].flatten()
    for j in range(len(corners)):
        if (j != i):
            d = corners[j].flatten()
            # corner c must have at least i other corners in vicinity
            if (np.sqrt((c[0] - d[0]) ** 2 + (c[1] - d[1]) ** 2) < max_distance):
                n += 1
        if (n >= min_neighbours):
            isOutlier = False
    return isOutlier

def add_corners(img, cornerList):
    if (len(cornerList)>0):
        xmin = np.min([x[0] for x in cornerList])
        xmax = np.max([x[0] for x in cornerList])
        ymin = np.min([y[1] for y in cornerList])
        ymax = np.max([y[1] for y in cornerList])

        x_center = int((xmin+xmax)/2)
        y_center = int((ymin+ymax)/2)

        cv.circle(img, (x_center, y_center), 20, 0, -1)

        for c in cornerList:
            x, y = c # extract components
            cv.circle(img, (x, y), 5, 0, -1)
        return True, np.array([x_center, y_center])  #img
    else:
        return False, np.array([-1,-1])  #img

def detect_bottle(queue, e_bottle):
    #t_start_det = time.time()
    camera = PiCamera()
    camera.rotation = 180
    camera.resolution = (1280,720)
    #img = np.empty((720,1280,3))
    camera.capture('frontal_img.jpg')
    img = cv.imread('frontal_img.jpg')

    hsv_min = np.array([0, 26, 116])
    hsv_max = np.array([178, 54, 166])
    brick_mask = HSV_mask(img, hsv_min, hsv_max)

    rectangles = find_rectangles_around_brick(brick_mask)
    img_out = img.copy()
    for rect in rectangles:
        #print(rect)
        x1, x2, y1, y2 = rect
        cv.rectangle(img_out, (x2, x1), (y2, y1), (255, 0, 0), 3)

    # Compute a mask on the beacon to exclude outliers given by extreme color gradient 
    beacon_hsv_min = np.array([0, 0, 200])
    beacon_hsv_max = np.array([180, 255, 255])
    beacon_mask = HSV_mask(img, beacon_hsv_min, beacon_hsv_max)

    # The region of interest excludes image upper part
    ROI = img[250:, 0:]
    img_gray = cv.cvtColor(ROI, cv.COLOR_BGR2GRAY)
    foundCorner, corners = corner_detection(img_gray)

    # Try to exclude outliers
    cornerList = []
    for i in range(len(corners)):
        c = corners[i].flatten()

        isBrick = False
        isBeacon = False
        for rect in rectangles:
            if corner_in_rectangle(rect, c):
                isBrick = True  # corner lies inside a rectangle describing a brick
                break

        if corner_in_beacon(beacon_mask, c):
            isBeacon = True

        if not isBrick and not isBeacon:
            min_neighbours = 6  # minimum amount of neighbours
            max_distance = 100  # distance in pixels within the corner must have its neighbours
            isOutlier = corner_is_outlier(corners, i, min_neighbours, max_distance)

        if not isBrick and not isBeacon and not isOutlier:
            # print("Append corner at  (x1, x2): (" , c[0], ", ", c[1],")")
            cornerList.append(c)

    # TODO add the case where there are more than 1 bottle in the image!

    # Plot a bounding box around the bottle
    has_bottle, center = add_corners(img_out, cornerList)

    queue.put(center)
    if(has_bottle):
        print("[DETECTION] Found a bottle at position ", center)
    else:
        print("[DETECTION] Found no bottle ...")
    print("center bottle:", center)
    e_bottle.set()
    #print("e_bottle:", e_bottle.is_set())
    return queue
