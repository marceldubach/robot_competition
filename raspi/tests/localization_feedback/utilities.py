import numpy as np
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

def corner_detection(img_grey):
    corners = cv.goodFeaturesToTrack(img_grey,10,0.05,10)
    found_corner = False

    if (len(corners)!=0):
        found_corner = True

    # Add back the correct y shift with respect to original image 
    for c in corners:
        c[0][1] += 200
      
    return found_corner, corners

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
    #print("neighbours",n)
   
    return isOutlier

def add_corners(img, cornerList):
    if (len(cornerList)>0):
        xmin = np.min([x[0] for x in cornerList])
        xmax = np.max([x[0] for x in cornerList])
        ymin = np.min([y[1] for y in cornerList])
        ymax = np.max([y[1] for y in cornerList])

        x_center = int((xmin+xmax)/2)
        y_center = int((ymin+ymax)/2)

        cv.circle(img, (x_center, y_center), 20, [0,0,255], -1)

        for c in cornerList:
            x, y = c # extract components
            cv.circle(img, (x, y), 5, 0, -1)
        return True, np.array([x_center, y_center])
    else:
        return False, np.array([-1,-1])

def bottle_ref(position, Zi, r2):
    
    x_b = position[0]
    y_b = position[1] 
    r1 = Zi.dot([x_b, y_b, 1.0]) 
    if 640 > x_b:
        sign = 1
    else:
        sign = -1
    cos_angle = r1.dot(r2) / (np.linalg.norm(r1) * np.linalg.norm(r2))
    angle_radians = sign*np.arccos(cos_angle) 
    distance = 1.5 - ((y_b)-200)/200
    return distance, angle_radians

def detect_bottle(queue, e_bottle, Zi, r2):
    #t_start_det = time.time()

    camera = PiCamera()
    camera.rotation = 180
    camera.resolution = (1280,720)
    
    #img = np.empty((720,1280,3))
    camera.capture('frontal_img.jpg')
    img = cv.imread('frontal_img.jpg')
    
    img_out = img.copy()
    
    # Compute a mask on the beacon to exclude outliers given by extreme color gradient 
    beacon_hsv_min = np.array([0, 0, 200])
    beacon_hsv_max = np.array([180, 255, 255])
    beacon_mask = HSV_mask(img, beacon_hsv_min, beacon_hsv_max)

    # The region of interest excludes image upper part
    ROI = img[200:, 0:]
    img_gray = cv.cvtColor(ROI, cv.COLOR_BGR2GRAY)
    foundCorner, corners = corner_detection(img_gray)

    # Try to exclude outliers
    cornerList = []
    for i in range(len(corners)):
        c = corners[i].flatten()

        isBrick = False
        isBeacon = False
    
        if corner_in_beacon(beacon_mask, c):
            isBeacon = True

        if not isBrick and not isBeacon:
            min_neighbours = 6  # minimum amount of neighbours
            max_distance = 100  # distance in pixels within the corner must have its neighbours
            isOutlier = corner_is_outlier(corners, i, min_neighbours, max_distance)

        if not isBrick and not isBeacon and not isOutlier:
            # print("Append corner at  (x1, x2): (" , c[0], ", ", c[1],")")
            cornerList.append(c)


    # Plot a bounding box around the bottle
    has_bottle, center = add_corners(img_out, cornerList)

    if(has_bottle):
        distance, angle = bottle_ref(center, Zi, r2)
        bottle_pos = np.array([distance, angle])
        queue.put(bottle_pos)
        print("[DETECTION] Found a bottle at position ", center, "(pixels), bottle position(arena) :", bottle_pos)
    else:
        queue.put(center)
        print("[DETECTION] Found no bottle ...")
    #print("center bottle:", center)
    camera.close()
    e_bottle.set()
    #print("e_bottle:", e_bottle.is_set())
    return queue

