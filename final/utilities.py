import numpy as np
from numpy.linalg import norm
import cv2 as cv
from picamera import PiCamera
from scipy.interpolate import interp1d
import time


def HSV_mask(img, hsv_min, hsv_max):
    """
    Find a mask using thresholding on HSV image
    """
    img_blurred = cv.GaussianBlur(img, (9, 9), 0)
    img_hsv = cv.cvtColor(img_blurred, cv.COLOR_BGR2HSV)
    erode_elem = cv.getStructuringElement(cv.MORPH_RECT, (20, 20))

    mask = cv.inRange(img_hsv, hsv_min, hsv_max)
    mask = cv.erode(mask, erode_elem)
    dilate_elem = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))
    mask = cv.dilate(mask, dilate_elem)
    return mask

def corner_detection(img_grey):
    """
    This function implements the openCV goodFeaturesToTrack() function to detect the
    most relevant corners in the ROI of the image, in particular it is asked to return 
    the 10 most important corners with each one separated by at least 10 pixels.
    """
    corners = cv.goodFeaturesToTrack(img_grey,10,0.05,10)
    found_corner = False

    if (len(corners)!=0):
        found_corner = True

    # Add back the correct y shift with respect to original image 
    for c in corners:
        c[0][1] += 200
      
    return found_corner, corners

def corner_in_beacon(mask, c):
    """
    Check if corner happens to be inside the beaacon mask
    """
    x = int(c[0])
    y = int(c[1])
    #mask[y][x]: value of point with coordinates y, x in the mask
    if (mask[y][x]) != 0:
        return True
    else:
        return False

def corner_is_outlier(corners, i, min_neighbours, max_distance):
    """
    Count how many corners lie in the circle of radius max_distance 
    around the given corner and asses if outlier or not
    """
    n = 0
    isOutlier = True
    c = corners[i].flatten()
    for j in range(len(corners)):
        if (j != i):
            d = corners[j].flatten()
            # corner c must have at least min_neighbours other corners in vicinity
            if (np.sqrt((c[0] - d[0]) ** 2 + (c[1] - d[1]) ** 2) < max_distance):
                n += 1
        if (n >= min_neighbours):
            isOutlier = False
   
    return isOutlier

def add_corners(img, cornerList):
    """
    Draw found corners on the image and return bottle centroid 
    """
    if (len(cornerList)>0):
        xmin = np.min([x[0] for x in cornerList])
        xmax = np.max([x[0] for x in cornerList])
        ymin = np.min([y[1] for y in cornerList])
        ymax = np.max([y[1] for y in cornerList])

        x_center = int((xmin+xmax)/2)
        y_center = int((ymin+ymax)/2)

        cv.circle(img, (x_center, y_center), 20, [0,0,255], -1)

        for c in cornerList:
            x, y = c 
            cv.circle(img, (x, y), 5, 0, -1)
        return True, np.array([x_center, y_center])
    else:
        return False, np.array([-1,-1])

def detect_bottle(queue, e_bottle):
    """
    Function called in the python process p_bottle(), it defines the interpolation functions
    to return distance and orientation of the bottle with respect to the robot, initializes 
    the Picamera and takes the picture, applies a mask to detect the presence of a beacon in
    the image, defines the region of interest ROI and detects corners. 
    Finally, it evaluates only the relevant corners to compute the bottle centroid in the 
    image and hence the bottle distance and orientation thanks to the interpolation functions.
    This information is passed to the main program through the queue.
    """
    y_img = np.array([448, 407, 385, 375, 300, 282, 258, 243, 232, 230])
    dist = np.array([30, 40, 50, 54, 75, 80.9, 100, 104, 126, 130]) 
    f_dist = interp1d(y_img, dist)
    x_img = np.array([1180, 1050, 842, 695, 500, 347, 200])
    theta = np.array([-26,-22, -10, 0, 10, 22, 26]) 
    f_theta = interp1d(x_img, theta)

    y_min = np.min(y_img)
    y_max = np.max(y_img)
    x_min = np.min(x_img)
    x_max = np.max(x_img)

    camera = PiCamera()
    camera.rotation = 180
    camera.resolution = (1280,720)
    
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
        isBeacon = False
    
        if corner_in_beacon(beacon_mask, c):
            isBeacon = True

        if not isBeacon:
            min_neighbours = 6  
            max_distance = 100  
            isOutlier = corner_is_outlier(corners, i, min_neighbours, max_distance)

        if not isBeacon and not isOutlier:
            cornerList.append(c)

    has_bottle, center = add_corners(img_out, cornerList)
    x = center[0]
    y = center[1]
    if(has_bottle):
        if (x < x_max) and (x > x_min) and (y < y_max) and (y > y_min):
            distance = f_dist(center[1])/100
            angle = f_theta(center[0])/180*np.pi
            bottle_pos = np.array([distance, angle])
            queue.put(bottle_pos)
            print("[DETECTION] Found a bottle at position ", center, "(pixels), bottle position(arena) :", bottle_pos)
        else:
            bottle_pos = np.array([-1, -1])
            queue.put(bottle_pos)
    else:
        queue.put(center)
        print("[DETECTION] Found no bottle ...")
    
    camera.close()
    e_bottle.set()
    return queue

def ultrasound_dist_to_rel_pos(dist, i):
    """ 
    Calculate the relative distance from a detection of the ultrasonic sensor 
    w.r.t the robot frame
    """
    offset = np.array([[-0.24, 0.15],[0.05, 0.15], [0.07, 0.09], [0.07, 0], [0.07, -0.09], [0.05, -0.15], [-0.24, -0.15]])
    angles = np.array([np.pi/2, np.pi/4, 0,0,0, -np.pi/4, -np.pi/2])
    rel_dist = offset[i] + dist*np.array([np.cos(angles[i]), np.sin(angles[i])])
    return rel_dist

def get_close_obstacles(local_obstacles, waypoint, radius):
    """
    Store in a list the obstacles closer than a given radius 
    """
    close_obstacles = []
    for wp in waypoint:
        for cl_obst in close_obstacles:
            if (norm(cl_obst-waypoint)<radius):
                close_obstacles.append(cl_obst)
    return close_obstacles

def waypoint_is_valid(waypoint):
    """
    Check if the computed waypoint lays in a feasible region (i.e not outside arena 
    nor to close to boundaries)
    """
    is_valid = False
    if (waypoint[0]>0.5) and (waypoint[0]<7.5):
        if (waypoint[1]>0.5) and (waypoint[1]<4.5):
            is_valid = True
    return is_valid

def is_obstacle(waypoint, obst_list):
    """
    Check that the bottle waypoint is not on an obstacle 
    """
    radius = 0.3 
    is_obstacle = False
    for o in obst_list:
        if (norm(o-waypoint)<radius):
            is_obstacle = True
    return is_obstacle

def get_time(time_start):
    return time.time() - time_start
