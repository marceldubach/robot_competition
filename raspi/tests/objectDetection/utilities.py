import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv


def HSV_brick_mask(img,hsv_min, hsv_max):
    """Find a mask for bricks using thresholding on HSV image"""
    img_blurred = cv.GaussianBlur(img, (9, 9), 0)
    img_hsv = cv.cvtColor(img_blurred, cv.COLOR_BGR2HSV)
    erode_elem_brick = cv.getStructuringElement(cv.MORPH_RECT, (20, 20))

    brick_mask = cv.inRange(img_hsv, hsv_min, hsv_max)
    brick_mask = cv.erode(brick_mask, erode_elem_brick)
    dilate_elem_brick = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))
    brick_mask = cv.dilate(brick_mask, dilate_elem_brick)

    return brick_mask

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
            print("Found a brick at:", np.array([x1,x2,y1,y2]))
            rectangles.append(np.array([x1,x2,y1,y2]))


    return rectangles

def corner_in_rectangle(rect, c):

    x1, x2, y1, y2 = rect
    if (((x1 <= c[1]) and (y1 >= c[1])) and ((x2 <= c[0]) and (y2 >= c[0]))):
        # print("Rectangle coordinates: ", rect)
        # print("Corner coordinates", c)
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
            # corner c must have at least 3 other corners in vicinity
            if (np.sqrt((c[0] - d[0]) ** 2 + (c[1] - d[1]) ** 2) < max_distance):
                n += 1
        if (n >= min_neighbours):
            isOutlier = False
    return isOutlier

def plot_corners(img, cornerList):

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
        plt.figure()
        plt.imshow(img)
        plt.title("Bottle found at [x,y]=["+str(x_center)+","+str(y_center)+"]")
        plt.show()
    else:
        plt.figure()
        plt.imshow(img)
        plt.title(": No bottle found")
        plt.show()