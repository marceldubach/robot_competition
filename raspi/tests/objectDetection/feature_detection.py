import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import os

from utilities import *

#  TODO:  - detect bricks
#         - remove the corners detected in brick region

# Shi-Tomasi Corner detector




do_all = True

plotBrickMask = True

# load images
if do_all:
    img_name_list = os.listdir('images/noIR/')
else:
    img_name_list = ['2TileFromBrick.jpg']

for img_name in img_name_list:
    img = cv.imread('images/noIR/'+img_name)
    if img is None:
        print("Error opening image ", img_name_list)
        continue

    # TODO continut with tutorial: https://docs.opencv.org/3.4/d2/d2c/tutorial_sobel_derivatives.html

    # find threshold values of brick in HSV image
    # x1_min = 200
    # x1_max = 300
    # x2_min = 500
    # x2_max = 700
    # img_blurred = cv.GaussianBlur(img, (9, 9), 0)
    # img_hsv = cv.cvtColor(img_blurred, cv.COLOR_BGR2HSV)
    #
    # h_min = np.min(img_hsv[x1_min:x1_max,x2_min:x2_max,0])
    # s_min = np.min(img_hsv[x1_min:x1_max,x2_min:x2_max,1])
    # v_min = np.min(img_hsv[x1_min:x1_max,x2_min:x2_max,2])
    # h_max = np.max(img_hsv[x1_min:x1_max,x2_min:x2_max,0])
    # s_max = np.max(img_hsv[x1_min:x1_max,x2_min:x2_max,1])
    # v_max = np.max(img_hsv[x1_min:x1_max,x2_min:x2_max,2])
    # # # # Brick detection:
    # print("Brick colors")
    # print("min HSV: ", [h_min, s_min, v_min])
    # print("max HSV: ", [h_max, s_max, v_max])

    hsv_min = np.array([0, 26, 116])
    hsv_max = np.array([178, 54, 166])
    brick_mask = HSV_brick_mask(img, hsv_min, hsv_max)

    if plotBrickMask:
        plt.figure()
        plt.title("Brick mask")
        plt.imshow(brick_mask)
        plt.show()

    rectangles = find_rectangles_around_brick(brick_mask)

    img_out = img.copy()
    for rect in rectangles:
        print(rect)
        x1,x2,y1,y2 = rect
        cv.rectangle(img_out, (x2,x1),(y2,y1), (255,0,0), 3)

    # plt.figure()
    # plt.imshow(img_out)
    # plt.show()


    # PART 2: Bottle detection
    # remove all corners laying on a brick area
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    foundCorner, corners = corner_detection(img_gray)

    # Try to exclude outliers
    cornerList = []
    for i in range(len(corners)):
        c = corners[i].flatten()

        isBrick = False
        for rect in rectangles:
            if corner_in_rectangle(rect, c):
                isBrick = True # corner lies inside a rectangle describing a brick
                break

        if not isBrick:
            min_neighbours = 5 # minimum amount of neighbours
            max_distance = 200 # distance in pixels within the corner must have its neighbours
            isOutlier = corner_is_outlier(corners, i, min_neighbours, max_distance)
            if isOutlier:
                break

        if not isBrick and not isOutlier:
            #print("Append corner at  (x1, x2): (" , c[0], ", ", c[1],")")
            cornerList.append(c)

    # TODO add the case where there are more than 1 bottle in the image!

    # Plot a bounding box around the bottle
    has_bottle, center, img_out = add_corners(img_out, cornerList)
    img_out = cv.cvtColor(img_out,cv.COLOR_BGR2RGB)
    if has_bottle:
        plt.figure()
        plt.title("Bottle found")
        plt.imshow(img_out)
        plt.show()
    else:
        plt.figure()
        plt.title("No bottle found")
        plt.imshow(img_out)
        plt.show()







