import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import os


#  TODO:  - detect bricks
#         - remove the corners detected in brick region

# Shi-Tomasi Corner detector
def corner_detection(img_grey):
    corners = cv.goodFeaturesToTrack(img_grey,20,0.01,10)
    found_corner = False

    if (len(corners)!=0):
        found_corner = True

    return found_corner, corners



do_all = False

plotBrickMask = True

# load images
if do_all:
    img_name_list = os.listdir('images/noIR/')
else:
    img_name_list = ['1TileFromBottle.jpg']

for img_name in img_name_list:
    img = cv.imread('images/noIR/'+img_name)
    if img is None:
        print("Error opening image ", img_name_list)
        continue

    # TODO continut with tutorial: https://docs.opencv.org/3.4/d2/d2c/tutorial_sobel_derivatives.html

    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    print("Image size is: ", img_gray.shape)


    # PART 1: BRICK DETECTION

    # apply a Gaussian blur on the colored image, then convert it to GrayScale
    img_blurred = cv.GaussianBlur(img, (9,9), 0)
    gray_blurred = cv.cvtColor(img_blurred, cv.COLOR_BGR2GRAY)

    # EDGE DETECTION (MAYBE OMIT THIS)
    ddepth = cv.CV_16S
    # Scharr filter works better than sobel
    grad_x = cv.Scharr(gray_blurred, ddepth, 1, 0)
    grad_y = cv.Scharr(gray_blurred, ddepth, 0,1)

    abs_grad_x = cv.convertScaleAbs(grad_x)
    abs_grad_y = cv.convertScaleAbs(grad_y)

    grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
    ret, grad_thresh = cv.threshold(grad,100,255,0)
    element = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    grad_thresh = cv.erode(grad_thresh, element)
    plt.figure()
    plt.title("Edge detection")
    plt.imshow(grad_thresh)
    plt.show()

    # find the contours of a brick
    #contours, hierachy = cv.findContours(grad_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    """
        cont = []
        min_length = 50 # remove lines that are too short
        for i in range(len(contours)):
            if (cv.arcLength(contours[i],False)>min_length):
                eps = 0.01*cv.arcLength(contours[i],False)
                approx_contour = cv.approxPolyDP(contours[i], eps, False)
                cont.append(approx_contour)
                #cont.append(contours[i])
                #print("Length of contour:", cv.arcLength(contours[i],False))
    """
    # plot the contours on a copy of the blurred graysacle image
    # brick_grayscale = gray_blurred.copy()

    # cv.drawContours(brick_grayscale, cont, -1, (0, 255, 0), 3)




    # find threshold values of brick in HSV image
    # x1_min = 0
    # x1_max = 200
    # x2_min = 450
    # x2_max = 800
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

    img_hsv = cv.cvtColor(img_blurred, cv.COLOR_BGR2HSV)
    erode_elem_brick = cv.getStructuringElement(cv.MORPH_RECT, (20,20))

    hsv_min = np.array([0, 33, 116])
    hsv_max = np.array([179, 54, 166])
    brick_mask = cv.inRange(img_hsv, hsv_min, hsv_max)
    brick_mask = cv.erode(brick_mask, erode_elem_brick)
    dilate_elem_brick  = cv.getStructuringElement(cv.MORPH_RECT, (10,10))
    brick_mask = cv.dilate(brick_mask, dilate_elem_brick)

    if plotBrickMask:
        plt.figure()
        plt.title("Brick mask")
        plt.imshow(brick_mask)
        plt.show()

    rectangles = []
    b_contours, b_hierarchy = cv.findContours(brick_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for i in range(len(b_contours)):
        if (cv.arcLength(b_contours[i],True))>900:
            cnt = b_contours[i]
            #cv.drawContours(img, [cnt],0, (0,255,0),3)
            # x1: from top to bottom
            # x2: from left to right
            # w: width (horizontal), h: height
            x2,x1,w,h = cv.boundingRect(cnt)
            print("Found a brick at:", [x2,x1,w,h])
            rectangles.append(np.array([x1,x2,x1+h,x2+w]))
            #cv.drawContours(img, cnt, 0,(0,255,0), 3)
            cv.rectangle(img, (x2,x1),(x2+w,x1+h), (255,0,0), 3)
            #print(cv.arcLength(b_contours[i],True))

    for rect in rectangles:
        x1,x2,y1,y2 = rect
        print("rectangle stored at (",x1,",",x2,") ",y1,",",y2,")")

    plt.figure()
    plt.imshow(img)
    plt.show()


    # PART 2: Bottle detection
    # TODO remove all corners that lie inside the brick!

    foundCorner, corners = corner_detection(img_gray)

    # Try to exclude outliers
    cornerList = []
    for i in range(len(corners)):
        isOutlier = True
        n = 0  # number of neighbours
        c = corners[i].flatten()
        for j  in range(len(corners)):
            if (i!=j):
                d = corners[j].flatten()

                # corner c must have at least 3 other corners in vicinity
                if (np.sqrt((c[0]-d[0])**2 + (c[1]-d[1])**2)<100):
                    n += 1
            if (n>=3):
                isOutlier = False
                break

        if (not isOutlier):

            isBrick = False
            for rect in rectangles:
                x1,x2,y1,y2 = rect
                #print("rectangle stored at (",x1,",",x2,") ",y1,",",y2,")")
                if ((x1<=c[0]) and (y1>=c[0])) and ((x2<=c[1]) and (x2>=c[1])):
                    isBrick = True

            #if (img_mask[int(c[1]),int(c[0])] != 255):
            if not isBrick:
                #print("Append corner at  (x1, x2): (" , c[0], ", ", c[1],")")
                cornerList.append(c)


    # Plot a bounding box around the bottle
    plt.figure()
    if (len(cornerList)>0):
        xmin = np.min([x[0] for x in cornerList])
        xmax = np.max([x[0] for x in cornerList])
        ymin = np.min([y[1] for y in cornerList])
        ymax = np.max([y[1] for y in cornerList])

        x_center = int((xmin+xmax)/2)
        y_center = int((ymin+ymax)/2)

        delta = 30
        for c in cornerList:
            x, y = c # extract components
            cv.circle(img_gray, (x, y), 5, 255, -1)

        upper_left = (xmin-delta, ymin-delta)
        lower_right = (xmax+delta, ymax+delta)

        cv.circle(img_gray, (x_center, y_center), 20, 255, -1)
        # rectangle doesnt work for some stupid reason
        # img_gray = cv.rectangle(img_gray, upper_left, lower_right, 255)

        plt.title(img_name+ ': Bottle found at [x,y]=['+str(x_center)+','+str(y_center)+']')
    else:
        plt.title(img_name+ ': No Bottle Found')

    plt.imshow(img_gray)
    plt.show()






