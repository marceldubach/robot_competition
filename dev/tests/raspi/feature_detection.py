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



do_all = True

plotBrickMask = True

# load images
if do_all:
    img_name_list = os.listdir('images/noIR/')
else:
    img_name_list = ['1TileFromBrick.jpg']

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

    # brick_grayscale = img_gray.copy()
    ddepth = cv.CV_16S
    # scale = 1
    # ksize= 5
    # delta = 0

    # Scharr filter works better than sobel?
    grad_x = cv.Scharr(gray_blurred, ddepth, 1, 0)
    grad_y = cv.Scharr(gray_blurred, ddepth, 0,1)

    # grad_x = cv.Sobel(gray_blurred, ddepth, 1, 0, ksize, scale, delta, cv.BORDER_DEFAULT)
    # grad_y = cv.Sobel(gray_blurred, ddepth, 0, 1, ksize, scale, delta, cv.BORDER_DEFAULT)
    abs_grad_x = cv.convertScaleAbs(grad_x)
    abs_grad_y = cv.convertScaleAbs(grad_y)

    grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
    ret, grad_thresh = cv.threshold(grad,100,255,0)

    # grad[grad_thresh] = 255
    # grad[np.invert(grad_thresh)] = 0
    # # grad[!grad_thresh] = 0
    # grad = cv.GaussianBlur(grad, (11,11), 0)
    # ret, grad_thresh = cv.threshold(grad, 90, 255, 0)

    plt.figure()
    plt.title("Edge detection")
    plt.imshow(grad_thresh)
    plt.show()

    # find the contours of a brick
    contours, hierachy = cv.findContours(grad_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    cont = []
    min_length = 50 # remove lines that are too short
    for i in range(len(contours)):
        if (cv.arcLength(contours[i],False)>min_length):
            eps = 0.01*cv.arcLength(contours[i],False)
            approx_contour = cv.approxPolyDP(contours[i], eps, False)
            cont.append(approx_contour)
            #cont.append(contours[i])
            #print("Length of contour:", cv.arcLength(contours[i],False))

    # plot the contours on a copy of the blurred graysacle image
    brick_grayscale = gray_blurred.copy()

    cv.drawContours(brick_grayscale, cont, -1, (0, 255, 0), 3)


#    plt.figure()
#    plt.title("Contour detection")
#    plt.imshow(brick_grayscale)
#    plt.show()


    # # not sure the following is a good idea...
    # # Try to find values for the floor region
    # blue_min = np.min(img[300:400,600:800,0])
    # green_min = np.min(img[300:400,600:800,1])
    # red_min = np.min(img[300:400,600:800,2])
    # blue_max = np.max(img[300:400,600:800,0])
    # green_max = np.max(img[300:400,600:800,1])
    # red_max = np.max(img[300:400,600:800,2])
    #
    # # # Brick detection:
    # print("Brick colors")
    # print("min BGR: ", [blue_min, green_min, red_min])
    # print("max BGR: ", [blue_max, green_max, red_max])

    # TODO do this in HSV!
    brick_mask_b = np.logical_and(img_blurred[:, :, 0] > 70, img_blurred[:, :, 0] < 165)
    brick_mask_g = np.logical_and(img_blurred[:, :, 0] > 70, img_blurred[:, :, 0] < 160)
    brick_mask_r = np.logical_and(img_blurred[:, :, 0] > 100, img_blurred[:, :, 0] < 180)
    brick_mask   = np.logical_and(np.logical_and(brick_mask_b, brick_mask_g), brick_mask_r)

    img_mask = img_gray.copy();
    img_mask[:,:] = 0
    img_mask[brick_mask] = 255;

    element = cv.getStructuringElement(cv.MORPH_RECT,(20,20))
    img_mask = cv.erode(img_mask, element)
    img_mask = cv.dilate(img_mask, element, iterations=2)
    #mask = np.logical_and(brick_mask, grad_thresh)
    if plotBrickMask:
        plt.figure()
        plt.title("Img Mask")
        plt.imshow(img_mask)
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
            #if (img_mask[int(c[1]),int(c[0])] != 255):
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






