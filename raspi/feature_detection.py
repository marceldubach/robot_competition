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

if do_all:
    img_name_list = os.listdir('images/noIR/')
else:
    img_name_list = ['1TileFromCoke.jpg']

for img_name in img_name_list:
    img = cv.imread('images/noIR/'+img_name)
    if img is None:
        print("Error opening image ", img_name_list)
        continue

# TODO continut with tutorial: https://docs.opencv.org/3.4/d2/d2c/tutorial_sobel_derivatives.html
    img_blurred = cv.GaussianBlur(img, (9,9), 0)
    gray_blurred = cv.cvtColor(img_blurred, cv.COLOR_BGR2GRAY)

    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    print(img_gray.shape)

    brick_grayscale = img_gray.copy()
    ddepth = cv.CV_16S
    scale = 1
    ksize= 5
    delta = 0

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
    plt.title("Gradient")
    plt.imshow(grad)
    plt.show()




    # find the contours of a brick
    contours, hierachy = cv.findContours(grad_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    cont = []
    min_length = 10
    for i in range(len(contours)):
        if (cv.arcLength(contours[i],False)>50):
            cont.append(contours[i])
            # print("Length of contour:", cv.arcLength(contours[i],False))

    cv.drawContours(brick_grayscale, cont, -1, (0, 255, 0), 3)
    #
    # plt.figure()
    # plt.title("Contour Mask")
    # plt.imshow(grad_thresh)
    # plt.show()

    plt.figure()
    plt.title("Contour detection")
    plt.imshow(brick_grayscale)
    plt.show()


    # # Try to find values for the floor region
    # blue_min = np.min(img[150:220,:100,0])
    # green_min = np.min(img[150:220,:100, 1])
    # red_min = np.min(img[150:220,:100, 2])
    # blue_max = np.max(img[150:220,:100,0])
    # green_max = np.max(img[150:220,:100, 1])
    # red_max = np.max(img[150:220,:100, 2])
    #
    # # Brick detection:
    # print("Brick colors")
    # print("min BGR: ", [blue_min, green_min, red_min])
    # print("max BGR: ", [blue_max, green_max, red_max])
    #
    # brick_mask_b = np.logical_and(img[:, :, 0] > 110, img[:, :, 0] < 165)
    # brick_mask_g = np.logical_and(img[:, :, 0] > 90, img[:, :, 0] < 140)
    # brick_mask_r = np.logical_and(img[:, :, 0] > 100, img[:, :, 0] < 160)
    # brick_mask = np.logical_and(np.logical_and(brick_mask_b, brick_mask_g), brick_mask_r)
    #
    # plt.figure()
    # plt.title("Brick Mask")
    # plt.imshow(brick_mask)
    # plt.show()


    foundCorner, corners = corner_detection(img_gray)

    # Try to exclude outliers
    for i in range(len(corners)):
        isOutlier = True
        for i in

    # Plot a bounding box around the bottle
    plt.figure()
    if foundCorner:
        xmin = np.min(corners[:,:,0])
        xmax = np.max(corners[:,:,0])
        ymin = np.min(corners[:,:,1])
        ymax = np.max(corners[:,:,1])

        x_center = int((xmin+xmax)/2)
        y_center = int((ymin+ymax)/2)

        delta = 30
        for i in corners:
            x, y = i.ravel()
            cv.circle(img_gray, (x, y), 10, 255, -1)

        # img_gray = cv.rectangle(img_gray, (xmin-delta, ymin-delta), (xmax+delta, ymax+delta), 255, 3)
        plt.title(img_name+ ': Bottle found at [x,y]=['+str(x_center)+','+str(y_center)+']')
    else:
        plt.title(img_name+ ': No Bottle Found')

    plt.imshow(img_gray)
    plt.show()

# HARRIS CORNER DETECTION
# img_grey = np.float32(img_grey)
# dest = cv.cornerHarris(img_grey,3,5,0.07)


