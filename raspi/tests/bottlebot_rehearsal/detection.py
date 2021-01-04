import os
import argparse
import cv2 as cv
import time
from picamera import PiCamera

from utilities import *



parser = argparse.ArgumentParser()

t_start = time.time()
parser.add_argument("-i", "--image", help="enter the name of the .jpg file for detection")
parser.add_argument("-n", "--name", help="enter aname of image")
args = parser.parse_args()

def detect_bottle(img=None):
    t_start_det = time.time()
    if img is None:
        camera = PiCamera()
        camera.rotation = 180
        camera.resolution = (1280,720)
        img = np.empty((720,1280,3))
        camera.capture(img, 'rgb')

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
    has_bottle, center, img_out = add_corners(img_out, cornerList)

    if (has_bottle):
        print("[DETECTION] Found a bottle at position ", center)
    else:
        print("[DETECTION] Found no bottle ...")

    t_stop_det = time.time()
    print("[DETECTION] detection time: ", t_stop_det - t_start_det, "seconds")
    return has_bottle, center, img_out

if __name__ == '__main__':
    img_name = "image1.jpg"
    if (parser.image is not None):
        print(args.image)
        img = cv.imread("images/noIR/"+args.image)
    else:
        if (parser.name is not None):
            img_name = parser.name
        cam = PiCamera()
        cam.capture("images/noIR/"+img_name)
    if img is None:
        print("Error opening image ", "images/noIR/"+args.image)
        pass
    else:

        print("Saving detection result to file ", "images/noIR/results")

        has_bottle, center, img_out = detect_bottle(img)

        img_out = cv.cvtColor(img_out, cv.COLOR_BGR2RGB)

        plt.figure()
        if has_bottle:
            plt.title("Found bottle")
        else:
            plt.title("No bottle found")
        plt.imshow(img_out)

        if not os.path.exists('images'):
            os.mkdirs('images')
        if not os.path.exists('images/noIR'):
            os.makedirs('images/noIR')
        if not os.path.exists('images/noIR/results'):
            os.makedirs('images/noIR/results')

        plt.imsave("images/noIR/results/result_"+img_name, img_out)







