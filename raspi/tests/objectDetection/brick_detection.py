import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import imutils

img = cv.imread('images/noIR/brickHL2.jpg')
if img is None:
    print("Error loading image")

img = img[100:,:,:]
img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

resized = imutils.resize(img, width=300)
ratio = img.shape[0] / float(resized.shape[0])

blurred = cv.GaussianBlur(img, (9, 9), 0)
thresh = cv.threshold(blurred, 60, 255, cv.THRESH_BINARY)[1]

img = cv.equalizeHist(img)
img_out = cv.cvtColor(img,cv.COLOR_BGR2RGB)
plt.figure()
plt.imshow(img_out)
plt.show()