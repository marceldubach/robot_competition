import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

# load image as BGR image
img = cv.imread('images/foo.jpg')

# access an item using
img.item(10,10,2)

# img.size returns the size of the image (nb pixels)
# img is an array of uint8 objects

# dim1: vertical (from top to bottom)
# dim2: horizontal (from left to right)
img = img[100:,:-160]


img_HSV = cv.cvtColor(img,cv.COLOR_BGR2HSV)

#mask = cv.inRange()
#img[:,:,2] = img[:,:,2]
edges = cv.Canny(img,80,150)

plt.figure()
plt.hist(img[0].ravel(),256,[0,256])
plt.show()

plt.figure()
plt.hist(img[1].ravel(),256,[0,256],'g')
plt.show()

plt.figure()
plt.hist(img[1].ravel(),256,[0,256],'r')
plt.show()
# split is numerically intense!
#b,g,r = cv.split(img)

plt.figure()
plt.imshow(edges)
plt.show()

print("helloworld!")