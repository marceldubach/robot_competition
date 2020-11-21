import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

# load image as BGR image
img = cv.imread('images/image4.jpeg')
#img = img[200:800,500:1500]
img_grey = cv.cvtColor(img,cv.COLOR_BGR2GRAY)

# print("Wall colour min")
# print(img_grey[0:200,0:200].min())
# print("Wall colour max")
# print(img_grey[0:200,0:200].max())
#
#
# print("Carpet colour")
# print(img_grey[400:550,0:400].max())
#
# print("Carpet colour min")
# print(img_grey[400:550,0:400].min())


thresh_img = img_grey
# add a gaussian blur
blurred = cv.GaussianBlur(thresh_img, (5,5), cv.BORDER_DEFAULT)

print("Wall colour min")
print(blurred[0:200,0:200].min())
print("Wall colour max")
print(blurred[0:200,0:200].max())


print("Carpet colour max")
print(blurred[400:550,0:400].max())

print("Carpet colour min")
print(blurred[400:550,0:400].min())


idx_array = (blurred>100)
idx_array2 = (blurred<180)

idx = np.logical_and(idx_array,idx_array2)
print(idx.shape)
thresh_img[idx] = 0


# remove wall part
# idx_wall = (blurred>210)
# idx_wall2 = (blurred<220)
# idx_w = np.logical_and(idx_wall,idx_wall2)

# thresh_img[idx_w] = 0

plt.figure()
plt.imshow(thresh_img)
plt.show()

# plt.figure()
# plt.imshow(img_grey)
# plt.show()

# HARRIS CORNER DETECTION
img_grey = np.float32(img_grey)

# arguments: (image, blockSize, kSize,freeParameter,borderType)
# blockSize: neighbourhood size
# ksize: aperture parameter for the Sobel() filter
# freeParameter: ?
# border type: pixel extrapolation method.
dest = cv.cornerHarris(img_grey,3,5,0.07)

# do thresholding
# plt.figure()
# plt.plot(dest>dest.max()*0.01)
# plt.show()

