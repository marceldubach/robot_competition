# THIS CODE IS NOT USED...

# # PART 1: BRICK DETECTION
# # apply a Gaussian blur on the colored image, then convert it to GrayScale
#
#
# gray_blurred = cv.cvtColor(img_blurred, cv.COLOR_BGR2GRAY)
#
# # EDGE DETECTION (MAYBE OMIT THIS)
# ddepth = cv.CV_16S
# # Scharr filter works better than sobel
# grad_x = cv.Scharr(gray_blurred, ddepth, 1, 0)
# grad_y = cv.Scharr(gray_blurred, ddepth, 0,1)
#
# abs_grad_x = cv.convertScaleAbs(grad_x)
# abs_grad_y = cv.convertScaleAbs(grad_y)
#
# grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
# ret, grad_thresh = cv.threshold(grad,100,255,0)
# element = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
# grad_thresh = cv.erode(grad_thresh, element)
#
#
# find the contours of a brick
# contours, hierachy = cv.findContours(grad_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

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