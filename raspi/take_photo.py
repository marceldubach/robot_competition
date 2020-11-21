import cv2 as cv

cap = cv.VideoCapture();

ret,frame = cap.read()

cv.imshow('img1',frame)
cv.waitKey(0)

cv.destroyAllWindows()