import numpy as np
import cv2

img = cv2.imread('image.png')
cv2.imshow('image',img)
cv2.waitKey(0)


ret, corners = cv2.findChessboardCorners(img, (5,4)) # 8 6 are the dimensions of the chessboard

cv2.destroyAllWindows()
print(corners)
