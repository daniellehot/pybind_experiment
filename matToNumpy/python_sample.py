import converter as c
import cv2 as cv
import numpy as np

img = c.readImage()
print("Python sample")
print(type(img))
print(img.shape)
print(img.dtype)
cv.imshow("temp", img)
cv.waitKey(0)