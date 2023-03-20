import sample
import cv2 as cv
import numpy as np

img = sample.readImage()
img=img.astype(np.uint8)
print("Python sample")
print(type(img))
print(img.shape)
print(img.dtype)
cv.imshow("temp", img)
cv.waitKey(0)