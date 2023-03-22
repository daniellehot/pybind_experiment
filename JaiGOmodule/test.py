import pyJaiGo as jai
import numpy as np
import cv2

camera = jai.JaiGo()

camera.FindAndConnect()
if camera.Connected:
    camera.StartStream()
    i = 0
    while camera.Streaming:
        i+=1
        
        if camera.GetImage():
            img = camera.Img
            print("Iteration ", i, " Image shape is ", np.shape(img))
            cv2.imshow("test", img)
            cv2.waitKey(1)

        if i == 100:
            camera.Streaming = False

camera.CloseAndDisconnect()

