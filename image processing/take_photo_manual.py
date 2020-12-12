import cv2
import imutils
import numpy as np
import camera
#import class camera

cap = cv2.VideoCapture(cv2.CAP_DSHOW+2)
cap.set(cv2.CAP_PROP_SETTINGS, 1) # Open setup window
codec = 0x47504A4D  # MJPG
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap.set(3, 1920)
cap.set(4, 1080)
#set up camera to Full HD

while True:
    frame = cap.read()[1]
    cv2.imshow("A", imutils.resize(frame, height=720))
    key = cv2.waitKey(1)
    if key == ord(' '):
        cv2.imwrite("f.jpg", frame)
    if key == 27:
        cv2.destroyAllWindows()
        break


    
