import cv2
import imutils
import numpy as np

frame = cv2.imread("Realy.jpg")
cap = cv2.VideoCapture(cv2.CAP_DSHOW+2)
codec = 0x47504A4D  # MJPG
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(3, 1920)
cap.set(4, 1080)
#set up camera to Full HD

while True:
    frame = cap.read()[1]

    alpha = 0.5
    rect_mask = cv2.rectangle(np.ones_like(frame)*255, (860, 440), (1060, 640), (0,0,0), -1)
    cv2.addWeighted(rect_mask, alpha, frame, 1-alpha,0,frame)
    cv2.imshow("rect_mask",frame)
    key = cv2.waitKey(1)
    if key == ord(' '):
        resized = imutils.resize(frame[440:640, 860:1060], height=200)
        cv2.imwrite("card.jpg", resized)


    
