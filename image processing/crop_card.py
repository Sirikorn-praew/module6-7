import cv2
import imutils
import numpy as np
import camera
#import class camera
def run():
    cap = camera.Camera()
    #set up camera to Full HD

    while True:
        frame = cap.read()
        rect_mask = cv2.rectangle(np.ones_like(frame)*255, (860, 440), (1060, 640), (0,0,0), -1)
        alpha = 0.5
        cv2.addWeighted(rect_mask, alpha, frame, 1-alpha,0,frame)
        cv2.imshow("rect_mask",frame)
        key = cv2.waitKey(1)
        if key == ord(' '):
            resized = imutils.resize(frame[440:640, 860:1060], height=200)
            return cv2.imwrite("card.jpg", resized)


    
if __name__ == "__main__":
    run()