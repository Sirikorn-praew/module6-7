import cv2

class Camera():
    def __init__(self):
        self.cap = cv2.VideoCapture(cv2.CAP_DSHOW+2)
        self.cap.set(cv2.CAP_PROP_SETTINGS, 1)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_SETTINGS, 1) # Open setup window
        self.codec = 0x47504A4D  # MJPG #set up camera to Full HD
        self.cap.set(cv2.CAP_PROP_FPS, 30.0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_SETTINGS, 1)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 2)
        self.cap.set(3, 1920)
        self.cap.set(4, 1080)
    def read(self):
        return self.cap.read()[1]
