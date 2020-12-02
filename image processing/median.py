import numpy as np
import cv2
imgs = []
# for i in range(9):
for i in [0,1,2,3,4,5]:
    imgs.append(cv2.imread("./img2/" + str(i) + "_out.jpg"))
# Convert images to 4d ndarray, size(n, nrows, ncols, 3)
imgs = np.asarray(imgs)

# Take the median over the first dim
med = np.median(imgs, axis=0)
cv2.imwrite("Realy.jpg",med)
cv2.imshow("Med", np.asarray(med, np.uint8))
cv2.waitKey(0)