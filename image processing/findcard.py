import numpy as np
import argparse
import imutils
import glob
import cv2

visualize = True

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = np.ones_like(image)*255
    mask = cv2.warpAffine(mask, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = cv2.erode(mask, np.ones((9, 9)))
    result_edge = cv2.Canny(result, 50, 200)
    result_edge = cv2.bitwise_and(mask,result_edge)
    cv2.imshow("A", result_edge)
    cv2.waitKey(10)
    return result_edge

# load the image image, convert it to grayscale, and detect edges
Chessboard = cv2.imread("card.jpg")
template_gray = cv2.cvtColor(Chessboard, cv2.COLOR_BGR2GRAY)

(tH, tW) = template_gray.shape[:2]
cv2.imshow("card.jpg", Chessboard)

# load the image, convert it to grayscale, and initialize the
# bookkeeping variable to keep track of the matched region
image = cv2.imread("Realx.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
found = None
# loop over the scales of the image
for rotate in np.arange(0, 90, 1):
    # resize the image according to the scale, and keep track
    # of the ratio of the resizing
    resized = imutils.resize(image, height=int(1.2*image.shape[0]))
    template = rotate_image(template_gray, rotate)
    r = gray.shape[1] / float(resized.shape[1])
    # if the resized image is smaller than the template, then break
    # from the loop
    if resized.shape[0] < tH or resized.shape[1] < tW:
        break
    resized = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    edge = cv2.Canny(resized, 50, 200)
    result = cv2.matchTemplate(edge, template, cv2.TM_CCOEFF)
    (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
    # check to see if the iteration should be visualized

    if visualize:
        clone = np.dstack([resized, resized, resized])
        cv2.rectangle(clone, (maxLoc[0], maxLoc[1]), (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
        cv2.imshow("Template", template)
        cv2.imshow("Visualize", clone)
        cv2.waitKey(10)
    # if we have found a new maximum correlation value, then update
    # the bookkeeping variable
if found is None or maxVal > found[0]:
    found = (maxVal, maxLoc, r)
# unpack the bookkeeping variable and compute the (x, y) coordinates
# of the bounding box based on the resized ratio
(_, maxLoc, r) = found
(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
(endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))

center_rec = ((round(startX+endX)/2), (round(startX+endX)/2))
print(center_rec)
# print(startX)
# print(startY)
# print(endX)
# print(endY)
# draw a bounding box around the detected result and display the image
cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
cv2.imshow("Image", image)
cv2.waitKey(0)