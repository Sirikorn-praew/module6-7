import cv2
import numpy as np
import math
from skimage import img_as_bool, io, color, morphology
from matplotlib import pyplot as plt
# Convert BGR to HSV
# frame3 = cv2.imread("Real5.png")
frame3 = cv2.imread("Realx.jpg")
kernel = np.ones((5,5),np.uint8)
gray = cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY)
gray_original = gray.copy()
dilation = cv2.dilate(frame3,kernel,iterations = 18)
erosion = cv2.erode(dilation,kernel,iterations = 16)
# canny = cv2.Canny(erosion, 20, 100)
# contour, aaa = cv2.findContours(canny,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
# b = np.zeros(frame.shape[:2], dtype="uint8") #For visualize card border
# print (len(contours)/2)
# for i in range(len(contours)): #Draw each contours
#       cv2.drawContours(b, contour, i, 255, 6)
# imshow(black)

# hsv = cv2.cvtColor(canny, cv2.COLOR_BGR2HSV)
# mask3 = cv2.inRange(hsv,(0, 56, 82), (180, 255, 255) )

# Bitwise-AND mask and original image
#res = cv2.bitwise_and(frame3,frame3, mask= mask3)

K = 2
attempts=10

# img = cv2.imread("Real5.png")
img=cv2.cvtColor(erosion,cv2.COLOR_BGR2HSV)

vectorized = img.reshape((-1,3))
vectorized = np.float32(vectorized)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

ret,label,center=cv2.kmeans(vectorized,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS) #  cv.KMEANS_PP_CENTERS and cv.KMEANS_RANDOM_CENTERS.
center = np.uint8(center)
res = center[label.flatten()]
result_image = res.reshape((img.shape))

rgb = cv2.cvtColor(result_image, cv2.COLOR_HSV2RGB)
gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

ret, thresh = cv2.threshold(gray,170,255,cv2.THRESH_BINARY_INV)
# cv2.imshow("thresh",thresh)

contour,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
print(len(contour))
thresh_new = np.zeros_like(gray)
for c in contour:
    area = cv2.contourArea(c)
    print("area")
    print(area)

    if area > 200:
        cv2.drawContours(thresh_new, [c], -1, 255, -1)

thresh = thresh_new/255

out = morphology.skeletonize(thresh) 

points_skeleton = np.column_stack(np.where(out.transpose() != 0) )# get array of points
poly_points = cv2.approxPolyDP(points_skeleton, 0.02 * frame3.shape[0], False) # approximate polygon

canva2 = np.zeros(frame3.shape[:2], dtype="uint8")
canva2 = cv2.polylines(canva2,[poly_points],False,(255,255,255),1)
# imshow(canva2)
   
#find height
max_height = 200 #mm
min_height = 100 #mm

Z = [] 

points = []
intensities = []
heights = []

path_points = []
path_heights = []
 
for point in points_skeleton:
    path_points.append((point[0],point[1]))
    intensities.append(gray_original[point[1]][point[0]])

sorted_intensities = sorted(intensities)
height_range = max_height - min_height
hist, bins = np.histogram(intensities, 256, [0, 256])
cdf = hist.cumsum()

cdf_max = cdf[-1]
cdf_thresh_min = int(cdf_max*0.10)
cdf_thresh_max = int(cdf_max*0.90)
# print(len(intensities), cdf_thresh_max)
height_thresh_min = sorted_intensities[cdf_thresh_min]
height_thresh_max = sorted_intensities[cdf_thresh_max]
height_thresh_range = height_thresh_max - height_thresh_min

# print(height_thresh_range)

if height_thresh_range < 23: ##change parameter eang
    Z = [-1 for i in range (len(points))]
else:
    Z = []

    for intensity in intensities:
        if intensity <= height_thresh_min: Z.append(min_height)
        elif intensity >= height_thresh_max: Z.append(max_height)
        else: Z.append((intensity - height_thresh_min) * height_range / height_thresh_range + min_height)
    # print(Z)
Z_real = []
for p in poly_points:
    best = Z[0]
    cost = math.sqrt((p[0][0]-path_points[0][0])**2 + (p[0][1]-path_points[0][1])**2)
    for i, point in enumerate(path_points):
        new_cost = math.sqrt((p[0][0]-point[0])**2 + (p[0][1]-point[1])**2)   
        if new_cost < cost:
            cost = new_cost
            best =  Z[i]
    Z_real.append(round(best))
    heights.append(best)
    # print(len(p))
    
for b in range(len(poly_points)-1):
    # if b == b[-1]:continue
    print(poly_points.shape)
    x0 = poly_points[b][0][0]
    y0 = poly_points[b][0][1]
    x1 = poly_points[b+1][0][0]
    y1 = poly_points[b+1][0][1]
    
    x = x1 - x0
    y = y1 - y0
    radian_theta = math.atan2(y, x) #theta = math.atan2(y[i], x[i])
    ongsa_theta = round(radian_theta*180/math.pi+90)
    points.append((x0, y0,Z_real[b],ongsa_theta))
points.append((x1, y1, Z_real[-1], ongsa_theta))
print(points)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(len(points)): ax.scatter(points[i][0], points[i][1], points[i][2])
ax.view_init(azim=-120, elev=60)
plt.show()

figure_size = 15
plt.figure(figsize=(figure_size,figure_size))
plt.subplot(1,2,1),plt.imshow(img)
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(1,2,2),plt.imshow(result_image)
plt.title('Segmented Image when K = %i' % K), plt.xticks([]), plt.yticks([])
plt.show()

cv2.imshow("frame3",frame3)
cv2.imshow("dilation",dilation)
cv2.imshow("erosion",erosion)
cv2.imshow("thresh",thresh)
cv2.imshow("canva2",canva2)
cv2.waitKey(0)  
#closing all open windows  
cv2.destroyAllWindows()  