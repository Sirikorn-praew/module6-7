import cv2
import numpy as np
import math
from transform import four_point_transform, order_points

import argparse
import imutils
import glob
from skimage import img_as_bool, io, color, morphology
from matplotlib import pyplot as plt

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)


cameraMatrix = np.array([[1395.3709390074625, 0.0, 984.6248356317226], [0.0, 1396.2122002126725, 534.9517311724618], [0.0, 0.0, 1.0]], np.float32) # Humanoid
dist = np.array([[0.1097213194870457, -0.1989645299789654, -0.002106454674127449, 0.004428959364733587, 0.06865838341764481]]) # Humanoid
rvec = np.array([0.0, 0.0, 0.0]) # float only
tvec = np.array([0.0, 0.0, 0.0]) # float only



parameters =  cv2.aruco.DetectorParameters_create()

markerLength = 0.04
markerSeparation = 0.01

board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=markerLength, markerSeparation=markerSeparation, dictionary=dictionary)
backSub = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=16, detectShadows=False)

img_list, mask_list = [], []

visualize = True

def drawBox(frame, rvec, tvec, size = 0.4):
    objpts = np.float32([[0,0,0], [1,0,0], [1,1,0], [0,1,0],
                         [0,0,1], [1,0,1], [1,1,1], [0,1,1]]).reshape(-1,3) * size
    imgpts, jac = cv2.projectPoints(objpts, rvec, tvec, cameraMatrix, dist)

    cv2.line(frame, tuple(imgpts[0].ravel()), tuple(imgpts[1].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[1].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[2].ravel()), tuple(imgpts[3].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (0,0,255), 2)

def aruco():
    i=0
    frame_counter = 0
    mode = True
    # while True:
        # _, frame = cap.read()
    for i in range(6):
        frame = cv2.imread("./img/0" + str(i) + ".jpg")
        # print(i)
        # cv2.imshow("img"+ str(i) ,frame)
        # cv2.waitKey(0)
        original = frame.copy()
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
        if markerIds is not None:
            ret, _, _ = cv2.aruco.estimatePoseBoard(corners=markerCorners, ids=markerIds, board=board, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec)
            if ret:
                
                # cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1) # origin
                T_marker = np.array([markerLength, markerLength, 0.0])
                A = np.array([0.0, 0.0, 0.0]) + T_marker
                B = np.array([0.4 + markerSeparation, 0.0, 0.0]) + T_marker
                C = np.array([0.4 + markerSeparation, 0.4 + markerSeparation, 0.0]) + T_marker
                D = np.array([0.0, 0.4 + markerSeparation, 0.0]) + T_marker
                ### Find Transformatio Matrix ###
                rotM = np.zeros(shape=(3,3))
                cv2.Rodrigues(rvec, rotM, jacobian = 0)
                ### Map to image coordinate ###
                pts, jac = cv2.projectPoints(np.float32([A, B, C, D]).reshape(-1,3), rvec, tvec, cameraMatrix, dist)
                pts = np.array([tuple(pts[i].ravel()) for i in range(4)], dtype = "float32")
                pts = order_points(pts)
                ### Draw axis ###
                for point in [A, B, C, D]: cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec + np.dot(point, rotM.T), length=0.1)
                ### Draw work space ###
                # drawBox(frame, rvec, tvec + np.dot(A, rotM.T), size=0.4 + markerSeparation)

            ## Fill Marker ##
            for corner, id in zip(markerCorners, markerIds):
                points = [(int(point[0]), int(point[1])) for point in corner[0]]
                ids = id[0]
                pts1 = np.array(points, np.int32)
                cv2.fillPoly(frame, [pts1], 255)
            ## Perspective Crop ##
            warped = four_point_transform(original, pts)
            warped = cv2.resize(warped, (800, 800))
            valid_mask = four_point_transform(np.ones(original.shape[:2], dtype="uint8") * 255, pts)
            valid_mask = cv2.resize(valid_mask, (800, 800))
            # cv2.imshow("Warped", warped)
            cv2.imwrite("./img/" + str(i) + "_out.jpg", warped)
            # cv2.imshow("Valid", valid_mask)

            frame_counter += 1
            if mode and frame_counter%1==0:
                if frame_counter < 1000:
                    mean_canvas = np.ones(warped.shape, dtype="uint8") * 200
                    mean_canvas = cv2.bitwise_or(mean_canvas, mean_canvas, mask=cv2.bitwise_not(valid_mask))
                else:
                    mean_canvas = cv2.bitwise_or(bg, bg, mask=cv2.bitwise_not(valid_mask))
                valid_mask = cv2.bitwise_or(warped, warped, mask=valid_mask)
                final = cv2.bitwise_or(mean_canvas, valid_mask)
                # cv2.imshow('Passed', final)

                fgMask = backSub.apply(cv2.GaussianBlur(final,(5,5),0))
                fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_CLOSE, kernel=np.ones((5,5),np.uint8))
                # fgMask += 255-valid_mask
                # cv2.imshow('FG Mask', fgMask)

                # img_list.append(warped)
                # mask_list.append(fgMask)

                bg = backSub.getBackgroundImage()
                # cv2.imshow('BG', bg)
            
        # cv2.imshow("Preview", frame)
        # cv2.imshow("marker33", markerImage)
        key = cv2.waitKey(1)
        if key == 27:
            break
        if key == ord('m'):
            mode = not mode
        if key == ord(' '):
            cv2.imwrite(str(i) + "_mask.jpg", fgMask)
            cv2.imwrite(str(i) + ".jpg", warped)
            i+=1
        if key == ord('g'):
            result = calculateBG(img_list, mask_list)
            # cv2.imshow("BG", result)
            cv2.waitKey(0)
    # cap.release()
    # cv2.destroyAllWindows()

def median():
    imgs = []
    for i in [0,1,2,3,4,5]:
        imgs.append(cv2.imread("./img/" + str(i) + "_out.jpg"))
    # Convert images to 4d ndarray, size(n, nrows, ncols, 3)
    imgs = np.asarray(imgs)

    # Take the median over the first dim
    med = np.median(imgs, axis=0)
    cv2.imwrite("Realy.jpg",med)
    # print("realyyyyyyyyyyyyyy")
    # cv2.imshow("Med", np.asarray(med, np.uint8))
    # cv2.waitKey(0)

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = np.ones_like(image)*255
    mask = cv2.warpAffine(mask, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = cv2.erode(mask, np.ones((9, 9)))
    result_edge = cv2.Canny(result, 50, 200)
    result_edge = cv2.bitwise_and(mask,result_edge)
    return result_edge

def FindPath():
    mix_point = []
    # load the image image, convert it to grayscale, and detect edges
    Card = cv2.imread("card.jpg")
    template_gray = cv2.cvtColor(Card, cv2.COLOR_BGR2GRAY)

    (tH, tW) = template_gray.shape[:2]
    # cv2.imshow("card.jpg", Card)

    # load the image, convert it to grayscale, and initialize the
    # bookkeeping variable to keep track of the matched region
    image = cv2.imread("Realy.jpg")
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
            # cv2.imshow("Template", template)
            # cv2.imshow("Visualize", clone)
            # cv2.waitKey(10)
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
    if found is None or maxVal > found[0]:
        found = (maxVal, maxLoc, r)
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))

    center_rec = [round(((startX+endX)/2)*0.5), round(((startX+endX)/2)*0.5)]
    # print(center_rec)
    # print(startX)
    # print(startY)
    # print(endX)
    # print(endY)
    # draw a bounding box around the detected result and display the image
    cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
    # cv2.imshow("Image", image)
    # cv2.waitKey(0)

    ######################### Find chessboard ###################################

    def rotate_chess(image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        mask = np.ones_like(image)*255
        mask = cv2.warpAffine(mask, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        mask = cv2.bitwise_not(mask)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        result = cv2.bitwise_or(result, mask)
        _, result_thresh = cv2.threshold(result,127,255,cv2.THRESH_BINARY)
        return result_thresh

    # load the image image, convert it to grayscale, and detect edges
    Chessboard = cv2.imread("chessboard.png")
    template_gray_chess = cv2.cvtColor(Chessboard, cv2.COLOR_BGR2GRAY)

    (tH_chess, tW_chess) = template_gray_chess.shape[:2]
    # cv2.imshow("chessboard", Chessboard)

    # load the image, convert it to grayscale, and initialize the
    # bookkeeping variable to keep track of the matched region

    # loop over the scales of the image
    for chessrotate in np.arange(0, 90, 5):
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized = image
        template = rotate_chess(template_gray_chess, chessrotate)
        r = gray.shape[1] / float(resized.shape[1])
        # if the resized image is smaller than the template, then break
        # from the loop
        if resized.shape[0] < tH_chess or resized.shape[1] < tW_chess:
            break
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        _, resized_thresh = cv2.threshold(resized,127,255,cv2.THRESH_BINARY)
        # cv2.imshow("A", resized_thresh)
        # cv2.imshow("B", template)
        # cv2.waitKey(1)
        result = cv2.matchTemplate(resized_thresh, template, cv2.TM_CCOEFF)
        (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
        # check to see if the iteration should be visualized

        if visualize:
            clone = np.dstack([resized, resized, resized])
            cv2.rectangle(clone, (maxLoc[0], maxLoc[1]), (maxLoc[0] + tW_chess, maxLoc[1] + tH_chess), (0, 0, 255), 2)
            # cv2.imshow("Template", template)
            # cv2.imshow("Visualize", clone)
            # cv2.waitKey(10)
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
    if found is None or maxVal > found[0]:
        found = (maxVal, maxLoc, r)
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW_chess) * r), int((maxLoc[1] + tH_chess) * r))

    center_rec_chess = [round(((startX+endX)/2)*0.5), round(((startX+endX)/2)*0.5)]
    # print(center_rec_chess)
    # print(startX)
    # print(startY)
    # print(endX)
    # print(endY)
    # draw a bounding box around the detected result and display the image
    cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
    # cv2.imshow("Image", image)
    # cv2.waitKey(0)

    ######################### Find path ###################################
    frame3 = cv2.imread("Realy.jpg")
    kernel = np.ones((5,5),np.uint8)
    gray = cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY)
    gray_original = gray.copy()
    dilation = cv2.dilate(frame3,kernel,iterations = 18)
    erosion = cv2.erode(dilation,kernel,iterations = 16)

    # cv2.imshow("erosion",erosion)

    K = 3
    attempts=10

    img=cv2.cvtColor(erosion,cv2.COLOR_BGR2HSV)

    vectorized = img.reshape((-1,3))
    vectorized = np.float32(vectorized)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

    ret,label,center=cv2.kmeans(vectorized,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS) #  cv.KMEANS_PP_CENTERS and cv.KMEANS_RANDOM_CENTERS.
    center = np.uint8(center)
    res = center[label.flatten()]
    result_image = res.reshape((img.shape))

    # figure_size = 15
    # plt.figure(figsize=(figure_size,figure_size))
    # plt.subplot(1,2,1),plt.imshow(img)
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(1,2,2),plt.imshow(result_image)
    # plt.title('Segmented Image when K = %i' % K), plt.xticks([]), plt.yticks([])
    # plt.show()

    rgb = cv2.cvtColor(result_image, cv2.COLOR_HSV2RGB)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

    ret, thresh = cv2.threshold(gray,180,255,cv2.THRESH_BINARY_INV)
    # cv2.imshow("thresh",thresh)

    contour,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # print(len(contour))
    thresh_new = np.zeros_like(gray)

    for c in contour:
        area = cv2.contourArea(c)
        # print("area")
        # print(area)

        if area > 200:
            cv2.drawContours(thresh_new, [c], -1, 255, -1)

    thresh = thresh_new/255
    # cv2.imshow("th",thresh)
    # cv2.waitKey()
    out = morphology.skeletonize(thresh,method="lee") 
    # cv2.imshow("ske",out)

    points_skeleton = np.column_stack(np.where(out.transpose() != 0) )# get array of points
    poly_points = cv2.approxPolyDP(points_skeleton, 0.02 * frame3.shape[0], False) # approximate polygon
    # print("poly_points")
    # print(poly_points)

    canva2 = np.zeros(frame3.shape[:2], dtype="uint8")
    canva2 = cv2.polylines(canva2,[poly_points],False,(255,255,255),1)
    # cv2.imshow("canva",canva2)
    
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
        # print(poly_points.shape)
        x0 = poly_points[b][0][0]
        x0 = x0*0.5
        y0 = poly_points[b][0][1]
        y0 = y0*0.5
        x1 = poly_points[b+1][0][0]
        x1 = x1*0.5
        y1 = poly_points[b+1][0][1]
        y1 = y1*0.5
        
        x = x1 - x0
        y = y1 - y0
        radian_theta = math.atan2(y, x) #theta = math.atan2(y[i], x[i])
        ongsa_theta = round(radian_theta*180/math.pi+90)
        points.append([round(x0), round(y0),Z_real[b],ongsa_theta])
    points.append([round(x1), round(y1), Z_real[-1], ongsa_theta])
    # print(points)
    
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # for i in range(len(points)): ax.scatter(points[i][0], points[i][1], points[i][2])
    # ax.view_init(azim=-120, elev=60)
    # plt.show()

    for i in points:
        # print(i)
        cv2.circle(frame3,(i[0],i[1]),3,(255,0,0),-1)
        cv2.waitKey(0)
    mix_point.append([center_rec[0], center_rec[1], Z_real[0], 0])
    mix_point += points
    mix_point.append([center_rec_chess[0], center_rec_chess[1], Z_real[-1], ongsa_theta])

    print(mix_point)
    return mix_point


# aruco()
# median()
# print(FindPath())