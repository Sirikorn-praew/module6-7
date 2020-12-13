import argparse
import imutils
import glob
import cv2
import numpy as np
import math
from skimage import img_as_bool, io, color, morphology
from skimage.morphology import skeletonize
from matplotlib import pyplot as plt
from scipy.ndimage import generic_filter

visualize = True
def distance(p1,p2):
    return abs((p1[0]-p2[0])+(p1[1]-p2[1]))
def sortPoint(points, start):
    # Convert array to list (Be able to use remove())
    start = (start[0], start[1])
    point_list = []
    for point in points:
        point_list.append((point[0], point[1]))
    points = list(points)
    new_points = []
    best = {}
    present_point = start
    new_points.append(start)
    point_list.remove(start)
    while len(point_list):
        best['vector'] = point_list[0]
        best['cost'] = distance(present_point, best['vector'])
        for point in point_list:
            cost = distance(point, present_point)
            if best['cost'] > cost: # Update better value (Nearest to present point)
                best['vector'] = point
                best['cost'] = cost
        new_points.append(best['vector'])
        point_list.remove(best['vector'])
        present_point = best['vector']
    return np.asarray(new_points, dtype=np.int32)

def Endline(P):
    """Central pixel and just one other must be set to be a line end"""
    return 255 * ((P[4]==255) and np.sum(P)==510)

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = np.ones_like(image)*255
    mask = cv2.warpAffine(mask, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = cv2.erode(mask, np.ones((9, 9)))
    result_edge = cv2.Canny(result, 50, 200)       
    result_edge = cv2.bitwise_and(mask,result_edge)
    # cv2.imshow("A", result_edge)
    # cv2.waitKey(10)
    return result_edge
def run():
    mix_point = []
    # load the image image, convert it to grayscale, and detect edges
    Card = cv2.imread("card.png") 
    template_gray = cv2.cvtColor(Card, cv2.COLOR_BGR2GRAY)

    (tH, tW) = template_gray.shape[:2]
    # cv2.imshow("card.jpg", Card)

    # load the image, convert it to grayscale, and initialize the
    # bookkeeping variable to keep track of the matched region
    image = cv2.imread("Real00.jpg")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5),0)
    unsharp = cv2.addWeighted(gray, 1.5, blurred, -0.5, 0)
    cv2.imshow("Unsharp", unsharp)
    cv2.waitKey(0)
    found = None
    # loop over the scales of the image
    for rotate in np.arange(0, 90, 1):
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized = imutils.resize(image, height=int(1*image.shape[0]))
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

    center_rec = round(((startX+endX)/2)*0.5), round(((startY+endY)/2)*0.5)
    print("Center rect:" + str(center_rec))
    print(center_rec)
    # print(startX)
    # print(startY)
    # print(endX)
    # print(endY)
    # draw a bounding box around the detected result and display the image
    clone_final = image.copy()
    cv2.rectangle(clone_final, (startX, startY), (endX, endY), (0, 0, 255), 2)
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
    # image = cv2.imread("Realx.jpg")
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # found = None
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
        cv2.imshow("A", resized_thresh)
        cv2.imshow("B", template)
        cv2.waitKey(1)
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
        print(found)
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW_chess) * r), int((maxLoc[1] + tH_chess) * r))

    center_rec_chess = round(((startX+endX)/2)*0.5), round(((startY+endY)/2)*0.5)
    print("Center chess:" + str(center_rec_chess))
    # print(center_rec_chess)
    # print(startX)
    # print(startY)
    # print(endX)
    # print(endY)
    # draw a bounding box around the detected result and display the image
    cv2.rectangle(clone_final, (startX, startY), (endX, endY), (0, 0, 255), 2)
    cv2.imshow("final",clone_final)
    # cv2.imshow("Image", image)
    # cv2.waitKey(0)

    ######################### Find path ###################################
    # frame3 = cv2.imread("Real5.png")
    frame3 = image.copy()
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

    img=frame3
    mask = np.zeros(img.shape[:2],np.uint8)
    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)

    drawingL = False # true if mouse is pressed
    drawingR = False
    mode = False # if True, draw rectangle. Press 'm' to toggle to curve
    ix,iy = -1,-1
    # mouse callback function
    def draw_circle(event,x,y,flags,param):
        global ix,iy,drawingL, drawingR,mode
        if event == cv2.EVENT_LBUTTONDOWN:
            drawingL = True
            ix,iy = x,y
        elif event == cv2.EVENT_MOUSEMOVE:
            if drawingL == True:
                cv2.circle(mask,(x,y),5,1,-1)
        elif event == cv2.EVENT_LBUTTONUP:
            drawingL = False
            cv2.circle(mask,(x,y),5,1,-1)
        if event == cv2.EVENT_RBUTTONDOWN:
            drawingR = True
            ix,iy = x,y
        elif event == cv2.EVENT_MOUSEMOVE:
            if drawingR == True:
                cv2.circle(mask,(x,y),5,0,-1)
        elif event == cv2.EVENT_RBUTTONUP:
            drawingR = False
            cv2.circle(mask,(x,y),5,0,-1)
    mask = np.ones(img.shape[:2], np.uint8)*2
    cv2.namedWindow('image')
    cv2.imshow("image", img)
    cv2.setMouseCallback('image',draw_circle)

    while(1):
        cv2.imshow('image',img)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('m'):
            mode = not mode
        elif k == 27:
            break
    cv2.destroyAllWindows()
    mask, bgdModel, fgdModel = cv2.grabCut(img,mask,None,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_MASK)
    mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    thresh = mask2*255
    cv2.imshow("A", thresh)
    cv2.waitKey(0)
    # rgb = cv2.cvtColor(result_image, cv2.COLOR_HSV2RGB)
    # gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

    # ret, thresh = cv2.threshold(gray,170,255,cv2.THRESH_BINARY_INV)
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

    thresh = np.asarray(thresh_new, dtype=np.uint8)
    contour,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    base_area = cv2.contourArea(contour[0])
    erode = thresh.copy()
    while True:
        erode = cv2.erode(erode, np.ones((3, 3)))
        contour,_ = cv2.findContours(erode,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour) != 1 or cv2.contourArea(contour[0]) < 0.1*base_area: break
        eroded = erode.copy()
        cv2.imshow("A", eroded)
        cv2.waitKey(1)

    out = morphology.skeletonize(eroded,method='lee')
    cv2.imshow("out",out) 
    cv2.imshow("thresh",thresh)
    cv2.waitKey(0)

    # figure_size = 15
    # plt.figure(figsize=(figure_size,figure_size))
    # plt.subplot(1,2,1),plt.imshow(img)
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(1,2,2),plt.imshow(result_image)
    # plt.title('Segmented Image when K = %i' % K), plt.xticks([]), plt.yticks([])
    # plt.show()

    # cv2.imshow("frame3",frame3)
    # cv2.imshow("dilation",dilation)
    # cv2.imshow("erosion",erosion)
    # cv2.imshow("thresh",thresh)
    # cv2.waitKey(0)  

    points_skeleton = np.column_stack(np.where(out.transpose() != 0) )# get array of points
    end_of_line_canvas = generic_filter(out, Endline, (3, 3))  # find end of line
    end_of_line_points = np.column_stack(np.where(end_of_line_canvas.transpose() != 0))  # get points from canvas
    # print(end_of_line_points)
    points = sortPoint(points_skeleton, start=end_of_line_points[0])
    points_cnt = points.reshape((points.shape[0], 1,points.shape[1]))
    poly_points = cv2.approxPolyDP(points_cnt, 0.02 * frame3.shape[0], False) # approximate polygon
    # print(poly_points)
    points_skeleton=points
    canva2 = np.zeros(frame3.shape[:2], dtype="uint8")
    canva2 = cv2.polylines(canva2,[poly_points],False,(255,255,255),1)
    # cv2.imshow("canva2",canva2)
    # cv2.waitKey(0)
    
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

    sorted_intensities = sorted(intensities.copy())
    height_range = max_height - min_height
    hist, bins = np.histogram(intensities, 256, [0, 256])
    cdf = hist.cumsum()
    cdf_max = cdf[-1]
    cdf_thresh_min = int(cdf_max*0.15)
    cdf_thresh_max = int(cdf_max*0.85)
    # print(len(intensities), cdf_thresh_max)
    height_thresh_min = sorted_intensities[cdf_thresh_min]
    height_thresh_max = sorted_intensities[cdf_thresh_max]
    height_thresh_range = height_thresh_max - height_thresh_min
    # print(height_thresh_range)
    # print(intensities)
    # print(height_thresh_range)

    Z = []
    if height_thresh_range < 16: ##change parameter eang
        print("constant height mode")
        Z = [max_height for i in range (len(intensities))]
    else:
        print("linear height mode")
        for intensity in intensities:
            # print(intensity)
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
        Z_real.append(round(300-best))
        heights.append(300-best)
        # print(len(p))
        
    for b in range(len(poly_points)-1):
        # if b == b[-1]:continue
        print(poly_points.shape)
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
    print(points)
    mix_point.insert(0, [center_rec[0], center_rec[1], Z_real[0], 0])

    point = mix_point[-1]
    if abs(point[0]-points[0][0]) + abs(point[1]-points[0][1]) < abs(point[0]-points[-1][0]) + abs(point[1]-points[-1][1]):
        mix_point += points
    else: 
        mix_point += reversed(points)    
    mix_point.append([center_rec_chess[0], center_rec_chess[1], Z_real[-1], 0])

    print(mix_point)
    canvas = np.ones((800, 800, 3))
    for i in range(len(mix_point)): 
        x, y = mix_point[i][0]*2, mix_point[i][1]*2
        cv2.circle(canvas, (int(x), int(y)), 5, (0, 255, 0), -1)
        if i < len(mix_point)-1:
            x1 = mix_point[i+1][0]*2
            y1 = mix_point[i+1][1]*2
            cv2.line(canvas, (int(x), int(y)), (int(x1), int(y1)), (0, 0, 255), 2)
            cv2.putText(canvas, str(mix_point[i][2]),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 0),3)
    cv2.imshow("Path", canvas)
    cv2.waitKey(0)
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # for i in range(len(points)): ax.scatter(points[i][0], points[i][1], points[i][2])
    # ax.view_init(azim=-120, elev=60)
    # plt.show()

    # figure_size = 15
    # plt.figure(figsize=(figure_size,figure_size))
    # plt.subplot(1,2,1),plt.imshow(img)
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(1,2,2),plt.imshow(result_image)
    # plt.title('Segmented Image when K = %i' % K), plt.xticks([]), plt.yticks([])
    # plt.show()

    # cv2.imshow("frame3",frame3)
    # cv2.imshow("dilation",dilation)
    # cv2.imshow("erosion",erosion)
    # cv2.imshow("thresh",thresh)
    # cv2.imshow("canva2",canva2)
    # cv2.waitKey(0)  
    #closing all open windows  
    cv2.destroyAllWindows()
    return mix_point
run()