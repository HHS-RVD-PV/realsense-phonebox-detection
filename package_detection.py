# -*- coding: utf-8 -*-
"""
Created on Tue May  4 18:43:55 2021
RVD-minor: Project
Package detection 

Michael Koreneef
18089127
"""
import cv2
import numpy as np

def bigcontour(contours):
    biggest = np.array([])
    max_area = 0
    for i in contours:
        area = cv2.contourArea(i)
        if area > 500:
            arclen = cv2.arcLength(i, True) 
            approx = cv2.approxPolyDP(i, 0.02* arclen, True)
            print(approx)
            if area > max_area and len(approx) == 4:
                biggest = approx
                max_area = area
    return biggest, max_area

def reorder(points):
    points = points.reshape((4,2))
    pointsnew = np.zeros((4,1,2), dtype=np.int32)
    add = points.sum(1)
    
    pointsnew[0] = points[np.argmin(add)]
    pointsnew[3] = points[np.argmax(add)]
    diff = np.diff(points, axis=1)
    pointsnew[1] = points[np.argmin(diff)]
    pointsnew[2] = points[np.argmax(diff)]
    
    return pointsnew
#image read
image = cv2.imread('package1.jpeg',20)
cv2.imshow("Image", image)

# image to gray values
grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow("Gray image", grayImage)

# blurred image
blurImage = cv2.medianBlur(grayImage,9)
cv2.imshow("Blur image", blurImage)

cannyImage = cv2.Canny(blurImage,50,200)
cv2.imshow("Canny image", cannyImage)

kernel = np.ones((3,3))
imgDial = cv2.dilate(cannyImage,kernel,iterations=2)
imgThre = cv2.erode(imgDial,kernel,iterations=1)
cv2.imshow("dial image", imgDial)
cv2.imshow("erode image", imgThre)

cnts, hierarchy = cv2.findContours(imgThre, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contourImage = image.copy()
cv2.drawContours(contourImage, cnts, -1, (0, 0, 255), 1, cv2.LINE_AA)
cv2.imshow("contour image", contourImage)

biggest, maxarea = bigcontour(cnts)
print(biggest, maxarea)

if biggest.size !=0:
    biggestr = reorder(biggest)
    print(biggestr)
    
canvas = image.copy()
M = cv2.moments(biggest) 
cx = int(M["m10"] / M["m00"])
cy = int(M["m01"] / M["m00"])
cv2.circle(canvas, (cx,cy), (1), (0, 255, 0),3) #tekenen middelpunt
cv2.circle(canvas, (biggestr[0][0][0],biggestr[0][0][1]), (1), (255, 0, 0),3) #tekenen middelpunt
cv2.circle(canvas, (biggestr[1][0][0],biggestr[1][0][1]), (1), (0, 255, 0),3) #tekenen middelpunt
cv2.circle(canvas, (biggestr[2][0][0],biggestr[2][0][1]), (1), (0, 0, 255),3) #tekenen middelpunt
cv2.circle(canvas, (biggestr[3][0][0],biggestr[3][0][1]), (1), (255, 255, 0),3) #tekenen middelpunt

cv2.imshow("detected", canvas)

#depth calculations
w = 8.7 #width in cm
w1 = biggestr[1][0][0] - biggestr[0][0][0]
w2 = biggestr[3][0][0] - biggestr[2][0][0]
l1 = biggestr[2][0][1] - biggestr[0][0][1]
l2 = biggestr[3][0][1] - biggestr[1][0][1]
l = (l1 + l2)/2
pixelw1 = w/w1
pixelw2 = w/w2
pixell1 = pixelw1 * l
pixell2 = pixelw2 * l
geml = (pixell1 + pixell2)/2
centerg = geml/2

print(pixelw1, pixelw2)
print(pixell1, pixell2)
print(geml, centerg)
# destroy the windows by press any key.
key = cv2.waitKey(0)  
cv2.destroyAllWindows()