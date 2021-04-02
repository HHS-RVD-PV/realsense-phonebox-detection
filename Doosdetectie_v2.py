# -*- coding: utf-8 -*-
"""
author: Michael
"""
import cv2
import numpy as np
a = 0
k = 0
#foto maken door op y te klikken
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    if cv2.waitKey(1) & 0xFF == ord('y'): #save on pressing 'y' 
        image = frame.copy()
        break
cap.release()

#afbeelding verbeteren voor contouren
imgGray = cv2.cvtColor(colorimage,cv2.COLOR_BGR2GRAY)
imgBlur = cv2.GaussianBlur(imgGray,(7,7),0)
imgCanny = cv2.Canny(imgBlur,50,100)
kernel = np.ones((5,5))
imgDial = cv2.dilate(imgCanny,kernel,iterations=3)
imgThre = cv2.erode(imgDial,kernel,iterations=2)

cv2.imshow("gray", imgGray)
cv2.imshow("blur", imgBlur)
cv2.imshow("canny", imgCanny)
cv2.imshow("dialation", imgDial)
cv2.imshow("threshold", imgThre)

#contouren zoeken
contours, hiearchy = cv2.findContours(imgThre,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
for contour in contours: 
    opp = cv2.contourArea(contour) #oppervlakte pixels
    approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True) #voor niet perfecte rechhoek correctie
    lengte = len(approx) #aantal lijnen voor gesloten contour
   
    if opp < 220000 and lengte == 4:
        M = cv2.moments(contour) # met momenten het middelpunt bepalen
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(image, (cx,cy), (5), (99, 115, 0),5) #tekenen middelpunt
        cv2.drawContours(image, [approx], 0, (99, 115, 0), 2) #tekenen contouren van bijbehorende punten
        print("Opp:", opp)
        print("Middelpunt:",(cx,cy))
        while a < lengte: #hoekpunten tekenen
            cv2.circle(image, ((approx[a][0][0]),(approx[a][0][1])), (5), (99, 115, 0),5)
            a = a + 1;        
            
cv2.imshow("afbeelding", image)


toets = cv2.waitKey(0)  
cv2.destroyAllWindows()

