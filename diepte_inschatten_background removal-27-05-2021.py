#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 21 14:06:40 2021

@author: wouter
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
def nothing(x):
    pass


a = 0
k = 0
i=0
pipeline=rs.pipeline()
config=rs.config()
cx=320 
cy=240
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 15)
size = []
'uncomment for recording' 
# rs.config.enable_device_from_file(config, "/home/wouter/Downloads/object_detection1.bag")
cfg = pipeline.start(config)
dev = cfg.get_device()

colorizer = rs.colorizer()
colorizer.set_option(rs.option.color_scheme, 0) #0 0-9

depth_sensor = dev.first_depth_sensor()
# depth_sensor.set_option(rs.option.visual_preset, 2)#2 0-5
emitter = depth_sensor.get_option(rs.option.emitter_enabled)
print("emitter = ", emitter)
set_emitter = 1
# depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
emitter1 = depth_sensor.get_option(rs.option.emitter_enabled)
print("new emitter = ", emitter1)
# Getting the depth sensor's depth scale (see rs-align example for explanation)

depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)
clipping_distance_in_meters = 1.0 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)
cv2.namedWindow("canny",cv2.WINDOW_AUTOSIZE)
cv2.namedWindow('RGB_RealSense', cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar("lower", "canny", 50, 1000, nothing)
cv2.createTrackbar("upper", "canny", 250, 1000, nothing)
cv2.createTrackbar("blur", "canny", 5, 1000, nothing)
cv2.createTrackbar("brightness", "RGB_RealSense", 800, 1000, nothing)

#eta = 0
try:
    while True:
        Hlines = []
        Vlines = []
        Horizontals = []
        Verticals = []
        MedianX = []
        ResultX = []
        ResetX = 0
        MedianY = []
        ResultY = []
        ResetY = 0
        Packages = []
        Coordinates = []
    
        lower = cv2.getTrackbarPos("lower", "canny")
        upper = cv2.getTrackbarPos("upper", "canny")
        #blur = cv2.getTrackbarPos("blur", "canny")
        beta = cv2.getTrackbarPos("brightness", "RGB_RealSense")
        # if blur % 2 == 0:
        #     blur = blur+1
        frames = pipeline.wait_for_frames()
        
        depth_frame = frames.get_depth_frame()
        
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        # color_frame = frames.get_color_frame()
        
        color_frame = aligned_frames.get_color_frame()
        # ir1_frame = frames.get_infrared_frame(1)
        if not depth_frame or not color_frame:
             continue
        # spatial = rs.spatial_filter()
        # spatial.set_option(rs.option.holes_fill, 1)
        # filtered_depth = spatial.process(depth_frame)
        # depth_image = depth_frame
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
     
        # depth_image = np.asanyarray(colorizer.colorize(depth_image).get_data())
        #depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # ir1_image = np.asanyarray(ir1_frame.get_data())
        #images = np.hstack((color_image, depth_image, ))
        grey_color = 100
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        alpha = 5# Simple contrast control
        beta = -1*beta    # Simple brightness control
        bg_removed = cv2.convertScaleAbs(bg_removed, alpha=alpha, beta=beta) 
        imgGray = cv2.cvtColor(bg_removed,cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray,(1,1),0)
        #imgBlur = cv2.medianBlur(imgGray,blur)
        imgCanny = cv2.Canny(imgBlur,lower,upper)
        
        kernel = np.ones((5,5))
        imgDial = cv2.dilate(imgCanny,kernel,iterations=1)
        # imgThre = cv2.erode(imgDial,kernel,iterations=2)

        Hlines=[]
        Vlines=[]
        for T in range(10):
            lines= cv2.HoughLinesP(image = imgDial, rho = 1, theta=np.pi/180.0, threshold =100, lines=np.array([]),minLineLength=100, maxLineGap=10)
            for line in lines:
                
                # x1,y1,x2,y2 = line[0]
       
                # cv2.line(bg_removed,(x1,y1),(x2,y2),(0,0,255),2)
                a,b,c = lines.shape
                for i in range(a):
                    AVert = math.degrees(math.atan(abs((lines[i,0,1]-lines[i,0,3])/(lines[i,0,0]-lines[i,0,2]))))
                    if (AVert < 5):
                        cv2.line(bg_removed, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 1, cv2.LINE_AA)
                        Hlines.append(lines[i][0][1])
                  
                    AVert = math.degrees(math.atan(abs((lines[i,0,0]-lines[i,0,2])/(lines[i,0,1]-lines[i,0,3]))))
                    if (AVert < 5):
                        cv2.line(bg_removed, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 1, cv2.LINE_AA)
                        Vlines.append(lines[i][0][0])
        Hlines.sort()
        Vlines.sort()
        while (len(Hlines) >= 60) and (ResetX != 2):          # 3 = aantal keer dat de eigenschap terugkomt
            for T in range(len(Hlines)):
                if Hlines[T] <= (Hlines[0] + 20):              # threshold waarbinnen de feature moet vallen
                    MedianX.append(Hlines[T])
                else:
                    if (len(MedianX)) >= 60:                # 3 = aantal keer dat de eigenschap terugkomt
                        ResultX.append(np.median(MedianX))
                    for L in range(len(MedianX)):
                        Hlines.pop(0)
                    MedianX = []
                    ResetX = 1
                if len(MedianX) > (len(Hlines)-1):
                    ResetX = 2                              # verlaat de forloop als er geen waardes meer in de list zijn die een lijn vormen
                    if np.median(MedianX) > ResultX[-1] + 50:
                        ResultX.append(np.median(MedianX))
                    break
                if ResetX == 1:                             # reset de for loop start weer bij 0
                    ResetX = 0
                    break
        # print(ResultX)
         ##if np.median(MedianX) > ResultX[-1] + 50:
        
        
        while (len(Vlines) >= 60) and (ResetY != 2):          # 3 = aantal keer dat de eigenschap terugkomt
            for T in range(len(Vlines)):
                if Vlines[T] <= (Vlines[0] + 20):              # threshold waarbinnen de feature moet vallen
                    MedianY.append(Vlines[T])
                else:
                    if (len(MedianY)) >= 60:                # 3 = aantal keer dat de eigenschap terugkomt
                        ResultY.append(np.median(MedianY))
                    for L in range(len(MedianY)):
                        Vlines.pop(0)
                    MedianY = []
                    ResetY = 1
                if len(MedianY) > (len(Vlines)-1):
                    ResetY = 2
                    if np.median(MedianY) > ResultY[-1] + 50:
                        ResultY.append(np.median(MedianY))
                    break
                if ResetY == 1:
                    ResetY = 0
                    break

        if len(ResultX) > 0:
            PackCoordX = np.zeros((len(ResultX)-1))
        else:
            PackCoordX = np.zeros(0)
        PackCoordY = np.zeros((len(ResultY)-1))
        
        
        if (len(ResultX) > 1) and (len(ResultY) > 1):
            for X in range(len(ResultX)-1):
                PackCoordX[X] = (ResultX[X+1] - ResultX[X])/2  + ResultX[X]
            for Y in range(len(ResultY)-1):
                PackCoordY[Y] = (ResultY[Y+1] - ResultY[Y])/2  + ResultY[Y]
            # print(PackCoordY)
            # print(PackCoordX)
        PackageColors = []
        
        for X in range(len(ResultX)-1):
            for Y in range(len(ResultY)-1):
                cropimage = bg_removed[round(ResultX[X]):round(ResultX[X+1]),round(ResultY[Y]):round(ResultY[Y+1])]
                V = cropimage.T[0].flatten().mean()
                PackageColors.append(V)
                if V < 180:
                    Packages.append([PackCoordY[Y],PackCoordX[X]])
                    # print(X)
                    cv2.circle(bg_removed, (round(PackCoordY[Y]), round(PackCoordX[X])), 10, (0, 255, 0), -1)
        

    
        dist = depth_frame.get_distance(cx, cy)
        for I in range(len(Packages)):
            Ax = (69.4/bg_removed.shape[1]) * abs((bg_removed.shape[1]/2) - Packages[I][0])
            Ay = (42.5/bg_removed.shape[0]) * abs((bg_removed.shape[0]/2) - Packages[I][1])
            Tx = math.tan(np.deg2rad(Ax)) * dist
            Ty = math.tan(np.deg2rad(Ay)) * dist
            if ((bg_removed.shape[1]/2) - Packages[I][0]) < 0:
                Tx = -Tx
            if ((bg_removed.shape[0]/2) - Packages[I][1]) < 0:
                Ty = -Ty
            Coordinates.append([Tx,Ty])
        print('wait')
        print(Coordinates)
            
  
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.circle(bg_removed,(cx,cy),2,(0,255,0),2)    
        cv2.circle(depth_colormap,(cx,cy),2,(0,255,0),2)    
  
        cv2.namedWindow('Depth_RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.namedWindow('IR_RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('RGB_RealSense', bg_removed)
        cv2.imshow('Depth_RealSense', depth_colormap)
        # # cv2.imshow('IR_RealSense', ir1_image)
        #cv2.imshow('Align Example', images)
        cv2.imshow("gray", imgGray)
        cv2.imshow("blur", imgBlur)
        cv2.imshow("canny", imgCanny)
        cv2.imshow("dialation", imgDial)
        # cv2.imshow("threshold", imgThre)
        
        key = cv2.waitKey(1)

        # else:
        #   print(i)
        dist = depth_frame.get_distance(cx, cy)
        distmm = dist*100
        print(distmm, 'cm')
        # print("Middelpunt:",(60,140))
        #  # time.sleep(0.5)
        #   i = i+1
              
  
        if key & 0xFF == ord('q') or key == 27:
           cv2.destroyAllWindows()
           break 


finally:
         
         pipeline.stop()

