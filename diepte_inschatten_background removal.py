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
def nothing(x):
    pass
a = 0
k = 0
i=0
pipeline=rs.pipeline()
config=rs.config()
cx =0
cy= 0
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 15)
rs.config.enable_device_from_file(config, "/home/wouter/Downloads/object_detection1.bag")
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
clipping_distance_in_meters = 0.8 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)
cv2.namedWindow("canny",cv2.WINDOW_AUTOSIZE)
cv2.namedWindow('RGB_RealSense', cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar("lower", "canny", 50, 1000, nothing)
cv2.createTrackbar("upper", "canny", 250, 1000, nothing)
#cv2.createTrackbar("blur", "canny", 5, 1000, nothing)
cv2.createTrackbar("brightness", "RGB_RealSense", 550, 1000, nothing)

#eta = 0
try:
    while True:
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
          grey_color = 0
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
          
          #contouren zoeken
          # _, contours, hiearchy = cv2.findContours(imgThre,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
          # for contour in contours: 
          #     opp = cv2.contourArea(contour) #oppervlakte pixels
          #     approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True) #voor niet perfecte rechhoek correctie
          #     lengte = len(approx) #aantal lijnen voor gesloten contour
          #     #print(lengte)
          #     #print("Opp:", opp)
    
          #     if opp < 200000  and opp > 10000 and lengte == 4:
          #         M = cv2.moments(contour) # met momenten het middelpunt bepalen
          #         cx = int(M["m10"] / M["m00"])
          #         cy = int(M["m01"] / M["m00"])
          #         cv2.circle(color_image, (cx,cy), (5), (99, 115, 0),5) #tekenen middelpunt
          #         cv2.drawContours(color_image, [approx], 0, (99, 115, 0), 2) #tekenen contouren van bijbehorende punten
          #        # print("Opp:", opp)
          #         #print("Middelpunt:",(cx,cy))
          #         while a < lengte: #hoekpunten tekenen
          #             cv2.circle(color_image, ((approx[a][0][0]),(approx[a][0][1])), (5), (99, 115, 0),5)
          #             a = a + 1;
          
          # lines= cv2.HoughLines(imgCanny, 1, np.pi/180.0, 120, np.array([]))
          # for line in lines:
          #     rho, theta = line[0]
          #     a = np.cos(theta)
          #     b = np.sin(theta)
          #     x0 = a*rho
          #     y0 = b*rho
          #     x1 = int(x0 + 1000*(-b))
          #     y1 = int(y0 + 1000*(a))
          #     x2 = int(x0 - 1000*(-b))
          #     y2 = int(y0 - 1000*(a))
 
          #     cv2.line(bg_removed,(x1,y1),(x2,y2),(0,0,255),2)
         
          lines= cv2.HoughLinesP(image = imgDial, rho = 1, theta=np.pi/180.0, threshold =100, lines=np.array([]),minLineLength=100, maxLineGap=10)
          for line in lines:
              # x1,y1,x2,y2 = line[0]
 
              # cv2.line(bg_removed,(x1,y1),(x2,y2),(0,0,255),2)
              a,b,c = lines.shape
              for i in range(a):
                  cv2.line(bg_removed, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 1, cv2.LINE_AA)
          
          depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

          cv2.namedWindow('Depth_RealSense', cv2.WINDOW_AUTOSIZE)
          # cv2.namedWindow('IR_RealSense', cv2.WINDOW_AUTOSIZE)
          #cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
          cv2.imshow('RGB_RealSense', bg_removed)
          cv2.imshow('Depth_RealSense', depth_colormap)
          # cv2.imshow('IR_RealSense', ir1_image)
          #cv2.imshow('Align Example', images)
          cv2.imshow("gray", imgGray)
          cv2.imshow("blur", imgBlur)
          cv2.imshow("canny", imgCanny)
          cv2.imshow("dialation", imgDial)
          # cv2.imshow("threshold", imgThre)
          
          key = cv2.waitKey(1)
          if (cx==0 & cy==0):
               print("noodplan")
               cx=320 
               cy=240
               dist = depth_frame.get_distance(320, 240)
          # else:
              
          #   print(i)
          #   dist = depth_frame.get_distance(cx, cy)
          #   distmm = dist*100
          #   print(distmm, 'cm')
          #   print("Middelpunt:",(cx,cy))
          #  # time.sleep(0.5)
          #   i = i+1
                

          if key & 0xFF == ord('q') or key == 27:
             cv2.destroyAllWindows()
             break 
                      


finally:

         pipeline.stop()

