# -*- coding: utf-8 -*-
"""
Created on Tue May 18 11:39:57 2021

@author: Gebruiker

Depth of package recognition system, because the size of a package is not 
known in all situations.The priciple is to recognize the width first and with
the width we can take a picture with an angle. This gives enough information 
to determine an area for the depth of the package.

The depth view is used to know the distance to the package. The rgb view is 
used for the detection and calculations for the width. Known is the angle of 
view which is horizontal equal to 69 degrees. Using goniometry we can calculate
the cm per pixel, which gives the possibility to calculate the exact width of
a package.

This system is needed to get the first depth midpoint, after this the pickup
point can be controlled by the depth view.
"""
import cv2
import numpy as np
import math

#Function for measuring the distance to the front of a package
def distpack(c1, c2):
    dist = depth_frame.get_distance(c1, c2)
    distmm = dist*10
    return depth

#Function for the goniometry calculating pixelcm
def distpixel(distance):
    viewwidth = math.tan(math.radians(alpha/2))*distance
    pixelmm = viewwidth/rgbwidth
    #print(viewwidth, pixelmm)
    return pixelmm

#Function for calculating the width of the package
def widthpack(pixelmm, x1, x2):
    widthpixel = x2 - x1
    width = pixelmm * widthpixel
    return width

#Function for determining depth with an picture taken under an angle
def center(w, w1, w2, l):
    pixelw1 = w/w1
    pixelw2 = w/w2
    pixell1 = pixelw1 * l
    pixell2 = pixelw2 * l
    geml = (pixell1 + pixell2)/2
    centerg = geml/2
    return centerg

alpha = 69
rgblength = 240
rgbwidth = 320

depthpack = distpack(rgbwidth, rgblength)
pixel = distpixel(depthpack)
point1 = 430
point2 = 600
realwidth = widthpack(pixel, point1, point2)
width1 = 400
width2 = 300
length = 200
centerpack = center(realwidth, width1, width2, length)
pickupdepth = depthpack + centerpack

print(pixel)
print(realwidth)
print(centerpack)
print(pickupdepth)

