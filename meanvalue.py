#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 25 13:18:30 2021

@author: wouter
"""


import cv2
crop_imgP1 = imgP1[y0:y1,x0:x1]
crop_imgP1 = cv2.cvtColor (crop_imgP1, cv2.COLOR_BGR2HSV)
v = crop_imgP1.T[2].flatten().mean()
print ("Value:% .2f"% (v))                   