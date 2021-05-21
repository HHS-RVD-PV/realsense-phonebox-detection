# -*- coding: utf-8 -*-
"""
Created on Tue May 18 16:05:48 2021

@author: ljjmj
"""

import numpy as np


MedianX = []
ListX = [1,2,3,4,5,5.2,5.3,5.5,6.4,6,3,6.2,7,8,8.1,8.3,8.4,20,25]
ResultX = []
ResetX = 0
MedianY = []
ListY = [2.3,2.5,2.8,2.9,3.1,8.2,9.3,9.4,9.4,9.5,20.2,20.5,20.6,20.7,20.7]
ResultY = []
ResetY = 0
Packages = []


while (len(ListX) >= 3) and (ResetX != 2):          # 3 = aantal keer dat de eigenschap terugkomt
    for T in range(len(ListX)):
        if ListX[T] <= (ListX[0] + 3):              # threshold waarbinnen de feature moet vallen
            MedianX.append(ListX[T])
        else:
            if (len(MedianX)) >= 3:                # 3 = aantal keer dat de eigenschap terugkomt
                ResultX.append(np.median(MedianX))
            for L in range(len(MedianX)):
                ListX.pop(0)
            MedianX = []
            ResetX = 1
        if len(MedianX) > (len(ListX)-1):
            ResetX = 2                              # verlaat de forloop als er geen waardes meer in de list zijn die een lijn vormen
            ResultX.append(np.median(MedianX))
            break
        if ResetX == 1:                             # reset de for loop start weer bij 0
            ResetX = 0
            break
print(ResultX)


while (len(ListY) >= 3) and (ResetY != 2):          # 3 = aantal keer dat de eigenschap terugkomt
    for T in range(len(ListY)):
        if ListY[T] <= (ListY[0] + 3):              # threshold waarbinnen de feature moet vallen
            MedianY.append(ListY[T])
        else:
            if (len(MedianY)) >= 3:                # 3 = aantal keer dat de eigenschap terugkomt
                ResultY.append(np.median(MedianY))
            for L in range(len(MedianY)):
                ListY.pop(0)
            MedianY = []
            ResetY = 1
        if len(MedianY) > (len(ListY)-1):
            ResetY = 2
            ResultY.append(np.median(MedianY))
            break
        if ResetY == 1:
            ResetY = 0
            break
print(ResultY)

PackCoordX = np.zeros((len(ResultX)-1))
PackCoordY = np.zeros((len(ResultY)-1))

if (len(ResultX) > 1) and (len(ResultY) > 1):
    for X in range(len(ResultX)-1):
        PackCoordX[X] = (ResultX[X+1] - ResultX[X])/2  + ResultX[X]
    for Y in range(len(ResultY)-1):
        PackCoordY[Y] = (ResultY[Y+1] - ResultY[Y])/2  + ResultY[Y]

for X in range(len(ResultX)-1):
    for Y in range(len(ResultY)-1):
        Packages.append([PackCoordX[X],PackCoordY[Y]])
        cv2.circle(img, (PackCoordX[X], PackCoordY[Y]), 3, (0, 0, 0), -1)
    
    
