# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 10:56:52 2021

@author: Gebruiker

Combining vision detection 
"""
import pandas as pd
import numpy as np

#This function calculates the depth of a package using phone and tablet specs.
def get_package_depth(width):
    data = pd.read_excel(r'C:\Users\Gebruiker\Documents\PV\diepte_data\phonespecs.xlsx')
    data = data.to_numpy()
    
    datalist = []
    x = int(width)
    datarange = 15 #range to select which data will be used for the average
    
    for i in range(len(data[0,:])):
        if(x-datarange < data[0,i]):
            if(x+datarange > data[0,i]):
                if(data[1,i] > data[0,i]): #check if depth is bigger than width
                    datalist.append(data[1,i])
    
    average = np.mean(datalist)
    centerdepth = average*0.5
    print("")
    print("Average depth in mm:", average)
    print("Center depth in mm:", centerdepth)
    
    return centerdepth

def depth_control(pixelx, pixely, depth_old):
    depth_new = 600
    package_depth_new = depth_new - depth_old
    new_center_depth = package_depth_new*0.5
    
    return new_center_depth

package_width_mm = 91 #width detection
depth_package = get_package_depth(package_width_mm)
print("Depth coordinate in mm:", depth_package)
depth_package_control = depth_control(100, 100, 400)
print("Depth control gives new coordinate in mm:", depth_package_control)
