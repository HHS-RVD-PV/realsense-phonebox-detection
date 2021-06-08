# -*- coding: utf-8 -*-
"""
Created on Thu May 27 12:54:23 2021

@author: Mees Wesseling
"""

import pandas as pd
import numpy as np

df = pd.read_excel(r'C:\Users\Gebruiker\Documents\PV\diepte_data\phonespecs.xlsx')

#df = pd.read_excel (r'Path where the Excel file is stored\File name.xlsx', sheet_name='your Excel sheet name')

df = df.to_numpy()

lijst = []

x = input("Enter an integer >>>")
x = int(x)

for i in range(len(df[0,:])):
    if(x-15 < df[0,i]):
        if(x+15 > df[0,i]):
            if(df[1,i] > df[0,i]): #Als diepte groter is dan breedte mag die worden toegevoegd
                lijst.append(df[1,i])

print(lijst)
gemiddelde = np.mean(lijst)

print(gemiddelde)
    


            
            
        
        
    
