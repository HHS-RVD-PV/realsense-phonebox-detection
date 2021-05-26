# -*- coding: utf-8 -*-
"""
Created on Tue May 25 15:17:57 2021

@author: Mees Wesseling
"""
from bs4 import BeautifulSoup
import requests
import xlsxwriter

phonelist = []

phone = ["apple-iphone-11","apple-iphone-xs"]

for i in range(len(phone)):
    
    url="https://www.dimensions.com/element/" + phone[i]
    
    # Make a GET request to fetch the raw HTML content
    html_content = requests.get(url).text
    
    # Parse the html content
    soup = BeautifulSoup(html_content, "lxml")
    
    width = soup.find(id="width-result").text
    height = soup.find(id="height-result").text
        
    splitheight = height.split();
    splitwidth = width.split();
    
    width = splitwidth[2]
    height = splitheight[2]
    
    phonespecs = [phone[i], width,height]
    phonelist.append(phonespecs)

print(phonelist)


workbook = xlsxwriter.Workbook('phonespecs.xlsx')
worksheet = workbook.add_worksheet()

row = 0

for col, data in enumerate(phonelist):
    worksheet.write_column(row, col, data)

workbook.close()