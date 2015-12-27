# -*- coding: cp1252 -*-
'''
* Team Id :         2941
* Author List :     K.S.Santosh
* Filename:         TrafficSignals.py
* Theme:            Courier Service
* Functions:        detect_blue_node(img),find_traffic_position_number(img)
* Global Variables: Thickness, Cell_height
'''
import numpy as np
import cv2
##########################################
##Global Constants 
##########################################
Thickness = 16 #Thickness of links
Cell_height = 106 #size of cell

'''
* Function Name: detect_red_green_node
* Input:         img –- Any one of the test images
* Output:        arrays –- number of coloured nodes
* Logic:         Masks the image for blue color and then finds contours for detecting blue nodes 
* Example Call:  a = detect_blue_node(img)
                >>>1
'''
def detect_blue_node(img): ##Helper Function
   
    
    param1 = [110,100,100]
    param2 = [140,255,255]

    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    #converting the parameters into a form that OpenCV can understand
    lower = np.array(param1)   
    upper = np.array(param2)
    #masking to target only coloured nodes
    mask = cv2.inRange(hsv, lower, upper)
    
    img   = cv2.bitwise_and(img, img, mask= mask)
   
    #process to find contours in the masked image
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,1,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    
    temp_contour = []
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        
        if ((area >= 100.0)and(area <= 200)): #selecting the contours formed on coloured nodes
            temp_contour.append(contours[i])
       
    node_contours = temp_contour 
    
   
    node_count =  len(node_contours)
    
    #returns number of coloured nodes
    return node_count

'''
* Function Name:  find_traffic_position_number
* Input:          img – Any one of the test images 
                  color -- 0 for red 1 for green 
* Output:         arrays –- traffic junction numbers(0 to 35)
* Logic:          Clips each node/junction and calls detect_blue_node function. 
                  The absence of blue node gives the position of traffic signal. 
* Example Call:   a = find_traffic_position_number(img)
                  >>> a = [35,30,22,40,43]
'''    
def find_traffic_position_number(img): ##Calling Function
    
    
    Trafficlist = []
    
    x = y = 0
    
    height_start = 0
    height_stop = 0
    
    #clips each row to detect colored node and stores it's row number
    for i in range(7):
        
        height_stop = height_start + Thickness
        
        width_start = 0
        width_stop = 0
        
        for j in range(7):    
            
            width_stop = width_start + Thickness
       
            img_col = img[height_start:height_stop,width_start:width_stop,:]
            width_start = width_start + Cell_height + Thickness
            
            blunode = detect_blue_node(img_col)
            if (blunode == 0):
                #stores column where the coloured node is detected
                x = j
                y = i
                node_number = (y * 7) + x
                Trafficlist.append(node_number)
                    
                      
        height_start = height_start + Cell_height + Thickness
        
    Trafficlist.remove(42)
    #returns the node number
    return Trafficlist