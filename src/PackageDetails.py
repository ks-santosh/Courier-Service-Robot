'''
* Team Id :         2941
* Author List :     K.S.Santosh
* Filename:         PackageDetails.py 
* Theme:            Courier Service
* Functions:        detect_color(img,color), package_shape_color(img), warehouse_package(img), delivery_cells(img), delivery_junction(img),
                    pickup_deliver_junctions(img)
* Global Variables: Colors, Thickness, Cell_height
'''
import numpy as np
import cv2
##########################################
##Global 
##########################################
Colors = ["Green","Skyblue","Orange","Pink"] #color of packages
Thickness = 16 #Thickness of links
Cell_height = 106 #size of cell

'''
* Function Name: detect_color
* Input:         img -- a test image 
                 color -- name of color to be masked
* Output:        A color masked image
* Logic:         Masking is performed by giving the lower and upper parameters of different colors to be masked and
                 using inRange and bitwise_and functions 
* Example Call:  detect_color(img, "Green")                 
'''
def detect_color(img,color): ##Helper Function  
    
    if(color == "Green"):
        param1 = [45,100,100]
        param2 = [75,255,255]
    
    elif(color == "Skyblue"):
        param1 = [90,100,100]
        param2 = [110,255,255]
    
    elif(color == "Orange"):
        param1 = [14,0,0]
        param2 = [16,255,255]
    
    elif(color == "Pink"):
        param1 = [130,100,100]
        param2 = [150,255,255]
    
    
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)    
   
    lower = np.array(param1)   
    upper = np.array(param2)
    
    mask = cv2.inRange(hsv, lower, upper)
    
    masked_img   = cv2.bitwise_and(img, img, mask= mask)
   
    return masked_img

'''
* Function Name: package_shape_color
* Input:         img -- a test image
* Output:        returns shape and color of all the packages in the img
* Logic :        Finds color using detect_color function, makes contours on the image returned, 
                 finds the shape of the contour according to which the shape is determined 
* Example Call:  package_shape_color(img)

                >>> [["Square","Green"],["Triangle","Pink"]]
'''    
def package_shape_color(img): ##Helper Function
   
    
    PackageShapeColor = []
    Shape = ""
    for Color in Colors:
        
        color_image = detect_color(img, Color) 
        gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,1,220,0)
        shape_contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        #for each cell find the color and the shape
        for i in range(len(shape_contours)):
            area = cv2.contourArea(shape_contours[i])
            
            if (area >= 400 )and(area < 700):
                Shape = "Triangle"
                PackageShapeColor.append([Shape,Color])
            
            elif(area >= 700 )and(area < 900):
                Shape = "Circle"
                PackageShapeColor.append([Shape,Color])
            
            elif(area >= 900 )and(area < 1100):
                Shape = "Square"
                PackageShapeColor.append([Shape,Color])             
    
    return PackageShapeColor       

'''
* Function Name: warehouse_package
* Input:         img -- a test image
* Output:        Returns a list of active companies along with the 
                 color and the shape of the packages it contains
* Logic:         Crops the region of each companies and finds the color and shape of the package it 
                 includes using package_shape_color function  
* Example Call: warehouse_package(img)

                >>> [[0,[[["Square","Green"],["Triangle","Pink"]]]], [1,[[["Circle","Skyblue"],["Triangle","Pink"]]]]]
'''
def warehouse_package(img): ##Helper Function
    
    #stores the warehouse details
    warehouse = []
    
    width_start = Thickness
    width_stop = 0
    height_start = Thickness
    height_stop = Cell_height + Thickness
    
    #crop each company of the warehouse and find the package details
    for i in range(6):
                   
        width_stop = width_start + Cell_height
            
        company = img[height_start:height_stop,width_start:width_stop,:]
        ShapeColor = package_shape_color(company)
        
        if(ShapeColor):
            warehouse.append([i,ShapeColor])

        width_start = width_stop + Thickness                      
        
    return warehouse
    

'''
* Function Name: delivery_cells
* Input:         img --  a test image
* Output:        returns the the cell number and its part number which contains 
                 packages along with their shape and color
* Logic:         Each cell is cropped which is further cropped in 4 equal parts. These parts are searched 
                 for the packages using package_shape_color function.  
* Example Call: delivery_cells(img)

                >>> [[29,0,["Square","Green"]],[35,3,["Triangle","Orange"]]]
'''
def delivery_cells(img): ##Helper Function
       
    Deliverylist = []
    #the number of cell after which the city starts(starts from 0)
    cell_count = 5
    
    height_start = Thickness + Cell_height + Thickness    
    
    for i in range(5):
        
        height_stop = height_start + Cell_height
        width_start = Thickness
        
        for j in range(6):    
            
            width_stop = width_start + Cell_height
            
            ##each cell cropped    
            img_cell = img[height_start:height_stop,width_start:width_stop,:]
            cell_count = cell_count + 1
            
            #found packages in the cell
            package = package_shape_color(img_cell)
            
            if(package):
               
                packs = 0
                parts = 0
                
                #loop runs till the position of all the packages detected in the cell is not found
                while packs < len(package) :
                    
                    HStart = 0
                    HStop = 0
                    #Splits each cell in 4 parts
                    for r in range(2):
                        
                        WStart = 0 
                        HStop = HStart + Cell_height/2
                    
                        for c in range(2):
                            
                            WStop = WStart + Cell_height/2
                            img_cell_part = img_cell[HStart:HStop,WStart:WStop:]
                            parts = parts + 1
                            deliverypack = package_shape_color(img_cell_part)
                            
                            if(deliverypack):
                                
                                packs = packs + 1
                                Deliverylist.append([cell_count,parts,deliverypack])
                            
                            WStart = WStop 
                        
                        HStart = HStop
                        
                        
            
            
            width_start = width_stop + Thickness                    
                      
        height_start = height_stop + Thickness
       
    return Deliverylist

'''
* Function Name: delivery_junction
* Input:         img -- a test image
* Output:        Returns the delivery junction and the shape and color of packages
* Logic:         Converts the part number and the cell number returned by delivery() function to the 
                 corresponding junction number 
* Example Call:  delivery_junction(img)
                >>> [[32,["Square","Green"]],[47,["Triangle","Orange"]]]
'''    
def delivery_junction(img): ##Helper Function
        
    Deliverylist = delivery_cells(img)
        
    DeliveryJunction = []
    position = [0,1,7,8]
    add = 0
        
    for chunks in Deliverylist :
        cell_count = chunks[0]
        packs = chunks[1]
        shapecolor = chunks[2][0]
        #calculates the junction number according to the part number and cell number    
        for i in range(6):
            c = i + 1
            if(cell_count in range(c*6, c*6 + 6)):
                add = c
            
        DeliveryJunction.append([cell_count + add + position[packs-1], shapecolor[0],shapecolor[1]])
        
    return DeliveryJunction

'''
* Function Name: pickup_deliver_junctions
* Input:         img -- a test image
* Output:        returns the PUJ along with its package color and shape and the delivery junction of each package
* Logic:         includes the package details, their pick-up junctions and the delivery junctions got 
                 from the warehouse_package and delivery_junction functions, in a list   
* Example Call:  pickup_deliver_junctions(img)

                >>> [[0,[[["Square","Green",29],["Triangle","Pink",48]]]], [1,[[["Circle","Skyblue",35],["Triangle","Pink",36]]]]]
'''
def pickup_deliver_junctions(img): ##Calling Function
    
    pickups = warehouse_package(img)
    deliver = delivery_junction(img)
    
    for picks in pickups :
        
        for shapecolor in picks[1] :
            #appends the appropriate delivery junctions in the pick-up list
            for packs in deliver :
                if((shapecolor[0] == packs[1])and(shapecolor[1] == packs[2])):
                    shapecolor.append(packs[0])
                    
    return pickups