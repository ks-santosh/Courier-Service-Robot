# -*- coding: cp1252 -*-
'''
* Team Id :         2941
* Author List :     K.S.Santosh
* Filename:         GraphOfGrid.py
* Theme:            Courier Service
* Functions:        find_link(img),grid_to_arrays(img), make_graph(img) 
* Global Variables: Thickness, Cell_height
'''
import numpy as np
import cv2

##########################################
##Global
##########################################
Thickness = 16 #Thickness of links
Cell_height = 106 #size of cell
#####################################################################
##ADJACENCY LIST OF THE GRID IN IMAGE
#####################################################################
###########################################
##Grid to Graph
###########################################
'''
* Function Name: find_link
* Input:         img – Any one of the test images 
* Output:        arrays –- a binary matrix(list of list) containing '1' for existing Horizontal Links
* Logic:         draws a white circle on the links so that circle is visible only on the existing links. 
                 The circles are then contoured. The centroid of the circles give the node number. 
                 Absence of a circle implies absence of a link.    
* Example Call:  a = find_link(img)
                 >>>  [[1,1,1,1,1,1],[0,1,0,1,1,1],[1,1,1,1,1,1,]]
'''
def find_link(img): ##Helper Function
    
    
    #cloning the image for later use (finding vertical links)
    #reference: http://stackoverflow.com/questions/16533078/clone-an-image-in-cv2-python
    image = cv2.copyMakeBorder(img,0,0,0,0,cv2.BORDER_CONSTANT)
    
    #draws white circle on all the links
    #the circle will only be visible where links(black) exists
    for j in range(7): #six vertical links  
        
        y = (Thickness * j) + (Cell_height * j) + Thickness/2
        
        for i in range(6): #five horizontal links
            
            x = (Thickness * (i + 1)) + (Cell_height * i) + Cell_height/2
            cv2.circle(image,(x,y), 5, (255,255,255), -1)
    
    #creates contours 
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,70,254,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    
    #Filtering contours and storing those contours that are formed around the circle 
    #drawn on the links
    temp_contour = []
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
   
        if area == 66.0: #area of circle drawn
            temp_contour.append(contours[i])
            
    #contours on circle 
    link_contours = temp_contour 
    
    links = []
    
    for k in range(len(link_contours)):
        #calculating centre of each circle
        M = cv2.moments(link_contours[k])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        #calculating the circle row number and column number
        ny = (cy - (Thickness/2))/(Thickness + Cell_height)
        nx = (cx - (Cell_height/2) - Thickness)/(Thickness + Cell_height) 
        
        #calculates the circle number
        #which gives the link number
        cell_num = (ny*6) + 1 + nx
        
        links.append(cell_num) #stores the existing links
    
    links = sorted(links)
    
    row =[] #links in a row 
    link_list = [] #stores rows of links
    
    for n in range(42): #total number of links
        #appends 1 if link exists
        if ((n+1) in links):
            row.append(1)
        #else link doesn't exist
        else:
            row.append(0)
        
        #if links of one row is completed add it to the link_list
        #and start a new list
        if(len(row) == 6):
            link_list.append(row)
            row = []
    
    #returns list of horizontal links
    return link_list

'''
* Function Name: grid_to_arrays
* Input: img – Any one of the test images 
* Output: arrays –- a list containing the Horizontal Links array and the Vertical Links array
* Example Call: a, b = grid_to_arrays(img)
                >>> a = horizontal_links, b = vertical_links
'''
def grid_to_arrays(img): ##Helper Function
    
   
    #add your code here
   
    horizontal_links = find_link(img) #finds horizontal links
    
    #for vertical links we flip the image and rotate it 
    #then pass the image to the find_link function which then gives vertical links
    rows,cols, color = img.shape
    
    # reference: http://enthusiaststudent.blogspot.in/2015/01/horizontal-and-vertical-flip-using.html
    img=cv2.flip(img,1) #flips around y axis
    
    # reference :
    # http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html
    # rotates image 90 degree
    M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
    dst = cv2.warpAffine(img,M,(cols,rows))
    
    Vlinks = find_link(dst)#finds vertical links
    
    #reference : http://stackoverflow.com/questions/17037566/transpose-a-matrix-in-python
    vertical_links = map(list, zip(*Vlinks))#transpose the matrix for proper format
    
    return horizontal_links, vertical_links 

'''
* Function Name: make_graph
* Input:         img – Any one of the test images 
* Output:       (adjacency list)dictionary containing node numbers as keys and 
                 their connected neighbours as values  
* Logic:        From the find_link function the matrix returned is sliced for every 2 digits. 
                The two digits tells whether the links are connected or not.         
* Example Call: graph = make_graph(img)
                >>> graph = {0: [1],
                             1: [0],
                             2: [3],
                             3: [2,4],
                             4: [3,5],
                             5: [4]}
'''    
def make_graph(img): ##Calling Function
    
   
    #forms graph using horizontal and vertical link lists 
    hlist, vlist = grid_to_arrays(img)
    
    #transform the horizontal_links from a list of lists to a single list
    Hlist = []
    for i in range(7):
        #appending zero before each row(first nodes are not connected to left)
        Hlist.append(0)
        for j in range(6):
            temp = hlist[i][j]
            Hlist.append(temp)
            
    #last node is not connected to left
    Hlist.append(0) #Hlist completed 
    
    Hgraph = {}
    for n in range(49): #loop checking all nodes and their horizontal neighbours
        
        key = n
        value = []
        
        slice = Hlist[n:n+2]
        
        #if node connected to left
        if(slice[0] == 1):
            value.append(n-1)
        
        #if node connected to right
        if(slice[1] == 1):
            value.append(n+1)
        
        Hgraph[key] = value #new entry in dictionary
            
    #reference : http://stackoverflow.com/questions/17037566/transpose-a-matrix-in-python
    #transposing the vertical_links matrix
    vlist = map(list, zip(*vlist))
    
    Vlist = []
    for i in range(7): #six vertical links in a row
        
        Vlist.append(0) #node not connected to top
        
        for j in range(6): #five vertical links in a column
            #converts matrix into a list
            temp = vlist[i][j]
            Vlist.append(temp)
            
    Vlist.append(0) #Vlist completed 
    
    Vgraph = {}
    count = 0
    
    #loop checking all nodes and their vertical neighbours
    for n in range(7):
        
        for c in range(7):
        
            key = n + (c * 7)
            
            value = []
            slice = Vlist[count:count+2]
            
            #if node connected to top node
            #add the top node number
            if(slice[0] == 1):
                value.append(key - 7)
            
            #if node connected to bottom node
            #add the bottom node number
            if(slice[1] == 1):
                value.append(key + 7)

            Vgraph[key] = value
            count = count + 1  

    Graph = {} 
    
    #merging the horizontal and vertical neighbours
    for i in range(49):
            
        key = i
            
        value = Vgraph[key] + Hgraph[key]
        
        Graph[key] = value
    
    #returns adjacency list of the graph(grids in image)
    return Graph    
