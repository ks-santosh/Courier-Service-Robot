# -*- coding: cp1252 -*-
'''
* Team Id :         2941
* Author List :     K.S.Santosh
* Filename:         PathTraversal.py
* Theme:            Courier Service
* Functions:        move_robot(Path, CheckSignal, ASignals = []), BreadthFirstSearch(graph,start,end,q,XSignals,ASignals,shortestlength),
                    shortest_path(graph, origin, destination), dijkstra(graph, initial),
                    path_planner(Start, End, TrafficPosition, Unweighted, AdjacencyList)
* Global Variables: Signal, Orient
'''
import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
from collections import defaultdict, deque

##########################################
##Global
##########################################
#Stores the status of 5 signals
# X - Unknown Signal
# N - No Signal
# A - Active Signal
Signal = ['X','X','X','X','X'] 
#Orientation of the robot with respect to the initial direction
#F for front, B for back, L for left and R for Right 
Orient = 'F' 
##For the straight path of the first path of the robot from the green node
##Line won't be followed as the robot goes straight(but doesn't turn perfectly)
First_Path = True

##For capturing frames
capture = cv2.VideoCapture(0)
##########################################
##GPIO settings
##########################################
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(24,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)

PWMR = GPIO.PWM(24,60)
PWMR1 = GPIO.PWM(23,60)
PWML1 = GPIO.PWM(27,60)
PWML = GPIO.PWM(22,60)

'''
    * Function Name: blue_node_detect
    * Input:         img - the image caught by the camera
    *                red_detect - Tells the function whether to search for red node(traffic signal status)
    * Output:        Returns "Blue" if blue node is detected else "NoBlue"
                     Returns "Red" if red node is detected else "NoRed"
    * Logic:         All the ranges of blue and red node is provided for masking. It is then contoured.
    *                The area found gives the existence of the blue/red node
    * Example Call:  blue_node_detect
'''
def blue_node_detect(img,red_detect = False):
   
    
    #Different ranges for capturing all shades of blue
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    param1 = [178,240,0]
    param2 = [178,240,240]
    lower = np.array(param1)   
    upper = np.array(param2)
    mask = cv2.inRange(hsv, lower, upper)
    
    lower = np.array([100,100,100])   
    upper = np.array([255,255,255])
    
    mask2 = cv2.inRange(hsv, lower, upper)  
    
    #for dark red
    paramred1 = [0,100,100]
    paramred2 = [10,255,255]
    lowerred = np.array(paramred1)   
    upperred = np.array(paramred2)
    
    maskred1 = cv2.inRange(hsv, lowerred, upperred)

    #for light red
    paramred1 = [160,100,100]
    paramred2 = [179,255,255]

    lowerred = np.array(paramred1)   
    upperred = np.array(paramred2)
    
    maskred2 = cv2.inRange(hsv, lowerred, upperred)
    
    if(red_detect):
        img2   = cv2.bitwise_and(img, img, mask = maskred1 + maskred2)
        gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,1,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        track_length = len(contours)
        if(track_length > 1):
                
            maxarea = 0
            track_contour = [0]
            for i in range(track_length):
                    
                area = cv2.contourArea(contours[i])
                #print area
                if area >= maxarea:
                    track_contour[0] = contours[i]
                 
                    maxarea = area
               
            area = cv2.contourArea(track_contour[0])
            
            #blue node detected
            if(area > 7000):
                
                return "Red"
            
        else:
            #no color detected
            return "NoRed"
    
    else:
        img2   = cv2.bitwise_and(img, img, mask = mask + mask2)
        gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,1,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        track_length = len(contours)
        if(track_length > 1):
                
            maxarea = 0
            track_contour = [0]
            for i in range(track_length):
                    
                area = cv2.contourArea(contours[i])
                #print area
                if area >= maxarea:
                    track_contour[0] = contours[i]
                 
                    maxarea = area
               
            area = cv2.contourArea(track_contour[0])
            
            #blue node detected
            if(area > 7000):
                
                return "Blue"
                
        else:
            #no color detected
            return "NoBlue"
    
    return 0 
'''
* Function Name: move_forward
* Input:         CheckSignal - Command specifying whether to check status of traffic signal during traversing path or not
* Output:        Moves the robot by line following and returns the status of Traffic Signal 
* Logic:         After masking and dilation the the black path is contoured and its centroid is determined. 
                 The robot follows in such a way that the centroid remains on the middle of the image. Blue node is continuously detected.
                 If at any blue node is undetected, that means a link is covered and the function returns.
* Example Call:  move_forward(True)
'''
def move_forward(CheckSignal = False):
    
    Signal_Status = False
        
    previous_error = 0
    count = 0
    node_detected = 0
    one_link_travelled = 0
    
    while cv2.waitKey(1) != 27 :

        
        
        flag, img = capture.read()
        
        ##the lowest image cropped for processing the line strip near the robot 
        track_crop = img[50:480,100:540,:]
        img = track_crop
        
        ##Detect red node if CheckSignal is True
        red_node_detected = False
        if(CheckSignal):
            red_node_detected = blue_node_detect(img,True)
        
        ##Blue node should always be detected as the red node may not exist
        ##and robot won't be able to know whether it has completed one link or not
        blue_node_detect = blue_node_detect(img,False)
        
        if(red_node_detected == "Red"):
            one_link_travelled = 1
            Signal_Status = True
            
        elif(blue_node_detected == "Blue"):
            one_link_travelled = 1
            Signal_Status = False
            
        ##if after detecting a node further no node is detected, that means one link is completed
        if((red_node_detected != "Red")and(blue_node_detected != "Blue"))and(one_link_travelled == 1):    
            ##Go forward for a small amount of time and then return the status of signal
            PWMR.start(100)
            PWML.start(100)
            time.sleep(0.8)
            PWMR.stop()
            PWML.stop()
            return Signal_Status
                
        ##range of detecting all shades of black    
        param1 = [ 0,0,0]
        param2 = [140,140,140]
    
      
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array(param1)   
        upper = np.array(param2)
        mask = cv2.inRange(hsv, lower, upper) 
        img = mask   
      
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
      
        ##Dilated to remove disturbances in image caused by shadow or dust   
        dilation = cv2.dilate(img, kernel, iterations = 3)
        
        img = dilation
        ret,thresh = cv2.threshold(img,1,254,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        if(not contours):
            continue
        
        #choosing the largest contour
        track_length = len(contours)
        if(track_length > 1):
            
            maxarea = 0
            track_contour = []
            for i in range(track_length):
                
                area = cv2.contourArea(contours[i])

                if area > maxarea:
                    track_contour = contours[i]
                    maxarea = area
            tp = cv2.contourArea(track_contour)
           
            contours[0] = track_contour
        
        M = cv2.moments(contours[0])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        ##mid point of the image captured 
        correct_spot = 220
        
        ##gives the amount of displacement with respect to the link
        error = cx - correct_spot
     
        
        ##Since the error values change frequently even though the robot is still 
        ##we will move the robot only when the error is approximately same for 4 times
        if(abs(error - perror) < 10):
            count = count+1 
        else:
            count = 0
        
        previous_error = error 
        
        if(count < 4):
            continue
                
        ##For starting count from one and moving the robot according to the error calculated
        count = 0
                     
        if(error < -35): #move left
            PWMR.start(100)
            PWML.start(0)  
            time.sleep(0.2)
            PWML.ChangeDutyCycle(100)
            time.sleep(0.2)
           
        
        elif(error > 35): #move right
            PWMR.start(0)
            PWML.start(100)  
            time.sleep(0.2)
            PWMR.ChangeDutyCycle(100)
            time.sleep(0.2)
            
        else: ##go straight
            PWMR.start(100)
            PWML.start(100)  
            time.sleep(1)
            
        ##stop all the motors    
        PWMR1.stop()
        PWML1.stop()
        PWMR.stop()
        PWML.stop()    
        
        
    PWMR.stop()
    PWMR1.stop()
    PWML.stop()
    PWML1.stop()
        
    return 0      
'''
* Function Name: turn_hard_right
* Input:         none
* Output:        Moves the robot to the right
* Logic:         Right motor is turned backwards and left motor is turned forward
* Example Call:  turn_hard_right()
'''    
def turn_hard_right():
    
    PWMR1.start(100)
    PWML.start(100)
    time.sleep(0.8)
    PWMR1.stop()
    PWML.stop()
    return
'''
* Function Name: turn_hard_left
* Input:         none
* Output:        Moves the robot to the left
* Logic:         Right motor is turned forward and left motor is turned backwards
* Example Call:  turn_hard_left()
'''
def turn_hard_left():
    PWMR.start(100)
    PWML1.start(100)
    time.sleep(0.8)
    PWMR.stop()
    PWML1.stop()
    return
'''
* Function Name: turn_backward
* Input:         none
* Output:        Moves the robot to backwards
* Logic:         turn_hard_right is called 2 times
* Example Call:  turn_backward()
'''
def turn_backward():
    
    turn_hard_right()
    turn_hard_right()
    return
    
'''
* Function Name: move_robot
* Input:         Path - A list of nodes to traverse
                 CheckSignal - Command specifying whether to check status of traffic signal during traversing path or not
                 ASignals - List of active signals
                 XSignals - List of unknown signals
* Output:        
* Logic:         The move_robot calls appropriate functions to move the robot according to the path passed as an input. 
                 Also updates the status of traffic signal if found active in the global list : signal
* Example Call:  move_robot(Path, True, ASignals,XSignals)
'''    
def move_robot(Path,TrafficPosition,CheckSignal, ASignals = [], XSignals = []): 
    
    Distance = len(Path)
    #for updating the global variable
    global Orient
    global First_Path
    
    Distance = Distance - 1
    Signal_status = False    
    
    link = 0
    ##For the first straight no image is processed as the robot moves straight
    ##But since the turning is not perfect program later uses line following by image processing
    if(First_Path):    
        Xcommon = set(XSignals).intersection(Path)
        Pos = -1
        if(Xcommon):
            TrafficNode = list(Xcommon)[0]
            Pos = TrafficPosition.index(TrafficNode) ##position of traffic light
        First_Path = False
        ##covering a short distance as the robot is kept behind the green node
        PWMR.start(100)
        PWML.start(100)
        time.sleep(1)
        PWMR.stop()
        PWML.stop()
        ##loop runs till the path is straight before any potential traffic signal
        while((Path[link + 1] == Path[link] + 7)and(link + 1 <= Distance)and(link != Pos )):        
            
            PWMR.start(100)
            PWML.start(100)
            time.sleep(3) ##takes 3 seconds to complete one link
            PWMR.stop()
            PWML.stop()
            link = link + 1
            
    ##only cover the left over path
    Distance = Distance - link 
    
    for links in range(Distance):
        
        ##start after the number of links that are covered
        ##start after the number of links that are covered
        ##only effects when the First_Path equals True
        if(links == 0):
            links = link
            
        NextNode = Path[links + 1]
        Node = Path[links]
        #Checks the directions it is facing and plans the path accordingly    
        if(NextNode == Node + 1): ##For Right
            
            if(Orient == 'L'):
                turn_backward()
            
            elif(Orient == 'F'):
                turn_hard_right()
            
            elif(Orient == 'B'):
                turn_hard_left()
                
            Orient = 'R'
            
        elif(NextNode == Node - 1): ##For Left
            
            if(Orient == 'R'):
                turn_backward()
            
            elif(Orient == 'F'):
                turn_hard_left()
            
            elif(Orient == 'B'):
                turn_hard_right()
                
            Orient = 'L'
            
        elif(NextNode == Node + 7): ##For Backward
            
            if(Orient == 'R'):
                turn_hard_right()
            
            elif(Orient == 'L'):
                turn_hard_left()
            
            elif(Orient == 'F'):
                turn_backward()
                
            Orient = 'B'
            
        elif(NextNode == Node - 7): ##For Forward
            ##If this is the first path
                
            if(Orient == 'R'):
                turn_hard_left()
            
            elif(Orient == 'L'):
                turn_hard_right()
            
            elif(Orient == 'B'):
                turn_backward()
                
            Orient = 'F'
        
        if((link == Distance)and(CheckSignal)):
            continue
            
        if(NextNode in XSignals):
            Signal_status = move_forward(True)
            
            Xcommon = set(XSignals).intersection(Path)
            TrafficNode = list(Xcommon)[0]
            Pos = TrafficPosition.index(TrafficNode)
            if(Signal_status == True):    
                Signal[Pos] = 'A'
                time.sleep(30) ##Waiting for 30 seconds over the active signal 
            else:
                Signal[Pos] = 'N'
           
        else:
            move_forward(False)
            
        if(NextNode in ASignals): ##Sleep for 30 seconds over the Active Traffic Signal
            time.sleep(30)
        
    return

class MyQUEUE: # just an implementation of a queue
    
    def __init__(self):
        self.holder = []
        
    def enqueue(self,val):
        self.holder.append(val)
        
    def dequeue(self):
        val = None
        try:
            val = self.holder[0]
            if len(self.holder) == 1:
                self.holder = []
            else:
                self.holder = self.holder[1:]    
        except:
            pass
            
        return val    
        
    def IsEmpty(self):
        result = False
        if len(self.holder) == 0:
            result = True
        return result
'''
* Function Name: BreadthFirstSearch
* Input:         graph - adjacency list of the grid_to_arrays
                 start - start junction
                 end - destination junction
                 q - an object of Queue
                 traffic - list of traffic signal positions
                 shortestlength - the length of the shortest path
* Output: list - out of multiple shortest paths it returns the path that pass thorough minimum number of traffic signals
* Example Call: BreadthFirstSearch(graph,1,24,q,traffic,13):
                >>>[[1, 2, 9, 10, 11, 12, 19, 18, 25, 26, 33, 32, 31, 24]
             
'''
def BreadthFirstSearch(graph,start,end,q,XSignals,ASignals,shortestlength): ##Helper Function
    
    start_time = time.time() ##note start time 
    MultiPath = []
    
    temp_path = [start]
    q.enqueue(temp_path)
    counter = 0
    while q.IsEmpty() == False:
        tmp_path = q.dequeue()
        last_node = tmp_path[-1]
        
        if (last_node == end):
            length = len(tmp_path)
            
            if(length == shortestlength): 
                Acommon = set(ASignals).intersection(tmp_path)
                Xcommon = set(XSignals).intersection(tmp_path)
                
                if(not MultiPath)and(not Acommon): #If multipath is not set
                    MultiPath = tmp_path
                    
                elif((not Acommon)and(not Xcommon)): ##path without traffic signals
                    MultiPath = tmp_path
                    return MultiPath
                 
                #path passing through less signals than the previously calculated path     
                elif((Xcommon < set(Xcommon).intersection(MultiPath))and(not Acommon)): 
                    MultiPath = temp_path
                   
                counter = counter + 1
                
            if(counter > 50):
                return MultiPath
        
        for link_node in graph[last_node]:
            if link_node not in tmp_path:
                new_path = []
                new_path = tmp_path + [link_node]
                q.enqueue(new_path)
                
       
        
        timetaken = time.time() - start_time
        #Limiting the time it takes to run the algorithm to 5 seconds
        if(timetaken > 3):
            return MultiPath
            
    return MultiPath
    

class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance

'''
* Function Name: dijkstra
* Input:         graph - Graph object
                 initial - starting junction
* Logic:         Dijkstra algorithm
* Example Call:  dijkstra(graph, 8)    
'''
def dijkstra(graph, initial): ##Helper Function
    
    visited = {initial: 0}
    path = {}

    nodes = set(graph.nodes)

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            try:
                weight = current_weight + graph.distances[(min_node, edge)]
            except:
                continue
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path

'''
* Function Name: shortest_path
* Input:         graph - Graph object
                 origin - start junction
                 destination - destination junction
* Output:        list - shortest path of weighted graph
                 int - length of shortest path
* Logic:         finds the shortest path from the dijkstra algorithm  
* Example Call:  shortest_path(graph, 42,10)
                 (8, [42, 35, 28, 29, 22, 15, 8, 9, 10])
'''
def shortest_path(graph, origin, destination): ##Helper Function
   
    visited, paths = dijkstra(graph, origin)
    full_path = deque()
    _destination = paths[destination]

    while _destination != origin:
        full_path.appendleft(_destination)
        _destination = paths[_destination]

    full_path.appendleft(origin)
    full_path.append(destination)

    return visited[destination], list(full_path)
 
'''
* Function Name: path_planner
* Input:         Start - Start node number
                 End - End node number
                 TrafficPosition - list of positions of traffic
                 Unweighted - A graph object of Unweighted graph
                 AdjacencyList - Adjacency List of the graph in the image
* Logic:         Finds the shortest path following through minimum number of traffic signals using Breadth First Search and 
                 updates the status of the traffic signals in the global list-Signal
* Example Call:  path_planner(42,8,TrafficPosition.Unweighted, AdjacencyList)
'''
def path_planner(Start, End, TrafficPosition, Unweighted, AdjacencyList): ##Calling Function
    
    
    if(Start == End):
        return
   
    XSignals = [] #list of signals whose status is unknown
    ASignals = [] #list of active signals
    Flag = 0        
    
    while True:
                    
        #All the active and unknown signals
        for num in range(5):
            if(Signal[num] == 'X'):
                XSignals.append(TrafficPosition[num])
            
            elif(Signal[num] == 'A'):
                ASignals.append(TrafficPosition[num])
            
        #If all the signals are known then we will use Dijkstra algorithm to find shortest path in the weighted graph
        if(not('X' in Signal)):
            Flag = 1
            break
            
        #first find the length of the shortest path    
        shortestlength, path  = shortest_path(Unweighted,Start, End)
        path_queue = MyQUEUE()
        
        #Path passing over minimum number of unknown signals and no active signals
        MinPath = BreadthFirstSearch(AdjacencyList, Start, End, path_queue, XSignals, ASignals, shortestlength + 1)
        
        #If path not found then we will use Dijkstra algorithm
        if(not MinPath):
            Flag = 2
            break
        
        #If path found check the unknown number of signals it passes through 
        Xcommon = set(XSignals).intersection(MinPath)
        if(Xcommon):
            move_robot(MinPath,TrafficPosition,False,[],XSignals)
            return
        else:
            move_robot(MinPath,TrafficPosition,False)
            return
    
    ##runs the dijkstra algorithm to find the shortest path as now the status of all the traffic signal is known
    if(Flag == 1):
        #Making Weighted Graph
        Weighted = Graph()
        for node in AdjacencyList:
            
            Weighted.add_node(node)
            if(node in ASignals):
                #half of the time (30 seconds) needed to halt the robot over the traffic signal is added as weight
                #to each link connected to the junction as the robot must travel any two links connected to the Signal when passing 
                #over that junction
                weight = 15 + 10
            else:
                #weight of the link
                #it is the approximate time robot takes to travel one link
                weight = 10
            
            for link in AdjacencyList[node]: 
                Weighted.add_edge(node,link,weight)
    
        length, path  = shortest_path(Weighted,Start, End)
        move_robot(path,TrafficPosition, False)
        return
    
    while(Flag == 2):
       
        XSignals = []
        for num in range(5):
            if(Signal[num] == 'X'):
                XSignals.append(TrafficPosition[num])
            
            elif(Signal[num] == 'A'):
                ASignals.append(TrafficPosition[num])
        #Making Weighted Graph
        Weighted = Graph()
        for node in AdjacencyList:
            
            Weighted.add_node(node)
            if(node in ASignals):
                #half of the time (30 seconds) needed to halt the robot over the traffic signal is added as weight
                #to each link connected to the junction as the robot must travel any two links connected to the Signal when passing 
                #over that junction
                weight = 15 + 10
            else:
                #weight of the link
                #it is the approximate time robot takes to travel one link
                weight = 10
            
            for link in AdjacencyList[node]: 
                Weighted.add_edge(node,link,weight)
        
        length, MinPath  = shortest_path(Weighted,Start, End)
           
        Xcommon = set(XSignals).intersection(MinPath)
       
        if(Xcommon):
            move_robot(MinPath,TrafficPosition,True, ASignals,XSignals)
            return                
        else:
            move_robot(MinPath,TrafficPosition,False, ASignals)
            return