# -*- coding: cp1252 -*-
'''
* Team Id :         2941
* Author List :     K.S.Santosh
* Filename:         CourierServiceMain.py
* Theme:            Courier Service
* Functions:        courier_service(img), output_package(packs)
* Global Variables: None
'''
import package_client
from PackageDetails import *
from GraphOfGrid import *
from TrafficSignals import find_traffic_position_number
from PathTraversal import *

#ipV4 address 
package_client.package_server_ip = 'XXX.XXX.XX.XX'

'''
    * Function Name: output_package 
    * Input:         packs - list of shape and color of packages
    * Output:        Shows graphical output of the package on the screen
    * Logic:         Calls all the appropriate function from the package 
    * Example Call : output_package([["Square","Blue"], ["Square","Orange"]])
'''
def output_package(packs, PackageStatus):
    
    
    Shape = packages[0][0]
    Color = packages[1][0]
    parameter = Color + Shape + PackageStatus
    package_client.Message(parameter)
    time.sleep(3)##after blinking the blocks 3 times
        
    print packs
    return
'''
    * Function Name: courier_service
    * Input:         img - one of the test images
    * Output:        Plans the robot motion and the path to pick up and deliver the packages in least amount of time.
    * Logic:         Calls all the appropriate functions for path traversal. This only gives the start and end node of each path.
    * Example Call : courier_service(img)
'''    
def courier_service(img): ##Control Function
    
    
    #Got all the active warehouses, packages and delivery junctions 
    Packages = pickup_deliver_junctions(img)
    
    #Got the adjacency list of the grid
    AdjacencyList = make_graph(img)
    
    #Making list of only packages to deliver
    PackToDeliver = []
    for chunks in Packages:
        puj = chunks[0] + 8
        for packs in chunks[1]:
            temp = [puj] + packs
            PackToDeliver.append(temp) 
  
    #Got the Traffic Signal position
    TrafficPosition = find_traffic_position_number(img)
   
    #Making Dijkstra graph
    Unweighted = Graph()
    for node in AdjacencyList:
        
        Unweighted.add_node(node)
        
        for link in AdjacencyList[node]:
            Unweighted.add_edge(node,link,1)
    
    #holds all the pick up and delivery stations along with the package details.
    Station = []
   
    while(PackToDeliver):
        if(len(PackToDeliver) >= 4):
            Temp4 = PackToDeliver[0:4]
            temp = ["P"]
            for puj in Temp4 :    
                temp = temp + puj[0:1] + [puj[1:3]]
            
            Station.append(temp)
            
            Temp4 = sorted(Temp4, key=lambda x: x[-1])  
            
            temp = ["D"]
            for puj in Temp4 :    
                temp = temp + puj[3:4] + [puj[1:3]]
            Station.append(temp)    
            
            del PackToDeliver[0:4]    
        else:
            temp = ["P"]
            for puj in PackToDeliver :    
                temp = temp + puj[0:1] + [puj[1:3]]
            
            Station.append(temp)
            
            PackToDeliver = sorted(PackToDeliver, key=lambda x: x[-1])  
            
            temp = ["D"]
            for puj in PackToDeliver :    
                temp = temp + puj[3:4] + [puj[1:3]]
            Station.append(temp) 
            
            del PackToDeliver[:]
    
    Start = 42 #Starting point(green node)
    End = 42
    
    print Station
    
    #calculates a series of starting and ending node to be followed
    #loop runs till all the Station in the grid are visited
    for batch in Station :
        if batch[0] == "P" :
            ##Pick-Up Path 
            for packs in batch[1:]:
                if(isinstance(packs, int)):
                    End = packs
                    path_planner(Start,End,TrafficPosition,Unweighted,AdjacencyList)
                    Start = End
                else:
                    output_package(packs , '1')
        else: 
            ##Delivery Path
            for packs in batch[1:]:
                if(isinstance(packs, int)):
                    End = packs
                    path_planner(Start,End,TrafficPosition,Unweighted,AdjacencyList)
                    Start = End
                else:
                    output_package(packs, '0')
                    
    ##shows all the squares
    FinalPackages = [["Square","Blue"], ["Square","Orange"],["Square","Pink"], ["Square","Green"]]
    for packs in FinalPackages:    
        output_package(packs, '1')
    ##closing the connection
    package_client.Message('CCC')
    return

#####################################################################
## Main Function
#####################################################################
if __name__ == "__main__":
    
    ##reading the image and minimising its size     
    image = cv2.imread('CS_Test_Image.jpg')
    eight, width, channels = image.shape
    img = cv2.resize(image,(750, 750), interpolation = cv2.INTER_CUBIC)
    
    ##Start your service!
    courier_service(img)
    print Signal
    cv2.waitKey(0)
    cv2.destroyAllWindows()