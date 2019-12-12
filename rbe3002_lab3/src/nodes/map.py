import cv2
import sys
import numpy as np
import copy
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from edge import Edge

np.set_printoptions(threshold=sys.maxsize)

class Map:
    #init function takes in map data returns a map object which stores the map as a 2d array
    def __init__(self, mapdata):
        self.metadata=mapdata.info
        self.data=np.reshape(mapdata.data,(mapdata.info.width,mapdata.info.height), order='F')

    def __str__(self):
        return str(self.metadata) + '\n' + str(self.data)

    def __nonzero__(self):
        return np.count_nonzero(self.data)==0

    #converts numpy map back into a 1d array
    def to_occupancy_grid(self):
        pass #do at some point

    #converts all elements > 0 to gridcells msg
    def to_grid_cells(self):
        msg=GridCells()
        msg.header.frame_id='map'
        msg.cell_width=self.metadata.resolution
        msg.cell_height=self.metadata.resolution
        msg.cells=[self.grid_to_world(loc) for loc,val in np.ndenumerate(self.data) if val > 0]
        return msg

    #converts list of (x,y) to gridcells
    def point_to_grid_cells(self,list):
        msg=GridCells()
        msg.header.frame_id='map'
        msg.cell_width=self.metadata.resolution
        msg.cell_height=self.metadata.resolution
        msg.cells=[self.grid_to_world(loc) for loc in list]
        return msg

    def c_space(self,d=1,path=False):
        map=copy.deepcopy(self)
        for index,x in np.ndenumerate(self.data):
            if path:
                if x<0:
                    neighbors=self.get_neighbors(index,d)
                    for n in neighbors:
                        map.data[n[0],n[1]]=100
            if x>70:
                neighbors=self.get_neighbors(index,d)
                for n in neighbors:
                    map.data[n[0],n[1]]=100
        return map


    #get nearest accesible neighbor
    def nearest_walkable_neigbor(self,(x,y)):
        i=1
        while self.data[x,y]!=0:
            neighbors=self.get_neighbors((x,y),i)
            for n in neighbors:
                if self.data[n[0],n[1]] == 0:
                    x=n[0]
                    y=n[1]
                    break
            i+=1
        return (x,y)

    #converts x,y to grid position in curr map
    def grid_to_world(self, (x,y)):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        #get importatn info out of mapdata
        resolution=self.metadata.resolution
        origin=self.metadata.origin.position
        #convert point
        return Point(x=((x+.5)*resolution+origin.x),y=((y+.5)*resolution+origin.y),z=0)

    #get list of neighbors around point with distance returns all neighbors with values above threshold
    def get_neighbors(self, point ,d=1,threshold=50, threshold2=-2):
        return [(x,y) for x in range(max(0,point[0]-d),min(self.metadata.width,point[0]+d+1)) for y in range(max(0,point[1]-d),min(self.metadata.height,point[1]+d+1)) if self.data[x,y] <= threshold and self.data[x,y] >= threshold2]
        # ^im sorry but also you cant stop me

    @staticmethod
    def euclidean_distance(p1, p2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        return math.hypot(p2[0]-p1[0],p2[1]-p1[1])

    def world_to_grid(self, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        #get importatn info out of mapdata
        resolution=self.metadata.resolution
        origin=self.metadata.origin.position
        #convert
        return (int((wp.x-origin.x)/resolution) ,int((wp.y-origin.y)/resolution))

    #displays map as image
    def show_map(self):
        #show map data as a picture
        cv2.imshow('image',self.data/np.average(self.data)*255)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    #modifies the boundries of a map and returns a new map object with appropriate borders if padding positive dialates if negative erodes
    def morph(self,padding=1):
        # duplicate map object
        new_map=copy.deepcopy(self)
        #define kernel on padding size
        ksize=abs(padding*2)+1
        kernel=np.ones((ksize,ksize),np.uint8)
        #convert map data between 0s and ones and convert to uint8
        max=float(np.max(self.data))
        if max<=0:
            max=1
        map=np.uint8(self.data/max)
        if padding>0:
            new_map.data=cv2.dilate(map,kernel,padding+1)
        elif padding<0:
            new_map.data=cv2.erode(map,kernel,padding+1)
        else:
            print('padding is 0. why?')
        #convert back into 0-100 space
        new_map.data=new_map.data*max
        return new_map

    #takes map with turns the map unkowns into detectable edges and makes all other unkown values 2 returns a new map
    def isolate_frontiers(self):
        data=copy.deepcopy(self.data)
        map=copy.deepcopy(self)
        #for each pixel is zero and neighbors is -1 set pixel to 100 else set pixel to 0
        for index,x in np.ndenumerate(self.data):
            neighbors=[self.data[n[0],n[1]] for n in self.get_neighbors(index)]
            if x==0 and -1 in neighbors:
                data[index[0],index[1]]=100
            else:
                data[index[0],index[1]]=0
        #return new map
        map.data=data
        return map

    #returns a new map with all the edges found uses Canny edge detection
    def edge_detector(self, blur=False, sigma=.33):
        map=copy.deepcopy(self)
        map.data=np.uint8(self.data)
        # compute the median of the single channel pixel intensities
        #v = np.median(map.data)

        #if blur specified blur the image
        if blur:
            map.data=cv2.GaussianBlur(map.data, (3,3), 0)

	    # apply automatic Canny edge detection using the computed median
	    #lower = int(max(0, (1.0 - sigma) * v))
	    #upper = int(min(255, (1.0 + sigma) * v))
        #calc edges
        map.data=cv2.Canny(map.data,50,150)

        return map


    #finds center of edges on a map returns a list of Edge Objects
    def to_edges(self):
        img=self.data
        #find contours in wait_for_message
        edge_list=[]
        _, thresh = cv2.threshold(np.uint8(self.data), 0, 255, cv2.THRESH_BINARY)
        _, contours, _=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        for c in contours:
            m = cv2.moments(c)
            if m["m00"] != 0:
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])
            else:
                cx, cy = 0, 0
                continue
            edge_list.append(Edge(cv2.arcLength(c,True),(cy,cx)))
            print([str(e) for e in edge_list])
        return [e for e in edge_list if e.size>=1]
