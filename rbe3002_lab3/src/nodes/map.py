import cv2
import sys
import numpy as np
import copy
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells

np.set_printoptions(threshold=sys.maxsize)

class Map:
    #init function takes in map data returns a map object which stores the map as a 2d array
    def __init__(self, mapdata):
        self.metadata=mapdata.info
        self.data=np.reshape(mapdata.data,(mapdata.info.width,mapdata.info.height), order='F')

    def __str__(self):
        return str(self.metadata) + '\n' + str(self.data)

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
        origin=self.metadata.origin
        #convert point
        return Point(x=((x+.5)*resolution+origin.position.x),y=((y+.5)*resolution+origin.position.y),z=0)

    #get list of neighbors around point with distance returns all neighbors with values above threshold
    def get_neighbors(point ,distance=1,threshold=-1):
        return [(x,y) for x in range(max(0,point[0]-d),min(self.metadata.width,point[0]+d+1)) for y in range(max(0,point[1]-d),min(self.metadata.height,point[1]+d+1)) if self.data[x,y] > threshold]
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
        origin=self.metadata.origin
        #convert
        return (int(wp.x-origin.x)/resolution) ,int(wp.y-origin.y)/resolution))

    #displays map as image
    def show_map(self):
        #show map data as a picture
        cv2.imshow('image',self.data*255)
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
        map=np.uint8(self.data/100.0)
        if padding>0:
            new_map.data=cv2.dilate(map,kernel,padding+1)
        elif padding<0:
            new_map.data=cv2.erode(self.data,kernel,iterations=1)
        else:
            print('padding is 0. why?')
        #convert back into 0-100 space
        new_map.data=new_map.data*100.0
        return new_map
