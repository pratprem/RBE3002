class Edge:
    #takes a list of (x,y) tuples that makes up the edge
    def __init__(self,size,centroid):
        self.size=size
        self.centroid=centroid

    def __str__(self):
        return 'Edge:\n'+ '--Centroid:' + str(self.centroid) + '\n--Size:' + str(self.size)
