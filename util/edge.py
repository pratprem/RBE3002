class Edge:
    #takes a list of (x,y) tuples that makes up the edge 
    def __init__(self,points):
        self.points=points
        self.size=float(len(points))
        self.centroid=self.calc_centroid()

    def calc_centroid(self):
        x_list,y_list=zip(*self.points)
        return (sum(x_list)/self.size,sum(y_list)/self.size)