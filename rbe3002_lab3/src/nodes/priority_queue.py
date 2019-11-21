import heapq

class PriorityQueue:

    def __init__(self, start=None):
        """
        Class constructor.
        """
        self.elements = []
        if start:
            self.put((0,start,None))

    def __nonzero__(self):
        """
        Returns False if the queue is empty, True otherwise.
        """
        return len(self.elements) != 0

    def put(self, el):
        """
        Puts an element in the queue.
        :param args takes a list of tuples (priority,element) to push
        """
        heapq.heappush(self.elements, el)

    def pop(self):
        """
        Returns the element with the top priority.
        """
        return heapq.heappop(self.elements)[1:3]

    def get_elements(self):
        """
        Returns the list of (x,y)
        """
        return [e[1] for e in self.elements]
