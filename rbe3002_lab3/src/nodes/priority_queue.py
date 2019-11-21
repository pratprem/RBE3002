import heapq

class PriorityQueue:

    def __init__(self, start=None):
        """
        Class constructor.
        """
        self.elements = []
        if start:
            self.push((0,start,None))

    def __nonzero__(self):
        """
        Returns False if the queue is empty, True otherwise.
        """
        return len(self.elements)

    def put(self, *args):
        """
        Puts an element in the queue.
        :param args takes a list of tuples (priority,element) to push
        """
        [heapq.heappush(self.elements, el) for el in args]

    def pop(self):
        """
        Returns the element with the top priority.
        """
        return heapq.heappop(self.elements)[1:3]

    def get_elements(self):
        """
        Returns the list of (x,y)
        """
        return [e for priority ,e ,prev_e in queue.elements]
