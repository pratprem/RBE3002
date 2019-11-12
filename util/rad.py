import math


class Radian:
    def __init__(self, angle, degrees=False):
        if type(angle) is int:
            angle=float(angle)
        elif type(angle) is not float:
            raise ValueError('expected type float recieved (!r)'.format(type(angle)))
        if degrees:
            angle=_degree_to_radians(a)
        self.value=self._norm(angle)

    def __add__(self, other):
        if type(other) == Radian:
            return Radian(self.value + other.value)
        else:
            return Radian(self.value + other)

    def __sub__(self, other):
        if type(other) == Radian:
            return Radian(self.value - other.value)
        else:
            return Radian(self.value - other)

    def __mul__(self, other):
        if type(other) == Radian:
            return Radian(self.value * other.value)
        else:
            return Radian(self.value * other)

    def __truediv__(self, other):
        if type(other) == Radian:
            return Radian(self.value / other.value)
        else:
            return Radian(self.value / other)

    def __str__(self):
        return str(self.value)+" radians"

    def __cmp__(self, other):
        if type(other )== Radian:
            return int(self.value - other.value)
        else:
            return int(self.valuei - other)

    def _degree_to_radians(a):
        """
        takes float a and coverts from degrees to radians
        :param a    [float] [radians] angle to conver
        """
        return 2*math.pi*a/360.0

    def norm(*args):
        """
        takes a set list of floats and returns a list of Radian Objects wrapped
        to -pi to pi space
        :param  args        [float] [radians] an list of angles to wrap
        :return angles      [float] [radians] list of radian object
        """
        # takes all angles and creates Radian Objects
        r=[Radian(a) for a in args]

        # if len of result is 1 unpack tuple else dont touch
        if len(result) == 1:
            return result[0]
        else:
            return result
        return result


    def _norm(self, *args):
        """
        internal functions to wrap all angles into -pi to pi space
        :param  args        [float] [radians] an list of angles to wrap
        :return angles      [float] [radians] list of angles returned -pi to pi
        """
        # convert angle to 0 to 2pi space
        r=[math.fmod(math.fmod(a, 2*math.pi) + 2*math.pi, 2*math.pi)
                     for a in args]
        # convert from wrap angle form pi to 2pi to -pi to zero
        result=tuple([a-2*math.pi if a >= math.pi else a for a in r])

        # if len of result is 1 unpack tuple else dont touch
        if len(result) == 1:
            return result[0]
        else:
            return result
        return result

