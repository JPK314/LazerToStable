import numpy

class PathControlPoint:
    
    LINEAR = 0
    PERFECT = 1
    BEZIER = 2
    CATMULL = 3
    
    
    Position = numpy.array((0,0), dtype='single')
    
    Type = None
    
    def __init__(self, position, typevar=None):
        self.Position = numpy.single(position)
        self.Type = typevar
        
    def Equals(self, other):
        return (all(self.Position == other.Position) and self.Type == other.Type)