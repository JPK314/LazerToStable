import numpy
import bisect
import src.PathApproximator as PathApproximator
from src.PathControlPoint import PathControlPoint


class SliderPath:

    # <summary>
    # The user-set distance of the path. If non-null, <see cref="Distance"/> will match this value,
    # and the path will be shortened/lengthened to match this length.
    # </summary>   
    ExpectedDistance = None
    
    # <summary>
    # The control points of the path
    # </summary>
    ControlPoints = []
    
    calculatedPath = []
    cumulativeLength = []
    
    calculatedLength = numpy.single(0)
    
    
    # <summary>
    # Creates a new <see cref="SliderPath"/> initialised with a list of control points.
    # </summary>
    # <param name="controlPoints">An optional set of <see cref="PathControlPoint"/>s to initialise the path with.</param>
    # <param name="expectedDistance">A user-set distance of the path that may be shorter or longer than the true distance between all control points.
    # The path will be shortened/lengthened to match this length. If null, the path will use the true distance between all control points.</param>
    def __init__(self, ControlPoints, ExpectedDistance=None):
        self.ControlPoints = ControlPoints
        self.ExpectedDistance = numpy.single(ExpectedDistance)
        self.calculatePath()
        self.calculateLength()
        
    # <summary>
    # The distance of the path after lengthening/shortening to account for <see cref="ExpectedDistance"/>.
    # </summary>
    def Distance(self):
        return 0 if len(self.cumulativeLength)==0 else self.cumulativeLength[-1]
    
    # <summary>
    # The distance of the path prior to lengthening/shortening to account for <see cref="ExpectedDistance"/>.
    # </summary>
    def CalculatedDistance(self):
        return self.calculatedLength
    
    # <summary>
    # Computes the slider path until a given progress that ranges from 0 (beginning of the slider)
    # to 1 (end of the slider) and stores the generated path in the given list.
    # </summary>
    # <param name="path">The list to be filled with the computed path.</param>
    # <param name="p0">Start progress. Ranges from 0 (beginning of the slider) to 1 (end of the slider).</param>
    # <param name="p1">End progress. Ranges from 0 (beginning of the slider) to 1 (end of the slider).</param>
    def GetPathToProgress(self, path, p0, p1):
        p0 = numpy.single(p0)
        p1 = numpy.single(p1)
        d0 = self.progressToDistance(p0)
        d1 = self.progressToDistance(p1)
        
        path.clear()
        
        i = 0
        while (i<len(self.calculatedPath) and self.cumulativeLength[i] < d0):
            i = i+1
        
        path.append(self.interpolateVertices(i, d0))
        
        while (i<len(self.calculatedPath) and self.cumulativeLength[i] <= d1):
            path.append(self.calculatedPath[i])
            i = i+1
        
        path.append(self.interpolateVertices(i, d1))
    
    # <summary>
    # Computes the position on the slider at a given progress that ranges from 0 (beginning of the path)
    # to 1 (end of the path).
    # </summary>
    # <param name="progress">Ranges from 0 (beginning of the path) to 1 (end of the path).</param>
    # <returns></returns>
    def PositionAt(self, progress):
        d = self.progressToDistance(numpy.single(progress))
        return self.interpolateVertices(self.indexOfDistance(d), d)
    
    
    def calculatePath(self):
        self.calculatedPath.clear()
        if (len(self.ControlPoints) == 0):
            return
        
        vertices = []
        for i in range(0, len(self.ControlPoints)):
            vertices.append(self.ControlPoints[i].Position)
            
        start = 0
        
        for i in range(0, len(self.ControlPoints)):
            if (self.ControlPoints[i].Type == None and i < len(self.ControlPoints)-1):
                continue
            
            # The current vertex ends the segment
            segmentVertices = vertices[start:(i+1)]
            segmentType = self.ControlPoints[start].Type
            if (segmentType == None):
                segmentType = PathControlPoint.LINEAR
                
            for t in self.calculateSubPath(segmentVertices, segmentType):
                if (len(self.calculatedPath) == 0 or not numpy.array_equal(self.calculatedPath[-1],t,equal_nan=True)):
                    self.calculatedPath.append(t)
                
            # Start the new segment at the current vertex
            start = i
            
    def calculateSubPath(self, subControlPoints, typevar):
        if (typevar == PathControlPoint.LINEAR):
            return PathApproximator.ApproximateLinear(subControlPoints)
        elif (typevar == PathControlPoint.PERFECT and len(subControlPoints) == 3):
            subpath = PathApproximator.ApproximateCircularArc(subControlPoints)
            
            # If for some reason a circular arc could not be fit to the 3 given points, fall back to a numerically stable bezier approximation.
            if (len(subpath) != 0):
                return subpath
            
        return PathApproximator.ApproximateBezier(subControlPoints)
    
    def calculateLength(self):
        self.calculatedLength = numpy.single(0)
        self.cumulativeLength.clear()
        self.cumulativeLength.append(numpy.single(0))
        
        for i in range(0, len(self.calculatedPath)-1):
            diff = self.calculatedPath[i+1] - self.calculatedPath[i]
            self.calculatedLength = self.calculatedLength + numpy.linalg.norm(diff)
            self.cumulativeLength.append(self.calculatedLength)
            
        if (self.ExpectedDistance != None):
            expectedDistance = self.ExpectedDistance
            if (self.calculatedLength != expectedDistance):
                # The last length is always incorrect
                del self.cumulativeLength[-1]
                
                pathEndIndex = len(self.calculatedPath)-1
                
                if (self.calculatedLength > expectedDistance):
                    # The path will be shortened further, in which case we should trim any more unnecessary lengths and their associated path segments
                    while (len(self.cumulativeLength) > 0 and self.cumulativeLength[-1] >= expectedDistance):
                        del self.cumulativeLength[-1]
                        del self.calculatedPath[pathEndIndex]
                        pathEndIndex = pathEndIndex-1
                
                if (pathEndIndex <= 0):
                    # The expected distance is negative or zero
                    # TODO: Perhaps negative path lengths should be disallowed altogether
                    self.cumulativeLength.append(numpy.single(0))
                    return
                
                # The direction of the segment to shorten or lengthen
                dir = self.calculatedPath[pathEndIndex]-self.calculatedPath[pathEndIndex-1]
                dir = dir/numpy.linalg.norm(dir)
                
                self.calculatedPath[pathEndIndex] = self.calculatedPath[pathEndIndex-1]+dir*(expectedDistance-self.cumulativeLength[-1])
                self.cumulativeLength.append(expectedDistance)
    
    def indexOfDistance(self, d):
        return bisect.bisect_left(self.cumulativeLength, d)
    
    def progressToDistance(self, progress):
        return max((numpy.single(0), min((progress, numpy.single(1)))))*self.Distance()
    
    def interpolateVertices(self, i, d):
        if (len(self.calculatedPath) == 0):
            return numpy.array((0,0), dtype='single')
        
        if (i<=0):
            return self.calculatedPath[0]
        
        if(i >= len(self.calculatedPath)):
            return self.calculatedPath[-1]
        
        p0 = self.calculatedPath[i-1]
        p1 = self.calculatedPath[i]
        
        d0 = self.cumulativeLength[i-1]
        d1 = self.cumulativeLength[i]
        
        # Avoid division by an almost-zero number in case two points are extremely close to each other.
        if (numpy.isclose(d0, d1)):
            return p0
        
        w = (d-d0)/(d1-d0)
        return p0+(p1-p0)*w
        
