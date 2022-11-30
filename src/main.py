import numpy
from src.PathControlPoint import PathControlPoint
from src.BezierConverter import ConvertToBezierAnchors

def zeroFloor(x):
    return int(numpy.floor(x) + (0 if x >= 0 else 1))

def processSlider(x, y, path, version):
    pos = numpy.array((x, y), dtype='single')
    segments_control_points = convertPathString(path, pos, version)

    # if there's only one segment - we don't need to process it
    if len(segments_control_points) == 1:
        return path

    all_points = []
    for i in range(len(segments_control_points)):
        if i > 0 and len(segments_control_points[i-1]) > 0:
            first_point = segments_control_points[i-1][-1]
            first_point.Type = segments_control_points[i][0].Type
            all_points += ConvertToBezierAnchors(segments_control_points[i])
        else:
            all_points += ConvertToBezierAnchors(segments_control_points[i])

    return "B|" + "|".join(["%d:%d" % (int(numpy.round((x[0]))), int(numpy.round((x[1])))) for x in all_points])

def convertPathType(input):
    if (input[0] == "C"):
        return PathControlPoint.CATMULL
    elif (input[0] == "B"):
        return PathControlPoint.BEZIER
    elif (input[0] == "L"):
        return PathControlPoint.LINEAR
    elif (input[0] == "P"):
        return PathControlPoint.PERFECT
    return -1

def clamp(val, absmax):
    return max(min(val, absmax), -absmax)

def readPoint(value, startPos):
    vertexSplit = value.split(":")
    pos = numpy.array((zeroFloor(clamp(float(vertexSplit[0]), 131072)), zeroFloor(clamp(float(vertexSplit[1]), 131072))), dtype='single') - startPos
    return PathControlPoint(list(pos))

def isLinear(p):
    return numpy.isclose(0, (p[1].Position[1] - p[0].Position[1]) * (p[2].Position[0] - p[0].Position[0]) - (p[1].Position[0] - p[0].Position[0]) * (p[2].Position[1] - p[0].Position[1]))

def convertPoints(points, endPoint, first, offset, formatVersion):
    typevar = convertPathType(points[0])
    # First control point is zero for the first segment
    readOffset = 1 if first else 0
    # Total points readable from the base point span
    readablePoints = len(points) - 1
    # Extra length if an endpoint is given that lies outside the base point span
    endPointLength = 1 if endPoint != None else 0
    
    vertices = []
    
    # Fill any non-read points.
    for _ in range(readOffset):
        vertices.append(PathControlPoint((0,0)))
    
    # Parse into control points.
    for i in range(1,len(points)):
        vertices.append(readPoint(points[i], offset))
    
    # If an endpoint is given, add it to the end.
    if (endPoint != None):
        vertices.append(readPoint(endPoint, offset))
    
    # Edge-case rules (to match stable)
    if (typevar == PathControlPoint.PERFECT):
        if (len(vertices) != 3):
            typevar = PathControlPoint.BEZIER
        elif (isLinear(vertices)):
            typevar = PathControlPoint.LINEAR
    
    for vertex in vertices:
        vertex.Type = typevar
    
    # A path can have multiple implicit segments of the same type if there are two sequential control points with the same position.
    # To handle such cases, this code may return multiple path segments with the final control point in each segment having a non-null type.
    # For the point string X|1:1|2:2|2:2|3:3, this code returns the segments:
    # X: { (1, 1), (2, 2) }
    # X: { (2, 2), (3, 3) }
    # Note: in Lazer code, (2, 2) is not returned in the second segment, as it is implicit in the path. However, I want this in the second segment,
    # so I will modify the code below to achieve this
    startIndex = 0
    endIndex = 1
    
    segments = []
    
    # The conditions of this loop have been modified from the Lazer code so that the end point of a segment is included in the segment itself.
    while(endIndex < len(vertices)):
        # Keep incrementing while an implicit segment doesn't need to be started
        if (not all(vertices[endIndex].Position == vertices[endIndex - 1].Position)):
            endIndex += 1
            continue
        
        # Legacy Catmull sliders don't support multiple segments, so adjacent Catmull segments should be treated as a single one.
        # Importantly, this is not applied to the first control point, which may duplicate the slider path's position
        # resulting in a duplicate (0,0) control point in the resultant list.
        if (typevar == PathControlPoint.CATMULL and endIndex > 1 and formatVersion < 128):
            endIndex += 1
            continue
        
        # The last control point of each segment is not allowed to start a new implicit segment
        if (endIndex == len(vertices) - endPointLength - 1):
            endIndex += 1
            continue
        
        # Return the current control point set as a segment
        # In Lazer, zero length segments can be returned by this method. I don't like this so I'm changing it to suit my needs here
        if startIndex < endIndex:
            segments.append(vertices[startIndex:endIndex])
        
        # Lazer would skip the current control point as it's the same as the one that's just been returned. However, this removes it from the following
        # segment, and I want it in. Therefore we do not skip.
        startIndex = endIndex
        endIndex += 1
    
    if (endIndex > startIndex):
        segments.append(vertices[startIndex:endIndex])
    
    return [[PathControlPoint(x.Position + offset, typevar=x.Type) for x in segment] for segment in segments]
        
def convertPathString(pointString, offset, formatVersion):
    pointSplit = pointString.split("|")
    
    segmentsControlPoints = []
    startIndex = 0
    endIndex = 1
    first = True
    
    while (endIndex < len(pointSplit)):
        # Keep incrementing endIndex while it's not the start of a new segment (indicated by having a type descriptor of length 1).
        if (len(pointSplit[endIndex]) > 1):
            endIndex += 1
            continue
        
        # In the Lazer stable code, multi-segmented sliders DON'T contain the end point as part of the current segment as it's assumed to be the start of the next segment.
        # The start of the next segment is the index after the type descriptor.
        # However, for ease of use for me, I WILL be including the end point.
        endPoint = pointSplit[endIndex+1] if endIndex < len(pointSplit) - 1 else None
        
        segmentsControlPoints += convertPoints(pointSplit[startIndex:endIndex], endPoint, first, offset, formatVersion)
        startIndex = endIndex
        first = False
        endIndex += 1
    
    if (endIndex > startIndex):
        segmentsControlPoints += convertPoints(pointSplit[startIndex:endIndex], None, first, offset, formatVersion)
        
    return segmentsControlPoints
