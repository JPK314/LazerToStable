import numpy
import os
import re
from src.PathControlPoint import PathControlPoint
from src.SliderPath import SliderPath
from src.BezierConverter import ConvertToBezierAnchors

def main():
    for file in os.listdir("."):
        if file.endswith(".osu"):
            FD = open(file, 'r', encoding="utf8")
            lines = FD.readlines()
            sliders = []
            formatVersion = int(re.search(r"osu file format v(\d+)$", lines[0]).group(1))
                    
            # Searching for sliders
            insideHOs = False
            for line in lines:
                if line == "[HitObjects]\n":
                    insideHOs = True
                    continue
                if insideHOs and line == "\n":
                    insideHOs = False
                    
                if insideHOs:
                    strspl = line.split(",")
                    xpos = zeroFloor(clamp(float(strspl[0]), 131072))
                    ypos = zeroFloor(clamp(float(strspl[1]), 131072))
                    time = float(strspl[2])
                    objType = clamp(int(strspl[3]), 131072)
                    if (objType & 2):
                        hitSound = clamp(int(strspl[4]), 131072)
                        positionsString = strspl[5]
                        repeats = clamp(int(strspl[6]), 131072)
                        length = None
                        if (len(strspl) > 7):
                            length = max((0, clamp(float(strspl[7]), 131072)))
                            if (length == 0):
                                length = None
                        
                        rest = ",".join(strspl[8:])
                        sliders.append([line, processSlider(xpos, ypos, time, objType, hitSound, positionsString, repeats, length, rest, formatVersion)])
                        
                        
            # Making new .osu file
            FDW = open(file[:-5]+"-STABLE].osu", 'w', encoding="utf8")
            FDW.write("osu file format v14\n")
            unchangedline = True
            insideHOs = False
            insideTPs = False
            for line in lines[1:]:
                unchangedline = True
                if line == "[TimingPoints]\n":
                    insideTPs = True
                    FDW.write(line)
                    continue
                if insideTPs and line == "\n":
                    insideTPs = False
                if line == "[HitObjects]\n":
                    insideHOs = True
                    FDW.write(line)
                    continue
                if insideHOs and line == "\n":
                    insideHOs = False
                
                if insideTPs:
                    strspl = line.split(",")
                    unchangedline = False
                    FDW.write("%d,%s" % (int(numpy.floor(float(strspl[0]))), ",".join(strspl[1:])))
                
                if insideHOs:
                    strspl = line.split(",")
                    objType = clamp(int(strspl[3]), 131072)
                    if (objType & 2):
                        unchangedline = False
                        matchingsliders = [x[1] for x in sliders if x[0] == line]
                        if matchingsliders:
                            s = matchingsliders[0]
                            FDW.write("%d,%d,%d,%d,%d,B|%s,%d,%f,%s" % (s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8]))
                        else:
                            FDW.write(line)
                        
                match = re.search(r"^Version:(.*)$", line)
                if match:
                    unchangedline = False
                    FDW.write("Version:%s-STABLE\n" % match.group(1))
                    
                if unchangedline:
                    FDW.write(line)
                    
            FDW.close()
            FD.close()

def zeroFloor(x):
    return numpy.floor(x) + (0 if x >= 0 else 1)

def processSlider(xpos, ypos, time, objtype, hitSound, positionsString, repeats, length, rest, formatVersion):
    pos = numpy.array((xpos, ypos), dtype='single')
    
    segmentsControlPoints = convertPathString(positionsString, pos, formatVersion)
    allPoints = []
    for i in range(len(segmentsControlPoints)):
        if i < len(segmentsControlPoints) - 1 and len(segmentsControlPoints[i+1]) > 0:
            allPoints += ConvertToBezierAnchors(segmentsControlPoints[i] + [segmentsControlPoints[i+1][0]])
        else:
            allPoints += ConvertToBezierAnchors(segmentsControlPoints[i])
    
        
    return (xpos, ypos, time, objtype, hitSound, "|".join(["%d:%d" % (int(numpy.round((x[0]))), int(numpy.round((x[1])))) for x in allPoints]), repeats, length, rest)

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
    # X: { (1,1), (2, 2) }
    # X: { (3, 3) }
    # Note: (2, 2) is not returned in the second segments, as it is implicit in the path.
    startIndex = 0
    endIndex = 1
    
    segments = []
    
    while(endIndex < len(vertices) - endPointLength):
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
        segments.append(vertices[startIndex:endIndex])
        
        # Skip the current control point - as it's the same as the one that's just been returned.
        startIndex = endIndex + 1
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
        
        # Multi-segmented sliders DON'T contain the end point as part of the current segment as it's assumed to be the start of the next segment.
        # The start of the next segment is the index after the type descriptor.
        endPoint = pointSplit[endIndex+1] if endIndex < len(pointSplit) - 1 else None
        
        segmentsControlPoints += convertPoints(pointSplit[startIndex:endIndex], endPoint, first, offset, formatVersion)
        startIndex = endIndex
        first = False
        endIndex += 1
    
    if (endIndex > startIndex):
        segmentsControlPoints += convertPoints(pointSplit[startIndex:endIndex], None, first, offset, formatVersion)
        
    return segmentsControlPoints
        
if __name__ == "__main__":
    main()
