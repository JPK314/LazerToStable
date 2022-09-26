import numpy
from src.PathApproximator import GetCircleArcProperties
from src.PathControlPoint import PathControlPoint

# Adapted from BezierConverter.cs in Olibomby's Mapping Tools repository and ppy's osu!Framework repository
def GetPoints(str):
    strPoints = str.split("|")
    points = []
    for strPoint in strPoints:
        strCoords = strPoint.split(":")
        points.append(numpy.array((float(strCoords[0]), float(strCoords[1])), dtype='single'))
    return points

CIRCLE_PRESETS = [
    {
        "MaxAngle": 0.4993379862754501,
        "Points": GetPoints("1.0:0.0|1.0:0.2549893626632736|0.8778997558480327:0.47884446188920726")
    },
    {
        "MaxAngle": 1.7579419829169447,
        "Points": GetPoints("1.0:0.0|1.0:0.6263026|0.42931178:1.0990661|-0.18605515:0.9825393")
    },
    {
        "MaxAngle": 3.1385246920140215,
        "Points": GetPoints("1.0:0.0|1.0:0.87084764|0.002304826:1.5033062|-0.9973236:0.8739115|-0.9999953:0.0030679568")
    },
    {
        "MaxAngle": 5.69720464620727,
        "Points": GetPoints("1.0:0.0|1.0:1.4137783|-1.4305235:2.0779421|-2.3410065:-0.94017583|0.05132711:-1.7309346|0.8331702:-0.5530167")
    },
    {
        "MaxAngle": 2 * numpy.pi,
        "Points": GetPoints("1.0:0.0|1.0:1.2447058|-0.8526471:2.118367|-2.6211002:7.854936e-06|-0.8526448:-2.118357|1.0:-1.2447058|1.0:0.0")
    },
    
]

def ConvertToBezierAnchors(controlPoints):
    if len(controlPoints) == 0:
        print("wtf")
    # First control point must have a type
    points = [x.Position for x in controlPoints]
    if (controlPoints[0].Type == PathControlPoint.LINEAR):
        return ConvertLinearToBezierAnchors(points)
    elif (controlPoints[0].Type == PathControlPoint.PERFECT):
        return ConvertCircleToBezierAnchors(points)
    elif (controlPoints[0].Type == PathControlPoint.CATMULL):
        return ConvertCatmullToBezierAnchors(points)
    else:
        return points

def ConvertCircleToBezierAnchors(points):
    cs = GetCircleArcProperties(points)
    
    if (not cs["IsValid"]):
        return points
    
    preset = CIRCLE_PRESETS[-1]
    for CBP in CIRCLE_PRESETS:
        if (CBP["MaxAngle"] >= cs["ThetaRange"]):
            preset = CBP
            break
    arc = preset["Points"].copy()
    arcLength = preset["MaxAngle"]
    
    # Converge on arcLength of thetaRange
    n = len(arc) - 1
    tf = cs["ThetaRange"] / arcLength
    while (numpy.abs(tf - 1) > 0.000001):
        for j in range(n):
            for i in range(n,j,-1):
                arc[i] = arc[i] * tf + arc[i-1] * (1-tf)
        arcLength = numpy.arctan2(arc[-1][1], arc[-1][0])
        if (arcLength < 0):
            arcLength += 2 * numpy.pi
        tf = cs["ThetaRange"] / arcLength
        
    # Adjust rotation, radius, and position
    rot = cs["Radius"]*numpy.array(((numpy.cos(cs["ThetaStart"]), -numpy.sin(cs["ThetaStart"]) * cs["Direction"]),
                      (numpy.sin(cs["ThetaStart"]), numpy.cos(cs["ThetaStart"]) * cs["Direction"])), dtype='double')
    for i in range(len(arc)):
        arc[i] = numpy.dot(rot, arc[i]) + cs["Centre"]
        
    # To fix some errors, set the first/last point of arc to the first/last point of points
    arc[0] = points[0]
    arc[-1] = points[-1]
    return arc
    
def ConvertCatmullToBezierAnchors(points):
    cubics = [points[0]]
    for i in range(len(points)-1):
        v1 = points[i-1] if i > 0 else points[i]
        v2 = points[i]
        v3 = points[i+1] if i < len(points) - 1 else v2 + v2 - v1
        v4 = points[i+2] if i < len(points) - 2 else v3 + v3 - v2
        cubics.append((-v1 + 6 * v2 + v3) / 6)
        cubics.append((-v4 + 6 * v3 + v2) / 6)
        cubics.add(v3)
        cubics.add(v3)
    cubics = cubics[:-1]
    return cubics

def ConvertLinearToBezierAnchors(points):
    bezier = [points[0]]
    for i in range(1,len(points)):
        bezier.append(points[i])
        bezier.append(points[i])
    bezier = bezier[:-1]
    return bezier

    
