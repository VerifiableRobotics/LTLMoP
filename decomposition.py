#!/usr/bin/env python

import math
import Polygon, Polygon.IO, Polygon.Utils, Polygon.Shapes


class myVertex():
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        
class decomposition():

    def __init__(self,polygon,holes=[]):
        """
        A function that decompose a simple polygon with MP5 method to convex polygons
        Return a list of convex polygon
        """

        # Initialization
        self.background = Polygon.Polygon(((-100,-100),(900,-100),(900,900),(-100,900)))
        self.P = polygon
        
        if self.P.orientation()[0] > 0:
            self.P = self.reversePolyOrientation(polygon) # A copy of the input polygon
        
        self.holeList = holes
        
        self.listOfConvexPoly = [] # List of convex polygons

        #self.drawPoly([self.P],'Initial')
        poly = []
        poly.append(self.P)
        for hole in self.holeList:
            poly.append(hole)
        #self.drawPoly(poly,'Initial')
        #print self.P
        
                
    def MP5(self):
        #print 'Mission Starts'
        self.notchVertices = [] # Indices of notches of polygon (refer to list "allVertices")
        self.orientation = -1 # Direction in which MP5 is running. Single integer: 1 for ccw, -1 for cw
        self.startVertex = myVertex(0,0)
        
        while self.P.nPoints() > 0:
            
            allVertices = [] # List of all vertices of the polygon to be decomposed
            self.vertexIndexOfNextPoly = [] # List of indices of vertices of next convex polygon (refer to list "allVertices")
            
            if self.orientation != int(self.P.orientation()[0]):
                self.P = self.reversePolyOrientation(self.P)
                
            for i,v in enumerate(self.getVertices(self.P)):
                allVertices.append(v)
            self.indexOfVertex = None
            
            self.findInitialVertex(allVertices)
            
            if self.indexOfVertex == None:
                if len(self.notchVertices) == 0:
                    if len(self.holeList) > 0:
                        for i,hole in enumerate(self.holeList):
                            if self.P.covers(hole):
                                self.indexOfVertex = 1
                                self.vertexIndexOfNextPoly.append(0)
                                self.vertexIndexOfNextPoly.append(1)
                                break
                    else:
                        self.listOfConvexPoly.append(self.P)
                        self.P = self.P - self.P
                        
                        
                else:
                    #print 'Turn'
                    self.notchVertices = []
                    self.orientation = self.orientation*-1
                    #raw_input()
                    
            if self.indexOfVertex != None:        
                findNextPolygon, initialVertex, holeIndex, vertexIndex = self.checkNextPoly(allVertices)
                
                if findNextPolygon == False:
                    self.notchVertices.append(initialVertex)
                elif findNextPolygon == 'intersec':
                    #print "Try to merge hole!"
                    #print
                    self.notchVertices = []
                    self.mergeHole(allVertices,initialVertex,holeIndex,vertexIndex)
                    # self.drawPoly([self.P],'P')
                    
                    #raw_input()
                else:
                    
                    #print 'Find one polygon'
                    #print
                    self.notchVertices = []
                    #self.P = self.P - self.listOfConvexPoly[-1]
                    self.P = self.removeContour(self.listOfConvexPoly[-1])
                    #print self.P
                    #print
                    #self.drawPoly([self.P],'P')
                    #self.drawPoly(self.listOfConvexPoly,'Result')
                    
                    #raw_input()            
        
    
            
        #self.drawPoly(self.listOfConvexPoly,'Result')
        return self.listOfConvexPoly
        #print "Mission Complete"
        
    def removeContour(self,contour):
        pt1 = contour[0][0]
        pt2 = contour[0][1]
        
        points = Polygon.Utils.pointList(self.P)
        
        # make sure the contour is in same orientation with polygon
        for i,pt in enumerate(points):
            if pt[0] == pt1[0] and pt[1] == pt1[1]:
                if points[i-1][0] == pt2[0] and points[i-1][1] == pt2[1]:
                    contour[0].reverse()
                    break
                    
        # remove the contour from polygon
        pt1 = contour[0][0]
        pt2 = contour[0][1]
        for i,pt in enumerate(points):
            
            if pt[0] == pt1[0] and pt[1] == pt1[1] and points[i+1-len(points)][0] == pt2[0] and points[i+1-len(points)][1] == pt2[1]:
                startIndex = i+1
                endIndex = i+len(contour[0])-1
                length = len(points)
                
                if startIndex > length-1:
                    del points[startIndex-length:endIndex-length]
                else:
                    if endIndex > length-1:
                        del points[startIndex:length]
                        del points[0:endIndex-length]
                    else:
                        del points[startIndex:endIndex]
                
                # remove the duplicated points if they are next to each other
                removeList = []
                for i,pt in enumerate(points):
                    if i < len(points)-1:
                        nextPoint = points[i+1]
                    else:
                        nextPoint = points[0]
                    if (pt[0]==nextPoint[0]) and (pt[1]==nextPoint[1]):
                        removeList.append(pt)
                #print 'removing...............',removeList
                map(points.remove,removeList)
                return Polygon.Polygon(points)    
            
        
    def mergeHole(self,allVertices,initialIndex,holeIndex,vertexIndex):
        hole = self.holeList[holeIndex]
        del self.holeList[holeIndex]
       
            
        holePoints = []
        for i,v in enumerate(self.getVertices(hole)):
            holePoints.append(v)
        
        points = tuple([(p.x,p.y) for p in holePoints])
        #print points
        #print 
        mergePoints = holePoints[vertexIndex:-1] + [holePoints[-1]] + holePoints[0:vertexIndex+1]
        
        points = tuple([(p.x,p.y) for p in mergePoints])
        #print points
        #print
        
        
        if self.orientation != int(hole.orientation()[0]):
            mergePoints.reverse()
        mergePoints = [allVertices[initialIndex]] + mergePoints
        
        points = tuple([(p.x,p.y) for p in mergePoints])
        #print points
                 
        for pt in mergePoints:
            allVertices.insert(initialIndex+1,pt) 
        
        points = tuple([(vertex.x,vertex.y) for vertex in allVertices])
        self.P = Polygon.Polygon(points)
        #self.drawPoly([self.P],'P')
        #raw_input()                  
        
    def reversePolyOrientation(self,poly):
        
        points = Polygon.Utils.pointList(poly)
        points.reverse()
        return Polygon.Polygon(points)
        
    def linePolyIntersection(self,poly,vertexA,vertexB,boundaryPoly):
        endPointList = []
        #print "Main Line (", vertexA.x,",",vertexA.y,") and (",vertexB.x,",",vertexB.y,")"
        for j,face in enumerate(self.getFaces(poly)):
            #print "Checking (", face[0].x,",",face[0].y,") and (",face[1].x,",",face[1].y,")"
            intersec, pt1, pt2 = self.lineLineIntersection(vertexA,vertexB,face[0],face[1])
            #print intersec
            if intersec:
                if pt1 not in endPointList and boundaryPoly.isInside(pt1.x,pt1.y):
                    endPointList.append(pt1)
                if pt2 not in endPointList and boundaryPoly.isInside(pt2.x,pt2.y):
                    endPointList.append(pt2)

        if len(endPointList) == 0:
            return False, None
        else:
            closestPoint = endPointList[0]
            distance = self.pointPointDistance(closestPoint,vertexA)
            for i,pt in enumerate(endPointList):
                if distance > self.pointPointDistance(endPointList[i],vertexA):
                    closestPoint = endPointList[i]
                    distance = self.pointPointDistance(closestPoint,vertexA)
                    
            return True, closestPoint
                    
            
            
    def pointPointDistance(self,pt1,pt2):
    
        return math.sqrt((pt1.x-pt2.x)**2+(pt1.y-pt2.y)**2)
                    
            
            
    def lineLineIntersection(self,l1p1,l1p2,l2p1,l2p2):
        if (min(l1p1.x,l1p2.x)>=max(l2p1.x,l2p2.x)) or (max(l1p1.x,l1p2.x)<=min(l2p1.x,l2p2.x)) \
        or (min(l1p1.y,l1p2.y)>=max(l2p1.y,l2p2.y)) or (max(l1p1.y,l1p2.y)<=min(l2p1.y,l2p2.y)):
            return False, None, None
        else:
            x1 = l1p1.x
            x2 = l1p2.x
            y1 = l1p1.y 
            y2 = l1p2.y
            if ((y1-y2)/(x1-x2)*(l2p1.x-x2)+y2 < l2p1.y and (y1-y2)/(x1-x2)*(l2p2.x-x2)+y2 > l2p2.y) \
            or ((y1-y2)/(x1-x2)*(l2p1.x-x2)+y2 > l2p1.y and (y1-y2)/(x1-x2)*(l2p2.x-x2)+y2 < l2p2.y):
                return True, l2p1, l2p2
            else:
                return False, None, None
                
    def checkNextPoly(self,allVertices):
        initialIndex = self.indexInRange(self.indexOfVertex-1)
        index1 = self.indexInRange(self.indexOfVertex-1)
        index2 = self.indexInRange(self.indexOfVertex)
        index3 = self.indexInRange(self.indexOfVertex+1)
        
            
        while self.indexOfVertex < initialIndex + self.P.nPoints() -1 \
        and self.calcAngle(allVertices[index1],allVertices[index2],allVertices[index3]) \
        and self.calcAngle(allVertices[index2],allVertices[index3],allVertices[self.vertexIndexOfNextPoly[0]]) \
        and self.calcAngle(allVertices[index3],allVertices[self.vertexIndexOfNextPoly[0]],allVertices[self.vertexIndexOfNextPoly[1]]) \
        and (not self.checkPointInside(allVertices)):
            
            self.vertexIndexOfNextPoly.append(index3)
            self.startVertex = allVertices[index3]
            self.indexOfVertex = self.indexOfVertex + 1
            index1 = self.indexInRange(self.indexOfVertex-1)
            index2 = self.indexInRange(self.indexOfVertex)
            index3 = self.indexInRange(self.indexOfVertex+1)



            
            
        if len(self.vertexIndexOfNextPoly) > 2:
            
            #self.vertexIndexOfNextPoly.reverse()
            points = tuple([(vertex.x,vertex.y) for vertex in [allVertices[index] for index in self.vertexIndexOfNextPoly]])
            #print points
            poly = Polygon.Polygon(points)
            
            if len(self.holeList) > 0:
                holeIndex = None
                closestPoint = allVertices[self.vertexIndexOfNextPoly[-1]]
                
                for i,hole in enumerate(self.holeList): 
                    cover = poly.covers(hole)
                    if cover:
                        # The next convex polygon cover one of the holes
                        distance = None
                        # find the point on the hole that is the closest to the first vertex of the next polygon
                        for pt in self.getVertices(hole):
                            if distance == None:
                                closestPoint = pt
                                distance = self.pointPointDistance(pt,allVertices[self.vertexIndexOfNextPoly[0]])
                            else:
                                if distance > self.pointPointDistance(pt,allVertices[self.vertexIndexOfNextPoly[0]]):
                                    closestPoint = pt
                                    distance = self.pointPointDistance(pt,allVertices[self.vertexIndexOfNextPoly[0]])
                        holeIndex = i
                        break
                
                intersec = True
                # Check whether the new generated boundary intersects any holes or not
                
                while intersec:

                    for i,hole in enumerate(self.holeList):
                        intersec, temp = self.linePolyIntersection(hole,allVertices[self.vertexIndexOfNextPoly[0]],closestPoint,poly)
                        if intersec:
                            #print "Find one intersection!"
                            #print "The closest point is (",temp.x,",",temp.y,")"
                            #raw_input()
                            closestPoint = temp
                            #print closestPoint.x
                            holeIndex = i
                            break
                                                    
                if holeIndex == None:
                    self.listOfConvexPoly.append(poly)
                    return True,initialIndex,None,None
                else:
                    for i,v in enumerate(self.getVertices(self.holeList[holeIndex])):
                        
                        if v.x ==closestPoint.x and v.y==closestPoint.y:
                            #print "Find closest point"
                            #raw_input()
                            return 'intersec', initialIndex, holeIndex, i

            else:
                self.listOfConvexPoly.append(poly)
                return True,initialIndex,None,None
                    
            
            
        else:
            return False,initialIndex,None,None
            
        
        
            
    def checkPointInside(self,allVertices):
        """
        True:   There is at least one vertex inside the polygon generated
        False:  There is no vertex inside
        """
        inside = False
        self.vertexIndexOfNextPoly.append(self.indexInRange(self.indexOfVertex+1))

        points = tuple([(vertex.x,vertex.y) for vertex in [allVertices[index] for index in self.vertexIndexOfNextPoly]])
        p = Polygon.Polygon(points)
        
        
        for v in allVertices:
            onEdge = False
            for pt in [allVertices[index] for index in self.vertexIndexOfNextPoly]:
                if v.x == pt.x and v.y == pt.y:
                    onEdge = True
                    break
            if (not onEdge) and p.isInside(v.x,v.y):
                inside = True
                break
        
        del self.vertexIndexOfNextPoly[-1]
        return inside
                
    def indexInRange(self,index):
        while index  >= self.P.nPoints()-1:
            index = index - self.P.nPoints()
        while index <= -self.P.nPoints():
            index = index + self.P.nPoints()
        return index
    
    def findInitialVertex(self,allVertices):
        """
        Find notch vertex to start
        
        allVertices:    List of all vertices of the polygon to be decomposed
        """
        
        # TODO start searching from the last point of the last polygon
        i = 0
        for p, v in enumerate(allVertices):
            #print v.x
            if v.x == self.startVertex.x and v.y == self.startVertex.y:
                i = p
                break
                
        initial_i = i   
        while i < initial_i + len(allVertices)-1:
            j=self.indexInRange(i)
            if ((j not in self.notchVertices) and (j+len(allVertices) not in self.notchVertices) and (j-len(allVertices) not in self.notchVertices))\
            and (not self.calcAngle(allVertices[j-1],allVertices[j],allVertices[j+1])):
                self.indexOfVertex = self.indexInRange(j+1)
                self.vertexIndexOfNextPoly.append(j)
                self.vertexIndexOfNextPoly.append(self.indexOfVertex)
                #print 'Find Initial Vertex!', allVertices[j].x,allVertices[j].y
                #print
                #raw_input()
                break
            else:
                i = i +1

                
    def getVertices(self,poly):
        for pt in Polygon.Utils.pointList(poly):
            yield myVertex(pt[0],pt[1])
            
            
    def getFaces(self,poly):
        """
        Wrapper function to allow for iteration over faces of regions.
        A face is a tuple of the two points (in absolute coordinates) that make up the face,
        sorted so that a given face is defined uniquely.

        FIXME: Make sure we take advantage of this uniqueness elsewhere; I think we check
        too many conditions sometimes
        """

        lastPt = None
        for pt in self.getVertices(poly):

            thisPt = myVertex(pt.x, pt.y)

            if lastPt != None:
                yield tuple(sorted((lastPt, thisPt)))
            else:
                firstPt = thisPt

            lastPt = thisPt

        yield tuple(sorted((lastPt, firstPt))) # Closing face
        
        
    def calcAngle(self,a,b,c):
        """
        A function that calculates the angle between 0 and 2*pi rad swept by a counterclockwisr rotation
        from line segment ba to bc
        
        a,b,c are vertices
        Return True when the angle is smaller than or equals pi  
        Return False when the angle is larger than pi  .
        """
        if self.P.orientation()[0] > 0.0:
            temp = a
            a = c
            c = temp

        angleBA = math.atan2((a.y-b.y),(a.x-b.x))
        angleBC = math.atan2((c.y-b.y),(c.x-b.x))
        if (angleBA > 0) and (angleBC < 0):
            angleBC = angleBC+2*math.pi
        
        
        if (angleBA<angleBC) and (angleBA + math.pi >= angleBC):
            return True
        else:
            return False
            
    def drawPoly(self,polyList,fileName):
        polyList.insert(0,self.background)
        Polygon.IO.writeSVG('/home/jim/Desktop/'+fileName+'.svg', polyList)
        polyList.pop(0) # make sure the back ground won't be added to the actual list

    
    
            
