#!/usr/bin/env python

import math,re, os, random
import Polygon, Polygon.IO, Polygon.Utils
import project
from regions import *
import itertools
import decomposition

Polygon.setTolerance(0.1)

class parseLP:
    """
    A parser to parse the locative prepositions in specification
    """
    def __init__(self):
        
        pass

    def main(self,argv):
        """ Main function; run automatically when called from command-line """

        spec_file = argv
        self.regionNear = []
        self.regionBetween = []
        defaultNearDistance = 50

        # load data
        self.proj = project.Project()
        self.proj.setSilent(True)
        self.proj.loadProject(spec_file)
        
        if self.proj.compile_options['decompose']:
            # we will do the decomposition
            # Look for a defined boundary region, and set it aside if available
            self.boundaryRegion = None
            for region in self.proj.rfi.regions:
                if region.name.lower() == 'boundary':
                    self.boundaryRegion = region
                    self.proj.rfi.regions.remove(region)
                    break
        
            # TODO: If not defined, use the minimum bounding polygon by default
            if self.boundaryRegion is None:
                print "ERROR: You need to define a boundary region (just create a region named 'boundary' in RegionEditor)"
                return

            # turn list of string into one string
            spec = "\n".join([line for line in self.proj.spec_data['SPECIFICATION']['Spec'] if not line.startswith("#")])
            
            # get all regions that need to find "region near"
            # the items in the list are tuple with region name and distance from the region boundary, default value is 50
            for m in re.finditer(r'near (?P<rA>\w+)', spec):
                if m.group("rA") not in self.regionNear:
                    self.regionNear.append((m.group("rA"),50))
                    
            # find "within distance from a region" is just special case of find "region near"
            for m in re.finditer(r'within (?P<dist>\d+) (from|of) (?P<rA>\w+)', spec):
                if m.group("rA") not in self.regionNear:
                    self.regionNear.append((m.group("rA"),int(m.group("dist"))))
                    
            # get all regions that need to find "region between"
            # the items in the list are tuple with two region names       
            for m in re.finditer(r'between (?P<rA>\w+) and (?P<rB>\w+)', spec):
                if (m.group("rA"),m.group("rB")) not in self.regionBetween and (m.group("rB"),m.group("rA")) not in self.regionBetween:
                    self.regionBetween.append((m.group("rA"),m.group("rB")))
                
            # generate new regions
            self.generateNewRegion()
            # break the overlapped regions into seperated parts
            self.checkOverLapping()
            # remove small regions
            self.removeSmallRegions()
            
            # decompose any regions with holes or are concave
            if self.proj.compile_options['convexify']:
                self.decomp()
            # store the regionMapping data to project file
            self.proj.regionMapping = self.newPolysMap
            # save the regions into new region file
            fileName = self.proj.getFilenamePrefix()+'_decomposed.regions'
            self.saveRegions(fileName)
        else:
            # if decompose option is disabled, we skip the following step but keep the mapping
            self.newPolysMap = {} # {"nameOfRegion":a list holds name of portion}
            for region in self.proj.rfi.regions:
                self.newPolysMap[region.name] = [region.name]
            # store the regionMapping data to project file
            self.proj.regionMapping = self.newPolysMap
            fileName = self.proj.getFilenamePrefix()+'_decomposed.regions'
            self.proj.rfi.writeFile(fileName)

        
    def generateNewRegion(self):
        """
        Generate new regions for locative prepositions
        """
        
        # regions related with "near/within" preposition
        for (regionName,dist) in self.regionNear:
            for region in self.proj.rfi.regions:
                if region.name == regionName:
                    oldRegion = region
            newRegion = oldRegion.findRegionNear(dist,mode="overEstimate",name='near$'+regionName+'$'+str(dist))
            self.proj.rfi.regions.append(newRegion)
            
            
        # regions related with "between" preposition
        for (regionNameA,regionNameB) in self.regionBetween:

            for region in self.proj.rfi.regions:
                if region.name == regionNameA:
                    regionA = region
                elif region.name == regionNameB:
                    regionB = region
                    
            newRegion = findRegionBetween(regionA,regionB,name='between$'+regionNameA+'$and$'+regionNameB+"$")
            self.proj.rfi.regions.append(newRegion)            
            
    def checkOverLapping(self):
        """
        Check if and regions overlap each other
        Break the ones that overlap into portions that don't overlap
        """
        oldRegionNames=[]
        self.oldPolys = {} # {"nameOfRegion":polygon of that region}
        self.newPolysMap = {} # {"nameOfRegion":a list holds name of portion}
        self.portionOfRegion = {} # {"nameOfPortion":polygon of that portion}
        for region in self.proj.rfi.regions:
            points = [(pt.x,pt.y) for pt in region.getPoints()]
            poly = Polygon.Polygon(points)
            self.oldPolys[region.name] = self.intAllPoints(poly)
            self.newPolysMap[region.name] = []
        oldRegionNames = sorted(self.oldPolys.keys())
        self.newPolysMap['others'] = [] # parts out side of all regions
        
        # set up a iterator of lists of boolean value (0/1) for finding overlapping regions
        # each item is corrsponding to one possible overlapping
        # each boolean value is corresponding to one region
        boolList = itertools.product([0,1],repeat=len(oldRegionNames))
        
        self.count = 1 # for naming the portion
        # break the overlapping regions
        for expr in boolList:
            tempRegionList = []
            result = self.intAllPoints(Polygon.Polygon([(pt.x,pt.y) for pt in self.boundaryRegion.getPoints()])) # starts with the boundary region
            for i,item in enumerate(expr):
                if item == 1:
                    # when the region is included
                    result = result & self.oldPolys[oldRegionNames[i]]
                    tempRegionList.append(oldRegionNames[i])
                else:
                    # when the region is excluded
                    result = result - self.oldPolys[oldRegionNames[i]]
                   
            if result.nPoints()>0:
                # there is a portion of region left
                holeList = []
                nonHoleList = []
                for i,contour in enumerate(result):
                    if not result.isHole(i):
                        nonHoleList.append(Polygon.Polygon(result[i]))
                    else:
                        holeList.append(Polygon.Polygon(result[i]))
                for nonHolePoly in nonHoleList:
                    polyWithoutOverlapNode = self.decomposeWithOverlappingPoint(nonHolePoly)
                    for poly in polyWithoutOverlapNode:
                        portionName = 'p'+str(self.count)
                        p = self.intAllPoints(poly)
                        for hole in holeList:
                            p = p - self.intAllPoints(hole)
                        self.portionOfRegion[portionName] = p
                        if len(tempRegionList) == 0:
                            self.newPolysMap['others'].append(portionName)
                        else:
                            for regionName in tempRegionList:
                                # update the maping dictionary
                                self.newPolysMap[regionName].append(portionName)
                    
                        self.count = self.count + 1

        
    def decomposeWithOverlappingPoint(self,polygon):
        """
        When there are points overlapping each other in a given polygon
        First decompose this polygon into sub-polygons at the overlapping point
        """
        
        # recursively break the polygon at any overlap point into two polygons until no overlap points are found
        # here we are sure there is only one contour in the given polygon

        
        ptDic = {}
        overlapPtIndex = None
        # look for overlap point and stop when one is found
        for i,pt in enumerate(polygon[0]):
            if pt not in ptDic:
                ptDic[pt]=[i]
            else:
                ptDic[pt].append(i) 
                overlapPtIndex = ptDic[pt]
                break

        if overlapPtIndex:
            polyWithoutOverlapNode = []
            # break the polygon into sub-polygons
            newPoly = Polygon.Polygon(polygon[0][overlapPtIndex[0]:overlapPtIndex[1]])
            polyWithoutOverlapNode.extend(self.decomposeWithOverlappingPoint(newPoly))
            reducedPoly = Polygon.Polygon(decomposition.removeDuplicatePoints((polygon-newPoly)[0]))
            polyWithoutOverlapNode.extend(self.decomposeWithOverlappingPoint(reducedPoly))
        else:
            # no overlap point is found
            return [polygon]

        return polyWithoutOverlapNode

    def decomp(self):
        """
        Decompose the region with holes or are concave
        """
        tempDic = {} # temporary variable for storing polygon
                     # will be merged at the end to self.portionOfRegion

        for nameOfPortion,poly in self.portionOfRegion.iteritems():
            result = [] # result list of polygon from decomposition
            if len(poly)>1:
                # the polygon contains holes
                holes = [] # list holds polygon stands for holes
                for i,contour in enumerate(poly):
                    if poly.isHole(i):
                        holes.append(Polygon.Polygon(poly[i]))
                    else:
                        newPoly = Polygon.Polygon(poly[i])
                        
                de = decomposition.decomposition(newPoly,holes)
                result = de.MP5()
            else:
                # if the polygon doesn't have any hole, decompose it if it is concave, 
                # nothing will be done if it is convex
                de = decomposition.decomposition(poly)
                result = de.MP5()
            
            if len(result)>1:
                # the region is decomposed to smaller parts
                newPortionName=[]
                # add the new portions                
                for item in result:
                    portionName = 'p'+str(self.count)
                    newPortionName.append(portionName)
                    tempDic[portionName] = item
                    self.count = self.count + 1
                
                # update the mapping dictionary
                for nameOfRegion,portionList in self.newPolysMap.iteritems():
                    if nameOfPortion in portionList:
                        self.newPolysMap[nameOfRegion].remove(nameOfPortion)
                        self.newPolysMap[nameOfRegion].extend(newPortionName)
                
            else:
                tempDic[nameOfPortion] = Polygon.Polygon(result[0])
        self.portionOfRegion = tempDic
                        
    def drawAllPortions(self):
        """
        Output a drawing of all the polygons that stored in self.portionOfRegion, for debug purpose
        """
        if len(self.portionOfRegion)==0:
            print "There is no polygon stored."
            print
            return
        
        polyList = []    
        for nameOfPortion,poly in self.portionOfRegion.iteritems():
            polyList.append(poly)
        Polygon.IO.writeSVG('/home/cornell/Desktop/ltlmop-google/allPortions.svg', polyList)
        
    def removeSmallRegions(self):
        """
        A function to remove small region
        """
        tolerance=0.0000001
        
        # find the area of largest regions
        area = 0
        for nameOfPortion,poly in self.portionOfRegion.iteritems():
            if area<poly.area():
                area = poly.area()
                
        # remove small regions
        smallRegion = []
        
        for nameOfPortion,poly in self.portionOfRegion.iteritems():
            if poly.area()<tolerance*area:
                smallRegion.append(nameOfPortion)
                for nameOfRegion, portionList in self.newPolysMap.iteritems():
                    if nameOfPortion in portionList:
                        self.newPolysMap[nameOfRegion].remove(nameOfPortion)
                        
        for region in smallRegion:
            #print "remove"+region
            del self.portionOfRegion[region]            
            
    def intAllPoints(self,poly):
        """
        Function that turn all point coordinates into integer
        Return a new polygon
        """
        return Polygon.Utils.prunePoints(Polygon.Polygon([(int(pt[0]),int(pt[1])) for pt in poly[0]]))
        
    def saveRegions(self, fileName=''):
        """
        Save the region data into a new region file
        """
        # use the existing rfi as to start
        
        # the only different data is regions
        self.proj.rfi.regions = []
        for nameOfPortion,poly in self.portionOfRegion.iteritems():
            newRegion                   = Region()
            newRegion.name              = nameOfPortion
            newRegion.color             = Color()
            newRegion.color.SetFromName(random.choice(['RED','ORANGE','YELLOW','GREEN','BLUE','PURPLE']))
            for i,ct in enumerate(poly):
                if poly.isHole(i):
                    newRegion.holeList.append([Point(*x) for x in Polygon.Utils.pointList(Polygon.Polygon(poly[i]))])
                else:  
                    newRegion.pointArray = [Point(*x) for x in Polygon.Utils.pointList(Polygon.Polygon(poly[i]))]
            newRegion.alignmentPoints   = [False] * len([x for x in newRegion.getPoints()])    
            newRegion.recalcBoundingBox()
            
            if newRegion.getDirection() == dir_CCW:
                newRegion.pointArray.reverse()
                
            self.proj.rfi.regions.append(newRegion)
        
        # Giant loop!
        for obj1 in self.proj.rfi.regions:
            for obj2 in self.proj.rfi.regions:
                self.proj.rfi.splitSubfaces(obj1, obj2)

        self.proj.rfi.recalcAdjacency()
        self.proj.rfi.writeFile(fileName)
        

