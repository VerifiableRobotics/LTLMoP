#!/usr/bin/env python

import math,re, os, random
import Polygon, Polygon.IO, Polygon.Utils
import wx
import project
import regions
import itertools
import decomposition

Polygon.setTolerance(0.1)


#########################
# MAIN EXECUTION THREAD #
#########################

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
        self.proj.loadProject(spec_file)
        
        # store the boundary region data and remove it from the region list
        for region in self.proj.rfi.regions:
            if region.name.lower() == 'boundary':
                self.boundaryRegion = region
                self.proj.rfi.regions.remove(region)

        # turn list of string into one string
        spec = "\n".join(self.proj.spec_data['SPECIFICATION']['Spec'])
        
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
        self.decomp()

        # store the regionMapping data to project file
        self.proj.regionMapping = self.newPolysMap
        # save the regions into new region file
        fileName = self.proj.getFilenamePrefix()+'_decomposed.regions'
        self.saveRegions(fileName)
        

        
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
                    
            newRegion = regions.findRegionBetween(regionA,regionB,name='between$'+regionNameA+'$and$'+regionNameB+"$")
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
                for poly in nonHoleList:
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
                
                # update the mapping dictionary
                for nameOfRegion,portionList in self.newPolysMap.iteritems():
                    if nameOfPortion in portionList:
                        self.newPolysMap[nameOfRegion].remove(nameOfPortion)
                        
                        # add the new portions                
                        for item in result:
                            portionName = 'p'+str(self.count)
                            tempDic[portionName] = item
                            self.newPolysMap[nameOfRegion].append(portionName)
                            self.count = self.count + 1
                
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
        Polygon.IO.writeSVG('/home/jim/Desktop/allPortions.svg', polyList)
        
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
            print "remove"+region
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
            newRegion                   = regions.Region()
            newRegion.name              = nameOfPortion
            newRegion.color             = wx.Colour()
            newRegion.color.SetFromName(random.choice(['RED','ORANGE','YELLOW','GREEN','BLUE','PURPLE']))
            newRegion.pointArray        = [wx.Point(*x) for x in Polygon.Utils.pointList(poly)]
            newRegion.alignmentPoints   = [False] * len([x for x in newRegion.getPoints()])    
            newRegion.recalcBoundingBox()
            
            if newRegion.getDirection() == regions.dir_CCW:
                newRegion.pointArray.reverse()
                
            self.proj.rfi.regions.append(newRegion)
        
        self.proj.rfi.recalcAdjacency()
        self.proj.rfi.writeFile(fileName)
        
