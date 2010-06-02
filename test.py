#!/usr/bin/env python

from regions import *
import Polygon, Polygon.IO, Polygon.Utils
import os


    regionNum=1

    rfi = RegionFileInterface()
    fileName=os.path.join(os.getcwd(),'examples/iros10/iros10.regions')

    rfi.readFile(fileName)
    newRegion=rfi.regions[regionNum].findRegionNear(100)

    backGround = Polygon.Polygon(((-300,-100),(1200,-100),(1200,1000),(-300,1000)))

    points = [x+rfi.regions[regionNum].position for x in rfi.regions[regionNum].pointArray]
    old = Polygon.Polygon(points)

    points = [x+newRegion.position for x in newRegion.pointArray]
    new = Polygon.Utils.convexHull((Polygon.Polygon(points)))

    print
    print rfi.regions[regionNum].pointArray
    print
    print
    print newRegion.pointArray
    print


    Polygon.IO.writeSVG('/home/jim/Desktop/test.svg', (backGround,new,old))

