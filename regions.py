#!/usr/bin/env python

""" ===========================
    regions.py - Regions Module
    ===========================

    A simple module that defines a class for describing and manipulating
    rectangular and polygonal regions.

    This is completely free software; please feel free to adapt or use this in
    any way you like.

    Some parts extracted from pySketch by Erik Westra (ewestra@wave.co.nz)
"""

# TODO: remove wx dependency!!
# TODO: store regions with absolute coordinates, not x/y-pos + relative!!

import wx   # Unfortunately necessary for wx.Point(), wx.Size(), and color stuff
import fileMethods
import re, random, math

############################################################

# Direction IDs (for defining convexity):
[dir_CW, dir_CCW, dir_CONCAVE] = range(3)

# Region type IDs:
[reg_RECT, reg_POLY] = range(2)

# Selection handle IDs:
[handle_NONE,           handle_TOP_LEFT,        handle_TOP_RIGHT,
 handle_BOTTOM_RIGHT,   handle_BOTTOM_LEFT] = range(-1, 4)

############################################################

class RegionFileInterface:
    """
    A wrapper class for handling collections of regions and associated metadata.

    Overview of exposed data structures:

        - background (string): relative path of background image file
        - regions (list): list of Region objects, with properties defined below
        - transitions (list of lists):
            * key1 = Region object index
            * key2 = Region object index
            * values = Lists of faces connecting the two regions
        - thumb (string): relative path of an image file that shows the regions overlayed 
                          on the background image, made by taking a screenshot
    """

    def __init__(self, background="None", regions=[], transitions=None, thumb=None):
        self.background = background
        self.regions = regions
        self.transitions = transitions
        self.thumb = thumb

    def setToDefaultName(self, region):
        if region.name is '':
            # Find an available name
            region.name = 'r' + str(self.getNextAvailableRegionNumber())

    def indexOfRegionWithName(self, name):
        for i, region in enumerate(self.regions):
            if region.name == name:
                return i
        print 'WARNING: Region "' + name + '" not found.'
        return -1

    def getCalibrationPoints(self):
        for region in self.regions:
            for index, bool in enumerate(region.alignmentPoints):
                if bool:
                    pts = [x for x in region.getPoints()]
                    yield([region.name, index, pts[index].x, pts[index].y])

    def getNextAvailableRegionNumber(self):
        """
        Look for the smallest region name of form r1, r2, ... available.
        """

        # First get all region names of the form we're interested in
        nums = []
        p = re.compile(r"^r(?P<num>\d+)$")
        
        for region in self.regions:
            m = p.match(region.name)
            if not m: continue
            nums.append(int(m.group('num')))
        
        nums.sort()

        last = 0
        for num in nums:
            if num == last + 1:
                last = num
            elif num == last:
                #print "Warning: Multiple regions with name 'r%d'." % num
                continue
            else:
                # There's a gap we can fill
                return last + 1
        
        # Everything was full, so let's just append to the end
        return last + 1

    def writeFile(self, filename):
        """
        File format is described inside the comments variable. 
        """

        comments = {"FILE_HEADER":"This is a region definition file for the LTLMoP toolkit.\n" +
                                  "Format details are described at the beginning of each section below.\n" +
                                  "Note that all values are separated by *tabs*.",
                    "Background": "Relative path of background image file",
                    "Regions": "Name, Type, Pos X, Pos Y, Width, Height, Color R, Color G, Color B, Vertices (x1, y1, x2, y2, ...)",
                    "Transitions": "Region 1 Name, Region 2 Name, Bidirectional transition faces (face1_x1, face1_y1, face1_x2, face1_y2, face2_x1, ...)",
                    "Thumbnail": "Relative path of image file that has region shapes overlayed on background image",
                    "CalibrationPoints": "Vertices to use for map calibration: (vertex_region_name, vertex_index)"}    
    
        regionData = []
        for i, region in enumerate(self.regions): 
            regionData.append("\t".join(map(str, region.getData(withAlignment=False)))) 
       
        transitionData = []
        for region1, destinations in enumerate(self.transitions):
            # Note: We are assuming all transitions are bidirectional so we only have to include
            # the parts of the adjacency matrix above the diagonal
            for region2, faces in enumerate(destinations[region1+1:]):
                if faces == []: continue    # No transition between these regions, so skip

                faceData = []
                for face in faces:
                    faceData.extend([face[0][0], face[0][1], face[1][0], face[1][1]])

                transitionData.append("\t".join([self.regions[region1].name,
                                                 self.regions[region1 + 1 + region2].name] +
                                                 map(str, faceData)))

        calibPoints = []
        for region in self.regions:
            for index, bool in enumerate(region.alignmentPoints):
                if bool:
                    calibPoints.append("\t".join([region.name, str(index)]))

        calibPointsStr = "\t".join(calibPoints)

        data = {"Background": self.background,
                "Regions": regionData,
                "Transitions": transitionData,
                "Thumbnail": self.thumb,
                "CalibrationPoints": calibPoints}

        fileMethods.writeToFile(filename, data, comments)

        return True

    def readFile(self, filename):
        """
        For file format information, refer to writeFile() above.
        """

        data = fileMethods.readFromFile(filename)

        if data is None:
            return False

        try:
            self.background = data["Background"][0]
        except KeyError:
            self.background = "None"

        self.thumb = data["Thumbnail"][0]

        self.regions = []
        for region in data["Regions"]:
            regionData = region.split("\t");
            newRegion = Region()
            newRegion.setData(regionData, withAlignment=False)    
            newRegion.alignmentPoints = [False] * len([x for x in newRegion.getPoints()])
            self.regions.append(newRegion)

        # Make an empty adjacency matrix of size (# of regions) x (# of regions)
        self.transitions = [[[] for j in range(len(self.regions))] for i in range(len(self.regions))]
        for transition in data["Transitions"]:
            transData = transition.split("\t");
            region1 = self.indexOfRegionWithName(transData[0])
            region2 = self.indexOfRegionWithName(transData[1])
            faces = []
            for i in range(2, len(transData), 4):
                p1 = wx.Point(int(transData[i]), int(transData[i+1]))
                p2 = wx.Point(int(transData[i+2]), int(transData[i+3]))
                faces.append(tuple(sorted((p1, p2))))
                
            # During adjacency matrix reconstruction, we'll mirror over the diagonal
            self.transitions[region1][region2] = faces
            self.transitions[region2][region1] = faces

        if "CalibrationPoints" in data:
            for point in data["CalibrationPoints"]:
                [name, index] = point.split("\t")
                self.regions[self.indexOfRegionWithName(name)].alignmentPoints[int(index)] = True
            
        return True
   
############################################################
 
class Region:
    """ A rectangular or polygonal region, defined by the following properties:

            - 'name'          Region name
            - 'type'          What type of region this is (rect or poly)
            - 'position'      The position of the object within the document 
                              (i.e., the top-left corner of the region's bounding-box)
            - 'size'          The size of the object's bounding box
            - 'color'         Color to use for drawing the region
            - 'pointArray'    Polygon points, relative to region position (stored in CW order)
            - 'alignmentPoints'  True/False array indicating for each vertex whether or not to use it as an alignment point

        NOTE: All coordinates are stored internally with (0,0) at the top left.
              X increases to right, and Y increases downwards.
    """

    def __init__(self, type=reg_POLY, position=wx.Point(0, 0), size=wx.Size(0, 0),
                 color=None, points=[], name=''):

        if color is None:
            # Give a random color
            color = wx.Colour()
            color.SetFromName(random.choice(['RED','ORANGE','YELLOW','GREEN','BLUE','PURPLE']))

        self.name              = name
        self.type              = type
        self.position          = position
        self.size              = size
        self.color             = color
        self.pointArray        = points
        self.alignmentPoints   = [False] * len([x for x in self.getPoints()])

    # =================================
    # == Region Manipulation Methods ==
    # =================================

    def addPoint(self, point, index):
        """
        Insert a new point at a given index, converting object type if necessary
        """
        if self.type == reg_RECT:
            # Convert from rect to poly :(
            self.pointArray = [p for p in self.getPoints(relative=True)]
            self.type = reg_POLY
        self.pointArray.insert(index, point)
        self.alignmentPoints.insert(index, False)
        self.recalcBoundingBox()

    def removePoint(self, index):
        """
        Remove the point at the given index, converting object type if necessary
        """
        if self.type == reg_POLY and len(self.pointArray) <= 3:
            # Hey now, don't go making lines
            return False
        if self.type == reg_RECT:
            # Convert from rect to poly :(
            self.pointArray = [p for p in self.getPoints(relative=True)]
            self.type = reg_POLY
        self.pointArray.pop(index)
        self.alignmentPoints.pop(index)
        self.recalcBoundingBox()
        return True

    def recalcBoundingBox(self):
        """
        Recalculate the bounding box for our object
        """
        
        # Find how much our bounding box has shifted in relation to the old one
        for i, pt in enumerate(self.getPoints(relative=True)):
            if i == 0:
                # Set initial values
                topLeftX  = pt.x
                topLeftY  = pt.y
                botRightX = pt.x
                botRightY = pt.y
            else:
                # Check for any points that would expand our bounds
                if pt.x > botRightX:
                    botRightX = pt.x
                if pt.x < topLeftX:
                    topLeftX = pt.x
                if pt.y < topLeftY:
                    topLeftY = pt.y
                if pt.y > botRightY:
                    botRightY = pt.y
    
        if self.type == reg_POLY:    
            # Shift our vertices to align with the new bounding box
            self.pointArray = map(lambda x: x-wx.Point(topLeftX, topLeftY), self.pointArray)

        # Store the new bounding box
        self.position = self.position + wx.Point(topLeftX, topLeftY)
        self.size = wx.Size(botRightX - topLeftX, botRightY - topLeftY)


    # =============================
    # == Object Property Methods ==
    # =============================

    def getDirection(self):
        """
        Determine convexity/concavity and the order of the points stored in the pointArray.
        For details on the algorithm, please refer to:
                 http://local.wasp.uwa.edu.au/~pbourke/geometry/clockwise/index.html
        """

        if self.type == reg_RECT:
            # We define RECTs as CW
            return dir_CW

        last_cp = None
        for [i, pt] in enumerate(self.pointArray):
            # Be sure to account for looping around the edges of the vertex array
            prev = self.pointArray[(i - 1) % len(self.pointArray)]
            next = self.pointArray[(i + 1) % len(self.pointArray)]

            cp = (pt.x - prev.x)*(next.y - pt.y) - (pt.y - prev.y)*(next.x - pt.x)

            if cp != 0:
                if last_cp is not None and (cp > 0) != (last_cp > 0):
                    return dir_CONCAVE
                last_cp = cp

        if last_cp > 0:
            return dir_CW
        else:
            return dir_CCW

    def getData(self, withAlignment=True):
        """ Return a copy of the object's internal data.
            This is used for undo and to save this region to disk.
        """

        if self.type == reg_RECT:
            type = "rect"
        else:
            type = "poly"

        expandedPoints = []

        if self.type == reg_POLY:
            for i, pt in enumerate(self.pointArray):
                if withAlignment:
                    expandedPoints.extend([pt.x, pt.y, self.alignmentPoints[i]])
                else:
                    expandedPoints.extend([pt.x, pt.y])
        elif self.type == reg_RECT:
            if withAlignment:
                expandedPoints = self.alignmentPoints

        return [self.name, type,
                self.position.x, self.position.y,
                self.size.width, self.size.height,
                self.color.Red(),
                self.color.Green(),
                self.color.Blue()] + expandedPoints

    def setData(self, data, withAlignment=True):
        """ Set the object's internal data.

            'data' is a copy of the object's saved data, as returned by
            getData() above.  This is used for undo and to restore a 
            previously saved region.
        """
        self.name              = data[0]

        if data[1].lower() == "rect":
            self.type = reg_RECT
        else:
            self.type = reg_POLY

        self.position          = wx.Point(int(data[2]), int(data[3]))
        self.size              = wx.Size(int(data[4]), int(data[5]))
        self.color             = wx.Colour(red=int(data[6]),
                                         green=int(data[7]),
                                          blue=int(data[8]))
        if self.type == reg_POLY:
            self.pointArray = []
            if withAlignment:
                self.alignmentPoints = []
                for i in range(9, len(data), 3):
                    self.pointArray.append(wx.Point(int(data[i]), int(data[i+1])))
                    self.alignmentPoints.append(data[i+2])
            else:
                for i in range(9, len(data), 2):
                    self.pointArray.append(wx.Point(int(data[i]), int(data[i+1])))
        elif self.type == reg_RECT:
            if withAlignment:
                self.alignmentPoints = []
                for i in range(9, len(data), 1):
                    self.alignmentPoints.append(data[i])

    def getFaces(self):
        """
        Wrapper function to allow for iteration over faces of regions.
        A face is a tuple of the two points (in absolute coordinates) that make up the face,
        sorted so that a given face is defined uniquely.

        FIXME: Make sure we take advantage of this uniqueness elsewhere; I think we check
        too many conditions sometimes
        """

        lastPt = None
        for pt in self.getPoints():

            thisPt = (pt.x, pt.y)

            if lastPt != None:
                yield tuple(sorted((lastPt, thisPt)))
            else:
                firstPt = thisPt

            lastPt = thisPt

        yield tuple(sorted((lastPt, firstPt))) # Closing face
                
    def getPoints(self, relative=False):
        """
        Wrapper function to allow for iteration over the points of a region without
        worrying about whether it's a RECT or POLY. 
        """

        if relative:
            offset = wx.Point(0, 0)
        else:
            offset = self.position

        if self.type == reg_POLY:
            for pt in self.pointArray:
                yield offset + pt
        elif self.type == reg_RECT:
            for pt in [wx.Point(0, 0),
                       wx.Point(self.size.width, 0),
                       wx.Point(self.size.width, self.size.height),
                       wx.Point(0, self.size.height)]:
                yield offset + pt

    def getCenter(self):
        """
        Find the 'center' of a region
        """

        if self.type == reg_POLY:
            # Let's just average the points in the poly; that's easy
            asum = reduce(lambda a,b: a+b, self.pointArray)
            cx = asum.x / len(self.pointArray) + self.position.x
            cy = asum.y / len(self.pointArray) + self.position.y
        else:
            # Rectangles are even easier!
            cx = self.size.GetWidth()/2 + self.position.x
            cy = self.size.GetHeight()/2 + self.position.y
        
        return wx.Point(cx, cy)

    # =======================
    # == Selection Methods ==
    # =======================

    def objectContainsPoint(self, x, y):
        """ Returns True iff this object contains the given point.

            This is used to determine if the user clicked on the object.
        """

        # Firstly, ignore any points outside of the object's bounds.

        if x < self.position.x: return False
        if x > self.position.x + self.size.x: return False
        if y < self.position.y: return False
        if y > self.position.y + self.size.y: return False

        if self.type in [reg_RECT]:
            # Rectangles and text are easy -- they're always selected if the
            # point is within their bounds.
            return True

        # For polygons, we have to check whether the clicked point is
        # inside or outside the polygon.
        # The algorithm used here was taken from:
        # http://local.wasp.uwa.edu.au/~pbourke/geometry/insidepoly/

        sum = 0
        n = len(self.pointArray)
        for i in range(n):
            v1_y = self.pointArray[i].y-(y-self.position.y)
            v1_x = self.pointArray[i].x-(x-self.position.x)
            v2_y = self.pointArray[(i+1)%n].y-(y-self.position.y)
            v2_x = self.pointArray[(i+1)%n].x-(x-self.position.x)
            angle_v1 = math.atan2(v1_y, v1_x)
            angle_v2 = math.atan2(v2_y, v2_x)
            angle = angle_v2 - angle_v1
            while(angle > math.pi):
                angle -= 2 * math.pi
            while(angle < -math.pi):
                angle += 2 * math.pi
            sum += angle

        if abs(sum) < math.pi:
            return False
        else:
            return True


    def getSelectionHandleContainingPoint(self, x, y, boundFunc=None):
        """ Return the selection handle containing the given point, if any.

            We return one of the predefined selection handle ID codes (defined at top).
        """
        if boundFunc is None:
            boundFunc = self._pointInSelRect
        
        for index, pt in enumerate(self.getPoints()):
            if boundFunc(x, y, pt.x, pt.y):
                return index    # For rectangles, the index corresponds to the handle ID by definition
        return handle_NONE


    def objectWithinRect(self, x, y, width, height):
        """ Return True iff this object falls completely within the given rect.
        """
        if x          > self.position.x:                    return False
        if x + width  < self.position.x + self.size.width:  return False
        if y          > self.position.y:                    return False
        if y + height < self.position.y + self.size.height: return False
        return True

    # =====================
    # == Private Methods ==
    # =====================

    def _pointInSelRect(self, x, y, rX, rY):
        """ Return True iff (x, y) is within the selection handle at (rX, rY).
        """
        if   x < rX - 3: return False
        elif x > rX + 3: return False
        elif y < rY - 3: return False
        elif y > rY + 3: return False
        else:            return True


############################################################


def pointLineIntersection(pt1, pt2, test_pt):
    """
    Given two points (pt1, pt2), find the point on the line formed by those points that is nearest
    to test_pt and give the distance.
    """

    [x1, y1] = [pt1.x, pt1.y]
    [x2, y2] = [pt2.x, pt2.y]
    [xm, ym] = [test_pt.x, test_pt.y]
    if x1 == x2: # vertical line
        xi = x1
        yi = ym
        d = (xm - xi)**2
        on_segment = min(y1,y2) < yi < max(y1,y2)
    elif y1 == y2: # horizontal line
        yi = y1
        xi = xm
        d = (ym - yi)**2
        on_segment = min(x1,x2) < xi < max(x1,x2)
    else:
        c1 = 1.0*(y2 - y1)/(x2 - x1)
        c2 = 1.0*-(x2 - x1)/(y2 - y1)
        xi = (x1*c1 - xm*c2 - y1 + ym)/(c1 - c2)
        yi = y1 + c1*(xi - x1)
        d = (xm - xi)**2 + (ym - yi)**2
        on_segment = min(x1,x2) < xi < max(x1,x2) 

    if not on_segment:
        d = None

    return [d, wx.Point(xi, yi)]

