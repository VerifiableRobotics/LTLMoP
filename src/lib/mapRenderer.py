import wx
import sys, os
from regions import *

#----------------------------------------------------------------------------

class DrawableRegion(Region):
    """ Extends the Region class to allow drawing.
    """

    # ============================
    # == Object Drawing Methods ==
    # ============================

    def draw(self, dc, pdc, selected, scale=1.0, showAlignmentPoints=True, highlight=False, deemphasize=False):
        """ Draw this Region into our window.

            'dc' is the device context to use for drawing.  If 'selected' is
            True, the object is currently selected and should be drawn as such.
        """

        if self.name.lower() == "boundary":
            dc.SetPen(wx.Pen(self.color, 3, wx.SOLID))
            dc.SetBrush(wx.Brush(wx.Colour(self.color.Red(), self.color.Green(),
                         self.color.Blue(), 0), wx.TRANSPARENT))
        else:
            if deemphasize:
                newcolor = wx.Colour(0.1*self.color.Red(), 0.1*self.color.Green(), 0.1*self.color.Blue())
                dc.SetPen(wx.Pen(newcolor, 1, wx.SOLID))
                dc.SetBrush(wx.Brush(wx.Colour(newcolor.Red(), newcolor.Green(),
                             newcolor.Blue(), 128), wx.SOLID))
            else: 
                dc.SetPen(wx.Pen(self.color, 1, wx.SOLID))
                dc.SetBrush(wx.Brush(wx.Colour(self.color.Red(), self.color.Green(),
                             self.color.Blue(), 128), wx.SOLID))

        self._privateDraw(dc, self.position, selected, scale, showAlignmentPoints)

        if highlight:
            pdc.SetPen(wx.Pen(wx.BLACK, 3, wx.SOLID))
            #pdc.SetBrush(wx.Brush(wx.RED, wx.BDIAGONAL_HATCH))
            pdc.SetBrush(wx.Brush(wx.BLACK, wx.CROSSDIAG_HATCH))

            self._privateDraw(pdc, self.position, selected, scale, showAlignmentPoints)


    # =====================
    # == Private Methods ==
    # =====================

    def _privateDraw(self, dc, position, selected, scale, showAlignmentPoints):
        """ Private routine to draw this Region.

            'dc' is the device context to use for drawing, while 'position' is
            the position in which to draw the object.  If 'selected' is True,
            the object is drawn with selection handles.  'scale' is a fixed 
            scaling ratio to use when drawing.  This private drawing
            routine assumes that the pen and brush have already been set by the
            caller.
        """

        if self.type == reg_POLY:
            scaledPointArray = map(lambda pt: wx.Point(scale*pt.x, scale*pt.y), self.pointArray)
            dc.DrawPolygon(scaledPointArray + [scaledPointArray[0]], scale*position.x, scale*position.y)
        elif self.type == reg_RECT:
            dc.DrawRectangle(scale*position.x, scale*position.y,
                             scale*self.size.width, scale*self.size.height)
                
        for i, pt in enumerate(self.getPoints()):
            if selected:
                # Draw selection handles at all vertices
                dc.SetPen(wx.TRANSPARENT_PEN)
                dc.SetBrush(wx.BLACK_BRUSH)
                self._drawSelHandle(dc, pt.x, pt.y)
            if showAlignmentPoints and self.alignmentPoints[i]:
                # Highlight vertices to be used for calibration
                dc.SetBrush(wx.Brush(wx.RED, wx.SOLID))
                dc.SetPen(wx.Pen(wx.BLACK, 1, wx.SOLID))
                dc.DrawRectangle(pt.x - 5, pt.y - 5, 10, 10)

                # Draw aligment vertex labels
                dc.SetTextForeground(wx.BLACK)
                dc.SetBackgroundMode(wx.TRANSPARENT)
                font = wx.Font(12, wx.FONTFAMILY_SWISS, wx.NORMAL, wx.BOLD, False)
                dc.SetFont(font)
                
                labelStr = self.name + "_P" + str(i);
                textWidth, textHeight = dc.GetTextExtent(labelStr)
                
                textX = pt.x + 8 # - textWidth/2
                textY = pt.y - 1.5*textHeight
                dc.DrawText(labelStr, textX, textY)


    def _drawSelHandle(self, dc, x, y):
        """ Draw a selection handle around this Region.

            'dc' is the device context to draw the selection handle within,
            while 'x' and 'y' are the coordinates to use for the centre of the
            selection handle.
        """
        dc.DrawRectangle(x - 3, y - 3, 6, 6)

#----------------------------------------------------------------------------

def drawMap(target, rfi, scaleToFit=True, drawLabels=True, highlightList=[], deemphasizeList=[], memory=False):
    """ Draw the map contained in the given RegionFileInterface onto the target canvas.
    """

    # Nothing to draw if there are no regions loaded yet
    if rfi is None:
        print "ERROR: Can't draw a map without loading some regions first"
        return

    # Upgrade from Regions to DrawableRegions, if necessary
    # TODO: Should really only be necessary once
    for i, region in enumerate(rfi.regions):
        if isinstance(region, DrawableRegion):
            continue

        obj = DrawableRegion(region.type)
        obj.setData(region.getData())
        rfi.regions[i] = obj
        del region

    if memory:
        dc = wx.MemoryDC()
        dc.SelectObject(target)
        pdc = dc
    else:
        pdc = wx.PaintDC(target)
        try:
            dc = wx.GCDC(pdc)
        except:
            dc = pdc
        else:
            target.PrepareDC(pdc)

        target.PrepareDC(dc)

    dc.BeginDrawing()

    dc.Clear()

    # TODO: draw background image?
    
    if scaleToFit:
        # Figure out scaling
        maximumWidth = target.GetSize().x
        maximumHeight = target.GetSize().y
        windowAspect = 1.0*maximumHeight/maximumWidth

        # TODO: Assuming the regions don't change, we only really need to calculate this once
        (leftMargin, topMargin, rightExtent, downExtent) = rfi.getBoundingBox()

        W = rightExtent + 2*leftMargin
        H = downExtent + 2*topMargin

        imgAspect = 1.0*H/W

        # Decide aspect based on whether rendering is width- or height-constrained
        if imgAspect >= windowAspect:
            NewH = maximumHeight
            mapScale = 1.0*NewH/H
        else:
            NewW = maximumWidth
            mapScale = 1.0*NewW/W
    else:
        mapScale = 1

    # Draw the regions!
    for i in range(len(rfi.regions)-1, -1, -1):
        obj = rfi.regions[i]
        doHighlight = (obj.name in highlightList)
        doDeemphasize = (obj.name in deemphasizeList)

        obj.draw(dc, pdc, False, mapScale, showAlignmentPoints=False, highlight=doHighlight, deemphasize=doDeemphasize)

        if drawLabels:
            # Draw region labels
            dc.SetTextForeground(wx.BLACK)
            dc.SetBackgroundMode(wx.TRANSPARENT)
            font = wx.Font(12, wx.FONTFAMILY_SWISS, wx.NORMAL, wx.BOLD, False)
            dc.SetFont(font)
            
            textWidth, textHeight = dc.GetTextExtent(obj.name)
            
            # TODO: Better text placement algorithm for concave polygons?
            if doDeemphasize:
                newcolor = wx.Colour(0.1*obj.color.Red(), 0.1*obj.color.Green(), 0.1*obj.color.Blue())
                dc.SetBrush(wx.Brush(newcolor, wx.SOLID))
                dc.SetPen(wx.Pen(newcolor, 1, wx.SOLID))
            else:
                dc.SetBrush(wx.Brush(obj.color, wx.SOLID))
                dc.SetPen(wx.Pen(obj.color, 1, wx.SOLID))

            center = obj.getCenter()
            if obj.name.lower() == "boundary":
                textX = mapScale*obj.position.x
                textY = mapScale*obj.position.y + mapScale*obj.size.height + textHeight/2
            else:
                textX = mapScale*center.x - textWidth/2
                textY = mapScale*center.y - textHeight/2

            dc.DrawRoundedRectangle(textX - 5, textY - 3, textWidth + 10, textHeight + 6, 3)
            dc.DrawText(obj.name, textX, textY)

    dc.EndDrawing()

    if memory:
        dc.SelectObject(wx.NullBitmap)

    return mapScale
