import wx
from regionEditor import DrawableRegion

def drawMap(target, proj, scaleToFit=True, drawLabels=True, highlightList=[], memory=False):
	""" Draw the map contained in the given project onto the target canvas.
	"""

	# Nothing to draw if there are no regions loaded yet
	if proj.rfi is None:
		print "ERROR: Can't draw a map without loading some regions first"
		return

	# Upgrade from Regions to DrawableRegions, if necessary
	# TODO: Should really only be necessary once
	for i, region in enumerate(proj.rfi.regions):
		if isinstance(region, DrawableRegion):
			continue

		obj = DrawableRegion(region.type)
		obj.setData(region.getData())
		proj.rfi.regions[i] = obj
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
		leftMargin = min([pt.x for region in proj.rfi.regions for pt in region.getPoints()])
		topMargin = min([pt.y for region in proj.rfi.regions for pt in region.getPoints()])
		rightExtent = max([pt.x for region in proj.rfi.regions for pt in region.getPoints()])
		downExtent = max([pt.y for region in proj.rfi.regions for pt in region.getPoints()])

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
	for i in range(len(proj.rfi.regions)-1, -1, -1):
		obj = proj.rfi.regions[i]
		doHighlight = (obj.name in highlightList)

		obj.draw(dc, pdc, False, mapScale, showAlignmentPoints=False, highlight=doHighlight)

		if drawLabels:
			# Draw region labels
			dc.SetTextForeground(wx.BLACK)
			dc.SetBackgroundMode(wx.TRANSPARENT)
			font = wx.Font(12, wx.FONTFAMILY_SWISS, wx.NORMAL, wx.BOLD, False)
			dc.SetFont(font)
			
			textWidth, textHeight = dc.GetTextExtent(obj.name)
			
			# TODO: Better text placement algorithm for concave polygons?
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
