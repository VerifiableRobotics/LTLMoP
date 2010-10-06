#!/usr/bin/env python

""" ===============================
    regionEditor.py - Region Editor
    ===============================
    
    A simple polygonal region editor.  The code is kind of a mess, but it does what it needs to do.

    Based on pySketch by Erik Westra (ewestra@wave.co.nz) and Bill Baxter (wbaxter@gmail.com)
"""

import sys, copy, re, shutil
import traceback, types
import cPickle, os.path, os
import wxversion
wxversion.select('2.8')
import wx
from wx.lib.buttons import GenBitmapButton

from regions import *

####################################################################
# TODO:
#   - backport new pysketch bugfixes so it will work well on OS X
#   - bring in scaling factors from spec file for display
#   - 
#   - profile and optimize, so we run smoothly with large numbers of regions
#   - allow for snaps to faces
#   - be more graceful when load fails
#   - automatic regioning?
#   - multi-level undo (also allow undo of non-drawing actions)
#   - all the other todos sprinkled around
####################################################################

#----------------------------------------------------------------------------
#                            System Constants
#----------------------------------------------------------------------------

# Preferences:

SNAP_RADIUS = 10

# Menu item IDs:

[menu_SETBG,        menu_UNDO,
 menu_SELECT_ALL,   menu_DUPLICATE,     menu_EDIT_REGION,
 menu_DELETE,       menu_SELECT,        menu_RECT,
 menu_POLY,         menu_ADD_PT,        menu_DEL_PT,
 menu_CALIB_PT,     menu_ABOUT] = [wx.NewId() for i in range(13)]

# Timer IDs:

id_SNAP_TIMER = wx.NewId()

# Tool IDs:

[id_SELECT,     id_RECT,    id_POLY,
 id_ADD_PT,     id_DEL_PT,  id_CALIB_PT] = [wx.NewId() for i in range(6)]

# Mousing operations:
# TODO: Actually implement the rotation operation (CTRL+handledrag?)
[mouse_NONE,    mouse_RESIZE,   mouse_MOVE,
 mouse_DRAG,    mouse_CREATE,   mouse_ROTATE] = range(6)

# Visual Feedback types:

[feedback_RECT, feedback_LINE,  feedback_POLY] = range(3)

# Mouse-event action parameter types:

[param_RECT, param_POLY] = range(2)

# Size of the drawing page, in pixels.

[PAGE_WIDTH, PAGE_HEIGHT] = [1000, 1000]

#----------------------------------------------------------------------------

class DrawingFrame(wx.Frame):
    """ A frame showing the contents of a single document. """

    # ==========================================
    # == Initialisation and Window Management ==
    # ==========================================

    def __init__(self, parent, id, title, fileName=None):
        """ Standard constructor.

            'parent', 'id' and 'title' are all passed to the standard wx.Frame
            constructor.  'fileName' is the name and path of a saved file to
            load into this frame, if any.
        """
        wx.Frame.__init__(self, parent, id, "Region Editor - " + title,
                         style = wx.DEFAULT_FRAME_STYLE | wx.WANTS_CHARS |
                                 wx.NO_FULL_REPAINT_ON_RESIZE)

        # Make status bar at bottom.

        self.sb = wx.StatusBar(self)
        self.SetStatusBar(self.sb)
        self.sb.SetFieldsCount(2)

        # Setup our snapping support.

        self.snapTimer = wx.Timer(self, id_SNAP_TIMER)
        self.Bind(wx.EVT_TIMER, self.onTimerEvent)
        self.lastCursor = None
        self.snapCoords = None

        # Setup our menu bar.

        menuBar = wx.MenuBar()

        self.fileMenu = wx.Menu()
        self.fileMenu.Append(wx.ID_NEW,    "&New\tCTRL-N")
        self.fileMenu.Append(wx.ID_OPEN,   "&Open...\tCTRL-O")
        self.fileMenu.Append(wx.ID_CLOSE,  "&Close\tCTRL-W")
        self.fileMenu.AppendSeparator()
        self.fileMenu.Append(menu_SETBG,   "&Import Background...\tCTRL-I")
        self.fileMenu.AppendSeparator()
        self.fileMenu.Append(wx.ID_SAVE,   "&Save\tCTRL-S")
        self.fileMenu.Append(wx.ID_SAVEAS, "Save &As...")
        self.fileMenu.Append(wx.ID_REVERT, "&Revert...")
        self.fileMenu.AppendSeparator()
        self.fileMenu.Append(wx.ID_EXIT,   "&Quit\tCTRL-Q")

        menuBar.Append(self.fileMenu, "&File")

        self.editMenu = wx.Menu()
        self.editMenu.Append(menu_UNDO,          "&Undo\tCTRL-Z")
        self.editMenu.AppendSeparator()
        self.editMenu.Append(menu_SELECT_ALL,    "Select &All\tCTRL-A")
        self.editMenu.AppendSeparator()
        self.editMenu.Append(menu_EDIT_REGION,   "&Edit Region...\tCTRL-E")
        self.editMenu.Append(menu_DUPLICATE,     "&Duplicate\tCTRL-D")
        self.editMenu.Append(menu_DELETE,        "D&elete\tDEL")

        menuBar.Append(self.editMenu, "&Edit")

        self.toolsMenu = wx.Menu()
        self.toolsMenu.Append(menu_SELECT,   "&Selection\tS",    kind=wx.ITEM_CHECK)
        self.toolsMenu.Append(menu_RECT,     "&Rectangle\tR",    kind=wx.ITEM_CHECK)
        self.toolsMenu.Append(menu_POLY,     "&Polygon\tP",      kind=wx.ITEM_CHECK)
        self.toolsMenu.Append(menu_ADD_PT,   "&Create Point\tC", kind=wx.ITEM_CHECK)
        self.toolsMenu.Append(menu_DEL_PT,   "&Delete Point\tD", kind=wx.ITEM_CHECK)
        self.toolsMenu.Append(menu_CALIB_PT, "&Toggle Calibration Points\tT", kind=wx.ITEM_CHECK)

        menuBar.Append(self.toolsMenu, "&Tools")

        self.helpMenu = wx.Menu()
        self.helpMenu.Append(menu_ABOUT, "&About Region Editor...")

        menuBar.Append(self.helpMenu, "&Help")

        self.SetMenuBar(menuBar)

        # Create our toolbar.

        tsize = (16,16)
        self.toolbar = self.CreateToolBar(wx.TB_HORIZONTAL |
                                          wx.NO_BORDER | wx.TB_FLAT)

        self.toolbar.AddSimpleTool(wx.ID_NEW,
                                   wx.ArtProvider.GetBitmap(wx.ART_NEW, wx.ART_TOOLBAR, tsize),
                                   "New")
        self.toolbar.AddSimpleTool(wx.ID_OPEN,
                                   wx.ArtProvider.GetBitmap(wx.ART_FILE_OPEN, wx.ART_TOOLBAR, tsize),
                                   "Open")
        self.toolbar.AddSimpleTool(wx.ID_SAVE,
                                   wx.ArtProvider.GetBitmap(wx.ART_FILE_SAVE, wx.ART_TOOLBAR, tsize),
                                   "Save")
        self.toolbar.AddSeparator()
        self.toolbar.AddSimpleTool(menu_UNDO,
                                   wx.ArtProvider.GetBitmap(wx.ART_UNDO, wx.ART_TOOLBAR, tsize),
                                   "Undo")
        self.toolbar.AddSeparator()
        self.toolbar.AddSimpleTool(menu_DUPLICATE,
                                   wx.Bitmap("images/duplicate.bmp",
                                            wx.BITMAP_TYPE_BMP),
                                   "Duplicate")
        self.toolbar.Realize()

        # Associate each menu/toolbar item with the method that handles that
        # item.

        menuHandlers = [
        (wx.ID_NEW,    self.doNew),
        (wx.ID_OPEN,   self.doOpen),
        (wx.ID_CLOSE,  self.doClose),
        (wx.ID_SAVE,   self.doSave),
        (wx.ID_SAVEAS, self.doSaveAs),
        (wx.ID_REVERT, self.doRevert),
        (wx.ID_EXIT,   self.doExit),

        (menu_SETBG,         self.doSetBackground),

        (menu_UNDO,          self.doUndo),
        (menu_SELECT_ALL,    self.doSelectAll),
        (menu_DUPLICATE,     self.doDuplicate),
        (menu_EDIT_REGION,   self.doEditRegion),
        (menu_DELETE,        self.doDelete),

        (menu_SELECT,    self.doChooseSelectTool),
        (menu_RECT,      self.doChooseRectTool),
        (menu_POLY,      self.doChoosePolyTool),
        (menu_ADD_PT,    self.doChooseAddPtTool),
        (menu_DEL_PT,    self.doChooseDelPtTool),
        (menu_CALIB_PT,  self.doChooseCalibPtTool),

        (menu_ABOUT, self.doShowAbout)]

        for combo in menuHandlers:
            id, handler = combo
            self.Bind(wx.EVT_MENU, handler, id = id)
                
        
        # Install our own method to handle closing the window.  This allows us
        # to ask the user if he/she wants to save before closing the window, as
        # well as keep track of which windows are currently open.

        self.Bind(wx.EVT_CLOSE, self.doClose)

        # Install our own method for handling keystrokes.  We use this to let
        # the user move the selected object(s) around using the arrow keys.

        self.Bind(wx.EVT_CHAR_HOOK, self.onKeyEvent)

        # Setup our top-most panel.  This holds the entire contents of the
        # window, excluding the menu bar.

        self.topPanel = wx.Panel(self, -1, style=wx.SIMPLE_BORDER)

        # Setup our tool palette, with all our drawing tools and option icons.

        self.toolPalette = wx.BoxSizer(wx.VERTICAL)

        self.selectIcon  = ToolPaletteIcon(self.topPanel, id_SELECT,
                                           "select", "Selection Tool")
        self.rectIcon    = ToolPaletteIcon(self.topPanel, id_RECT,
                                           "rect", "Rectangle Tool")
        self.polyIcon    = ToolPaletteIcon(self.topPanel, id_POLY,
                                           "poly", "Polygon Tool")
        self.addPtIcon   = ToolPaletteIcon(self.topPanel, id_ADD_PT,
                                           "addPoint", "Add Point Tool")
        self.delPtIcon   = ToolPaletteIcon(self.topPanel, id_DEL_PT,
                                           "delPoint", "Delete Point Tool")
        self.calibPtIcon = ToolPaletteIcon(self.topPanel, id_CALIB_PT,
                                           "calibPoint", "Toggle Calibration Point Tool")

        toolSizer = wx.GridSizer(0, 2, 5, 5)
        toolSizer.Add(self.selectIcon)
        toolSizer.Add(self.polyIcon)
        toolSizer.Add(self.rectIcon)
        toolSizer.Add(self.addPtIcon)
        toolSizer.Add(self.delPtIcon)
        toolSizer.Add(self.calibPtIcon)

        margin = wx.TOP | wx.LEFT | wx.RIGHT | wx.ALIGN_CENTRE
        self.toolPalette.Add(toolSizer, 0, margin, 5)

        # Make the tool palette icons respond when the user clicks on them.

        for tool in [self.selectIcon, self.rectIcon, self.polyIcon,
                     self.addPtIcon, self.delPtIcon, self.calibPtIcon]:
            tool.Bind(wx.EVT_BUTTON, self.onToolIconClick)

        # Setup the main drawing area.

        self.drawPanel = wx.ScrolledWindow(self.topPanel, -1,
                                          style=wx.SUNKEN_BORDER)
        self.drawPanel.SetBackgroundColour(wx.WHITE)

        # TODO: I guess the virtual size should scale according to background image,
        # but we don't want to shrink it too much when the image is small.
        self.drawPanel.SetVirtualSize((PAGE_WIDTH, PAGE_HEIGHT))
        self.drawPanel.SetScrollRate(20, 20)

        self.drawPanel.Bind(wx.EVT_LEFT_DOWN, self.onMouseEvent)
        self.drawPanel.Bind(wx.EVT_LEFT_DCLICK, self.onDoubleClickEvent)
        self.drawPanel.Bind(wx.EVT_RIGHT_DOWN, self.onRightClick)
        self.drawPanel.Bind(wx.EVT_MOTION, self.onMouseEvent)
        self.drawPanel.Bind(wx.EVT_LEFT_UP, self.onMouseEvent)
        self.drawPanel.Bind(wx.EVT_PAINT, self.onPaintEvent)

        # Position everything in the window.

        topSizer = wx.BoxSizer(wx.HORIZONTAL)
        topSizer.Add(self.toolPalette, 0)
        topSizer.Add(self.drawPanel, 1, wx.EXPAND)

        self.topPanel.SetAutoLayout(True)
        self.topPanel.SetSizer(topSizer)

        self.SetSizeHints(250, 200)
        self.SetSize(wx.Size(900, 700))

        # Setup our frame to hold the contents of a sketch document.

        self.rfi = RegionFileInterface()
        self.backgroundImage = None
        self.fileName  = fileName
        
        self.transitionFaces = {}          # The keys are faces that are shared by more than one region

        self.dirty     = False
        self.needsAdjacencyRecalc = True
        self.selection = []                # List of selected Regions
        self.undoInfo  = None              # Saved contents for undo
        self.mouseMode  = mouse_NONE       # Current mousing mode
        self.newPoints = []                # A temp buffer to store vertices when drawing out new regions

        # Select an initial tool.

        self.curTool = None
        self._setCurrentTool(self.selectIcon)

        # Load our file if necessary        

        if self.fileName != None:
            if self.loadContents() is False:
                raise IOError

    # ============================
    # == Event Handling Methods ==
    # ============================

    # FIXME: These are really messy; refactor me

    def onToolIconClick(self, event):
        """ Respond to the user clicking on one of our tool icons.
        """
        iconID = event.GetEventObject().GetId()
        if   iconID == id_SELECT:    self.doChooseSelectTool()
        elif iconID == id_RECT:      self.doChooseRectTool()
        elif iconID == id_POLY:      self.doChoosePolyTool()
        elif iconID == id_ADD_PT:    self.doChooseAddPtTool()
        elif iconID == id_DEL_PT:    self.doChooseDelPtTool()
        elif iconID == id_CALIB_PT: self.doChooseCalibPtTool()
        else:                       print "Tool not yet implemented"

    def onKeyEvent(self, event):
        """ Respond to a keypress event.

            We make the arrow keys move the selected object(s) by one pixel in
            the given direction.
        """
        if event.GetKeyCode() == wx.WXK_UP:
            self._moveObjects(0, -1)
        elif event.GetKeyCode() == wx.WXK_DOWN:
            self._moveObjects(0, 1)
        elif event.GetKeyCode() == wx.WXK_LEFT:
            self._moveObjects(-1, 0)
        elif event.GetKeyCode() == wx.WXK_RIGHT:
            self._moveObjects(1, 0)
        elif event.GetKeyCode() == wx.WXK_BACK:
            self.doDelete()
        elif event.GetKeyCode() == wx.WXK_SPACE:
            self.testNear()
            #self.testBetween()
        elif event.GetKeyCode() == wx.WXK_ESCAPE:
            self.mouseMode = mouse_NONE
            self.sb.SetStatusText("", 1)
            self.doChooseSelectTool()
            self.deselectAll()
            self.drawPanel.Refresh()
        else:
            event.Skip()
    
    def testNear(self):
        newReg = self.selection[0].findRegionNear(20, mode='overEstimate')
        obj = DrawableRegion(newReg.type)
        obj.setData(newReg.getData())
        self.rfi.regions.append(obj)
        self.drawPanel.Refresh()

    def testBetween(self):
        newReg = findRegionBetween(self.selection[0], self.selection[1])
        obj = DrawableRegion(newReg.type)
        obj.setData(newReg.getData())
        self.rfi.regions.append(obj)
        self.drawPanel.Refresh()

    def onMouseEvent(self, event):
        """ Respond to the user clicking on our main drawing panel.

            How we respond depends on the currently selected tool.
        """
        if not (event.LeftDown() or event.Dragging() or event.Moving() or event.LeftUp()):
            return

        if self.curTool == self.selectIcon:
            feedbackType = feedback_RECT
            action       = self.selectByRectangle
            actionParam  = param_RECT
            selecting    = True
            dashedLine   = True
        elif self.curTool == self.rectIcon:
            feedbackType = feedback_RECT
            action       = self.createRect
            actionParam  = param_RECT
            selecting    = False
            dashedLine   = False
        elif self.curTool == self.polyIcon:
            feedbackType = feedback_LINE
            action       = self.createPoly
            actionParam  = param_POLY
            selecting    = False
            dashedLine   = False
        elif self.curTool == self.delPtIcon:
            selecting    = False
        elif self.curTool == self.addPtIcon:
            selecting    = False
        elif self.curTool == self.calibPtIcon:
            selecting    = False
        else:
            print "Unknown tool"
            return

        if event.LeftDown():
            mousePt = self._getEventCoordinates(event)
            if selecting:
                obj, handle = self._getObjectAndSelectionHandleAt(mousePt)

            if selecting and (obj != None) and (handle != handle_NONE):
                # The user clicked on an object's selection handle.  Let the
                # user resize the clicked-on object.

                self.mouseMode    = mouse_RESIZE
                self.resizeObject = obj

                self.select(obj)

                if obj.type == reg_POLY:
                    self.resizeFeedback = feedback_POLY
                    self.resizeAnchor  = (obj.position + obj.pointArray[(handle-1) % len(obj.pointArray)],
                                          obj.position + obj.pointArray[(handle+1) % len(obj.pointArray)]) 
                    self.resizeFloater = obj.position + obj.pointArray[handle]
                else:
                    points = [x for x in obj.getPoints()]
                    self.resizeFeedback = feedback_RECT
                    self.resizeFloater = points[handle]
                    self.resizeAnchor = points[(handle+2) % 4]

                self.curPt = mousePt
                self.resizeOffsetX = self.resizeFloater.x - mousePt.x
                self.resizeOffsetY = self.resizeFloater.y - mousePt.y
                endPt = wx.Point(self.curPt.x + self.resizeOffsetX,
                                self.curPt.y + self.resizeOffsetY)
                self._drawVisualFeedback(self.resizeAnchor, endPt,
                                         self.resizeFeedback, False)

            elif selecting and (self._getObjectAt(mousePt) != None):

                # The user clicked on an object to select it.  If the user
                # drags, he/she will move the object.
                obj = self._getObjectAt(mousePt)
                if event.ShiftDown():
                    if obj in self.selection:
                        self.selection.remove(obj)
                    else:
                        self.selection.append(obj)    
                    self.drawPanel.Refresh()
                else:
                    if obj not in self.selection:
                        self.select(obj)
                    self.mouseMode = mouse_MOVE
                    self.moveOrigin = mousePt
                    self.curPt      = mousePt
                    self._drawObjectOutline(0, 0)
                self._adjustMenus()

            elif selecting:
                # The user is dragging out a selection rect.

                self.dragOrigin = mousePt
                self.curPt      = mousePt
                self.drawPanel.SetCursor(wx.CROSS_CURSOR)
                self.drawPanel.CaptureMouse()
                self._drawVisualFeedback(mousePt, mousePt, feedbackType,
                                         dashedLine)
                self.mouseMode = mouse_DRAG

            event.Skip()
            return

        if event.Moving() or event.Dragging():
            mousePt = self._getEventCoordinates(event)

            # Reset timer and cursor
            if self.lastCursor is not None:
                self.drawPanel.SetCursor(self.lastCursor)
                self.lastCursor = None
                self.snapCoords = None
            self.snapTimer.Start(250, wx.TIMER_ONE_SHOT)

            # Update mouse coord display on status bar
            self.sb.SetStatusText("Mouse at (%dpx, %dpx)" % (mousePt.x, mousePt.y), 0)

        if event.Dragging():
            if self.mouseMode == mouse_RESIZE:

                # We're resizing an object.

                mousePt = self._getEventCoordinates(event)
                if (self.curPt.x != mousePt.x) or (self.curPt.y != mousePt.y):
                    # Erase previous visual feedback.
                    endPt = wx.Point(self.curPt.x + self.resizeOffsetX,
                                    self.curPt.y + self.resizeOffsetY)
                    self._drawVisualFeedback(self.resizeAnchor, endPt,
                                             self.resizeFeedback, False)
                    self.curPt = mousePt
                    # Draw new visual feedback.
                    endPt = wx.Point(self.curPt.x + self.resizeOffsetX,
                                    self.curPt.y + self.resizeOffsetY)
                    self._drawVisualFeedback(self.resizeAnchor, endPt,
                                             self.resizeFeedback, False)

            elif self.mouseMode == mouse_MOVE:

                # We're moving a selected object.

                mousePt = self._getEventCoordinates(event)
                if event.ShiftDown():
                    if abs(mousePt.x-self.moveOrigin.x) < abs(mousePt.y-self.moveOrigin.y):
                        mousePt.x = self.moveOrigin.x
                    else:
                        mousePt.y = self.moveOrigin.y
                if (self.curPt.x != mousePt.x) or (self.curPt.y != mousePt.y):
                    # Erase previous visual feedback.
                    self._drawObjectOutline(self.curPt.x - self.moveOrigin.x,
                                            self.curPt.y - self.moveOrigin.y)
                    self.curPt = mousePt
                    # Draw new visual feedback.
                    self._drawObjectOutline(self.curPt.x - self.moveOrigin.x,
                                            self.curPt.y - self.moveOrigin.y)

            elif self.mouseMode == mouse_DRAG:

                # We're dragging out a selection rect.

                mousePt = self._getEventCoordinates(event)
                if (self.curPt.x != mousePt.x) or (self.curPt.y != mousePt.y):
                    # Erase previous visual feedback.
                    self._drawVisualFeedback(self.dragOrigin, self.curPt,
                                             feedbackType, dashedLine)
                    self.curPt = mousePt
                    # Draw new visual feedback.
                    self._drawVisualFeedback(self.dragOrigin, self.curPt,
                                             feedbackType, dashedLine)

            event.Skip()
            return

        if event.Moving():
            if self.mouseMode == mouse_CREATE:

                mousePt = self._getEventCoordinates(event)
                if (self.curPt.x != mousePt.x) or (self.curPt.y != mousePt.y):
                    # Erase previous visual feedback.
                    self._drawVisualFeedback(self.dragOrigin, self.curPt,
                                             feedbackType, dashedLine)
                    self.curPt = mousePt
                    # Draw new visual feedback.
                    self._drawVisualFeedback(self.dragOrigin, self.curPt,
                                             feedbackType, dashedLine)

                # Update the status bar!
                if self.curTool == self.rectIcon:
                    self.sb.SetStatusText("Width: %dpx, Height: %dpx" % (abs(mousePt.x - self.dragOrigin.x), abs(mousePt.y-self.dragOrigin.y)), 1)
                elif self.curTool == self.polyIcon:
                    length = math.sqrt((mousePt.x - self.newPoints[-1].x)**2 + (mousePt.y-self.newPoints[-1].y)**2)
                    if len(self.newPoints) >= 2:
                        # We can show angle
                        v1_y = self.newPoints[-2].y-self.newPoints[-1].y
                        v1_x = self.newPoints[-2].x-self.newPoints[-1].x
                        v2_y = mousePt.y-self.newPoints[-1].y
                        v2_x = mousePt.x-self.newPoints[-1].x
                        angle_v1 = math.atan2(v1_y, v1_x)
                        angle_v2 = math.atan2(v2_y, v2_x)
                        angle = angle_v2 - angle_v1 
                        if(angle < 0):
                            angle += 2 * math.pi
                        # Only show the acute angle
                        if(angle > math.pi):
                            angle = 2*math.pi - angle
                        angle *= 180 / math.pi
                        self.sb.SetStatusText("Side Length: %dpx, Angle: %ddeg" % (length, angle), 1)
                    else:
                        self.sb.SetStatusText("Side Length: %dpx" % (length), 1)

            elif self.mouseMode == mouse_NONE and not selecting:
                self.curPt = self._getEventCoordinates(event)

            event.Skip()
            return

        if event.LeftUp():
            if self.mouseMode == mouse_RESIZE:

                # We're resizing an object.
                mousePt = self._getEventCoordinates(event,snap=True)
                # Erase last visual feedback.
                endPt = wx.Point(self.curPt.x + self.resizeOffsetX,
                                self.curPt.y + self.resizeOffsetY)
                self._drawVisualFeedback(self.resizeAnchor, endPt,
                                         self.resizeFeedback, False)

                resizePt = wx.Point(mousePt.x + self.resizeOffsetX,
                                   mousePt.y + self.resizeOffsetY)

                if (self.resizeFloater.x != resizePt.x) or \
                   (self.resizeFloater.y != resizePt.y):
                   self._resizeObject(self.resizeObject,
                                      self.resizeAnchor,
                                      self.resizeFloater,
                                      resizePt)
                else:
                    self.drawPanel.Refresh() # Clean up after empty resize.

            elif self.mouseMode == mouse_MOVE:
                mousePt = self.curPt
                # Erase last visual feedback.
                self._drawObjectOutline(self.curPt.x - self.moveOrigin.x,
                                        self.curPt.y - self.moveOrigin.y)
                if (self.moveOrigin.x != mousePt.x) or \
                   (self.moveOrigin.y != mousePt.y):
                    self._moveObjects(mousePt.x - self.moveOrigin.x,
                                     mousePt.y - self.moveOrigin.y)
                else:
                    self.drawPanel.Refresh() # Clean up after empty drag.

            elif self.mouseMode == mouse_DRAG:

                # We're dragging out a selection rect.
                mousePt = self._getEventCoordinates(event)
                # Erase last visual feedback.
                self._drawVisualFeedback(self.dragOrigin, self.curPt,
                                         feedbackType, dashedLine)
                self.drawPanel.ReleaseMouse()
                self.drawPanel.SetCursor(wx.STANDARD_CURSOR)
                # Perform the appropriate action for the current tool.
                if actionParam == param_RECT:
                    x1 = min(self.dragOrigin.x, self.curPt.x)
                    y1 = min(self.dragOrigin.y, self.curPt.y)
                    x2 = max(self.dragOrigin.x, self.curPt.x)
                    y2 = max(self.dragOrigin.y, self.curPt.y)

                    startX = x1
                    startY = y1
                    width  = x2 - x1
                    height = y2 - y1

                    action(x1, y1, x2-x1, y2-y1)

            elif self.mouseMode == mouse_CREATE:
                mousePt = self._getEventCoordinates(event, snap=True)

                if self.curTool == self.rectIcon:
                    self.curPt = mousePt
                    # Create rect
                    x1 = min(self.newPoints[0].x, self.curPt.x)
                    y1 = min(self.newPoints[0].y, self.curPt.y)
                    x2 = max(self.newPoints[0].x, self.curPt.x)
                    y2 = max(self.newPoints[0].y, self.curPt.y)

                    #if ((x2-x1) < 8) or ((y2-y1) < 8): return # Too small.

                    action(x1, y1, x2-x1, y2-y1)

                    self.mouseMode = mouse_NONE
                    self.sb.SetStatusText("", 1)
                elif self.curTool == self.polyIcon:
                    # Update to account for snap cases:
                    self._drawVisualFeedback(self.dragOrigin, self.curPt,
                                             feedbackType, dashedLine)
                    self.curPt = mousePt
                    self._drawVisualFeedback(self.dragOrigin, self.curPt,
                                             feedbackType, dashedLine)
                    # Append point, end if equal to start point
                    if self.curPt != self.newPoints[0]:
                        self.newPoints.append(self.curPt)
                        self.dragOrigin = self.curPt
                    else:
                        action(self.newPoints)
                        self.mouseMode = mouse_NONE
                        self.sb.SetStatusText("", 1)

            elif self.mouseMode == mouse_NONE and not selecting:
                if self.curTool == self.delPtIcon:
                    mousePt = self._getEventCoordinates(event)
                    obj = self._getObjectAt(mousePt)
                    if obj is not None:
                        if obj in self.selection:
                            obj, handle = self._getObjectAndSelectionHandleAt(mousePt)
                            if handle is not handle_NONE:
                                if not (obj.type == reg_POLY and len(obj.pointArray) <= 3):
                                    self._saveUndoInfo()
                                    obj.removePoint(handle)
                                    self.dirty = True
                                    self.needsAdjacencyRecalc = True
                                    self.doChooseSelectTool()
                        else:
                            self.select(obj) 
                        self.drawPanel.Refresh()
                        self._adjustMenus()
                elif self.curTool == self.addPtIcon:
                    mousePt = self._getEventCoordinates(event)
                    obj = self._getObjectAt(mousePt)
                    if obj is not None:
                        if obj in self.selection:
                            # TODO: because of the getObjectAt call,
                            # clicks outside the object will not be recognized
                            for i, face in enumerate(obj.getFaces()):
                                [d, pint] = pointLineIntersection(wx.Point(*face[0]), wx.Point(*face[1]), mousePt)
                                if d is not None and d <= 4:
                                    self._saveUndoInfo()
                                    obj.addPoint(pint-obj.position, i+1)
                                    self.dirty = True
                                    self.needsAdjacencyRecalc = True
                                    self.doChooseSelectTool()
                                    break
                        else:
                            self.select(obj)
                        self._adjustMenus()
                        self.drawPanel.Refresh()
                elif self.curTool == self.calibPtIcon:
                    mousePt = self._getEventCoordinates(event)

                    for obj in self.rfi.regions:
                        topLeft = obj.position
                        botRight = obj.position + wx.Point(obj.size.GetWidth(), obj.size.GetHeight())
                            
                        # First do rough checking based on obj bound rects
                        if (topLeft.x > mousePt.x + SNAP_RADIUS or botRight.x < mousePt.x - SNAP_RADIUS or
                            topLeft.y > mousePt.y + SNAP_RADIUS or botRight.y < mousePt.y - SNAP_RADIUS):
                            continue
                        else:
                            handle = obj.getSelectionHandleContainingPoint(mousePt.x, mousePt.y,
                            boundFunc = self._pointInSnapRange)
                            if handle == handle_NONE:
                                continue
                            else: 
                                self._saveUndoInfo()
                                obj.alignmentPoints[handle] = not obj.alignmentPoints[handle]
                                self.dirty = True
                                break

                    self.drawPanel.Refresh()
                    self._adjustMenus()
                else:
                    self.deselectAll()
                    mousePt = self._getEventCoordinates(event,snap=True)
                    self.curPt = mousePt
                    self.dragOrigin = mousePt
                    self.newPoints = [mousePt]
                    self.mouseMode = mouse_CREATE

            if self.mouseMode != mouse_CREATE:
                self.mouseMode = mouse_NONE # We've finished with this mouse event.
    
            event.Skip()


    def onDoubleClickEvent(self, event):
        """ Respond to a double-click within our drawing panel.
        """
        if self.mouseMode == mouse_CREATE:
            # TODO: Get rid of action variable in above
            self.createPoly(self.newPoints)
            self.mouseMode = mouse_NONE
            self.sb.SetStatusText("", 1)
            return

        mousePt = self._getEventCoordinates(event)
        obj = self._getObjectAt(mousePt)

        if obj == None: return

        self.select(obj)
        self.drawPanel.Refresh()

        # Let the user edit the given object.
        self.doEditRegion()

    def onTimerEvent(self, event):
        timerID = event.GetEventObject().GetId()

        if timerID == id_SNAP_TIMER:
            if self.curTool == self.selectIcon and self.mouseMode != mouse_RESIZE: return

            # Allow snapping to first point of polygon in creation
            if len(self.newPoints) > 1 and self._pointInSnapRange(self.curPt.x, self.curPt.y,
               self.newPoints[0].x, self.newPoints[0].y):
                self.snapCoords = self.newPoints[0]
                self.lastCursor = self.drawPanel.GetCursor()
                self.drawPanel.SetCursor(wx.StockCursor(wx.CURSOR_BULLSEYE))
                return

            for obj in self.rfi.regions:
                # Don't allow snapping to self when resizing: (you wouldn't ever want to do that, right?)
                if self.mouseMode == mouse_RESIZE and len(self.selection) != 0 and self.selection[0] == obj: continue

                topLeft = obj.position
                botRight = obj.position + wx.Point(obj.size.GetWidth(), obj.size.GetHeight())
                    
                # First do rough checking based on obj bound rects
                if (topLeft.x > self.curPt.x + SNAP_RADIUS or botRight.x < self.curPt.x - SNAP_RADIUS or
                    topLeft.y > self.curPt.y + SNAP_RADIUS or botRight.y < self.curPt.y - SNAP_RADIUS):
                    continue
                else:
                    handle = obj.getSelectionHandleContainingPoint(self.curPt.x, self.curPt.y,
                    boundFunc = self._pointInSnapRange)
                    if handle == handle_NONE:
                        continue
                    else: 
                        self.snapCoords = [x for x in obj.getPoints()][handle]
                        self.lastCursor = self.drawPanel.GetCursor()
                        self.drawPanel.SetCursor(wx.StockCursor(wx.CURSOR_BULLSEYE))
                        break
        

    def _pointInSnapRange(self, x, y, rX, rY):
        """ Return True iff (x, y) is within snapping range of the vertex at (rX, rY).
        """
        return ((x - rX)**2 + (y - rY)**2 < SNAP_RADIUS**2)

    def onRightClick(self, event):
        """ Respond to the user right-clicking within our drawing panel.

            We select the clicked-on item, if necessary, and display a pop-up
            menu of available options which can be applied to the selected
            item(s).
        """
        mousePt = self._getEventCoordinates(event)
        obj = self._getObjectAt(mousePt)

        if obj == None: return # Nothing selected.

        # Select the clicked-on object.

        if obj not in self.selection:
            self.select(obj)

        # Build our pop-up menu.

        menu = wx.Menu()
        menu.Append(menu_EDIT_REGION, "Edit Region...")
        menu.Append(menu_DUPLICATE, "Duplicate")
        menu.Append(menu_DELETE,    "Delete")

        menu.Enable(menu_EDIT_REGION, len(self.selection) == 1)

        self.Bind(wx.EVT_MENU, self.doEditRegion, id=menu_EDIT_REGION)
        self.Bind(wx.EVT_MENU, self.doDuplicate, id=menu_DUPLICATE)
        self.Bind(wx.EVT_MENU, self.doDelete, id=menu_DELETE)
                            
        # Show the pop-up menu.

        self.drawPanel.PopupMenu(menu, mousePt)
        menu.Destroy()


    def makeScreenshot(self, minimal=False):
        """
        Write out a png file with the regions overlayed on the background image.

        If minimal is True, only region borders will be drawn (as dotted lines).
        If minimal is False, the output will look like the editor window.

        Returns the output filename. 
        """

        # TODO: Make minimal/full an option settable via the GUI

        name, ext = os.path.splitext(os.path.basename(self.fileName))

        memory = wx.MemoryDC()

        # If there's a background image we'll use that size at minimum
        if self.backgroundImage is not None:
            x,y = self.backgroundImage.GetSize()
        else:
            x = None
            y = None
    
        # Now let's just make sure all the regions fit.

        # Make sure we have defined at least one regoin
        if len(self.rfi.regions) == 0:
            return None

        for region in self.rfi.regions:
            for pt in region.getPoints():
                if x == None or pt.x > x: x = pt.x
                if y == None or pt.y > y: y = pt.y

        bitmap = wx.EmptyBitmap(x,y)
        memory.SelectObject(bitmap)

        memory.BeginDrawing()
        if self.backgroundImage != None:
            memory.DrawBitmap(self.backgroundImage, 0, 0, False)

        if minimal:
            memory.SetBrush(wx.TRANSPARENT_BRUSH)
            #memory.SetLogicalFunction(wx.INVERT)

            faces = []
            for obj in self.rfi.regions:
                for face in obj.getFaces():
                    if (face[0], face[1]) in faces or (face[1], face[0]) in faces:
                        continue
                    faces.append(face)
                    memory.SetPen(wx.Pen(wx.WHITE, 5, wx.SOLID))
                    memory.DrawLine(face[0][0], face[0][1], face[1][0], face[1][1])
                    memory.SetPen(wx.Pen(wx.BLACK, 3, wx.LONG_DASH))
                    memory.DrawLine(face[0][0], face[0][1], face[1][0], face[1][1])
        else:
            if self.needsAdjacencyRecalc:
                self.recalcAdjacency()
            #self.drawRegions(memory, memory, drawAdjacencies=False)
            self.drawRegions(memory, memory)

        memory.EndDrawing()
        memory.SelectObject(wx.NullBitmap)
        fname = os.path.join(os.path.dirname(self.fileName),"%s_simbg.png" % name)
        bitmap.SaveFile(fname, wx.BITMAP_TYPE_PNG)

        return fname

    def onPaintEvent(self, event):
        """ Respond to a request to redraw the contents of our drawing panel.
        """

        pdc = wx.PaintDC(self.drawPanel)
        try:
            dc = wx.GCDC(pdc)
        except:
            dc = pdc
        else:
            self.drawPanel.PrepareDC(pdc)

        if self.needsAdjacencyRecalc:
            self.recalcAdjacency()

        self.drawPanel.PrepareDC(dc)
        dc.BeginDrawing()
        # TODO: Alpha BG so you can make it lighter
        if self.backgroundImage != None:
            dc.DrawBitmap(self.backgroundImage, 0, 0, False)
        
        self.drawRegions(dc, pdc)

        dc.EndDrawing()
        
    def drawRegions(self, dc, pdc, drawLabels=True, drawAdjacencies=True):
        for i in range(len(self.rfi.regions)-1, -1, -1):
            obj = self.rfi.regions[i]

            # If this region is concave, indicate this with hatching
            doHighlight = (obj.name.lower() != "boundary" and obj.getDirection() == dir_CONCAVE)

            if obj in self.selection:
                obj.draw(dc, pdc, True, highlight=doHighlight)
            else:
                obj.draw(dc, pdc, False, highlight=doHighlight)
    
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
                    textX = obj.position.x
                    textY = obj.position.y + obj.size.height + textHeight/2
                else:
                    textX = center.x - textWidth/2
                    textY = center.y - textHeight/2
                dc.DrawRoundedRectangle(textX - 5, textY - 3, textWidth + 10, textHeight + 6, 3)
                dc.DrawText(obj.name, textX, textY)

            if drawAdjacencies:
                # Highlight adjacent faces
                dc.SetPen(wx.Pen(wx.Colour(255,0,0,100), 5, wx.DOT))
                for face in self.transitionFaces.keys():
                    p1 = list(face)[0]
                    p2 = list(face)[1]
                    dc.DrawLine(p1[0], p1[1], p2[0], p2[1])
    


    # ==========================
    # == Menu Command Methods ==
    # ==========================

    def doNew(self, event):
        """ Respond to the "New" menu command.
        """
        global _docList
        newFrame = DrawingFrame(None, -1, "Untitled")
        newFrame.Show(True)
        _docList.append(newFrame)


    def doOpen(self, event):
        """ Respond to the "Open" menu command.
        """
        global _docList

        curDir = os.getcwd()
        fileName = wx.FileSelector("Open File", default_extension="regions",
                                  flags = wx.OPEN | wx.FILE_MUST_EXIST)
        if fileName == "": return
        fileName = os.path.join(os.getcwd(), fileName)
        os.chdir(curDir)
        self._doOpenHelper(fileName)


    def _doOpenHelper(self, fileName, existing=True):

        title = os.path.basename(fileName)

        if (self.fileName == None) and (len(self.rfi.regions) == 0):
            # If our current document is blank
            if existing:
                # Load contents into current (empty) document.
                self.fileName = fileName
                self.projectName = os.path.splitext(title)[0]
                self.projectDir = os.path.dirname(os.path.abspath(self.fileName))
                if self.loadContents() == False:
                    self.fileName = None
                    self.projectName = None
                    self.projectDir = None
                    return 
            else:
                # We're now editing the virgin file
                self.fileName = fileName

            self.SetTitle("Region Editor - " + os.path.basename(fileName))
        else:
            # Open a new frame for this document.
            try:
                newFrame = DrawingFrame(None, -1, os.path.basename(fileName),
                                        fileName=fileName)
            except IOError:
                return

            newFrame.Show(True)
            _docList.append(newFrame)

        self.projectName = os.path.splitext(title)[0]
        self.projectDir = os.path.dirname(os.path.abspath(self.fileName))

    def doClose(self, event):
        """ Respond to the "Close" menu command.
        """
        global _docList

        if self.dirty:
            if not self.askIfUserWantsToSave("closing"): return

        _docList.remove(self)
        self.Destroy()


    def checkSubfaces(self, obj1, obj2):
        """
        If we have a face of one region that is a subset of a face of another region,
        we want to split the larger face into two parts appropriately. 
        For now, this only works when the two faces share one point.
        """

        for obj in obj1:
            for face in obj.getFaces():
                for other_obj in obj2:
                    points = [x for x in other_obj.getPoints()]
                    if (face[0] in points and face[1] not in points):
                        existing = wx.Point(*face[0])
                        new = wx.Point(*face[1])
                    elif (face[1] in points and face[0] not in points):
                        existing = wx.Point(*face[1])
                        new = wx.Point(*face[0])
                    else:
                        continue
                    
                    index = points.index(existing)
                    prev = (index - 1) % len(points)
                    next = (index + 1) % len(points)

                    [d, pint] = pointLineIntersection(existing, points[prev], new)
                    if d is not None and d < 1: 
                        other_obj.addPoint(new-other_obj.position, index)
                        break 
                    [d, pint] = pointLineIntersection(existing, points[next], new)
                    if d is not None and d < 1: 
                        other_obj.addPoint(new-other_obj.position, next)
                        break 
        
    def recalcAdjacency(self):
        """
        Call the RegionFileInterface's recalcAdjacency() method to figure out where to draw dotted transition lines
        """

        self.transitionFaces = self.rfi.recalcAdjacency() # This is just a list of faces to draw dotted lines on

        #self.drawPanel.Refresh()

        self.needsAdjacencyRecalc = False

    def getProjectDir(self):
        """
        Make sure we've saved our region file somewhere so that our project directory
        and project name are defined.
        """

        if self.fileName is None:
            wx.MessageBox("Please save first to give the project a name.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            self.doSave()
            if self.fileName is None:
                # If the save was cancelled, forget it
                return None

        return self.projectDir

    def doSetBackground(self, event=None):
        if self.getProjectDir() is None:
            return

        if event == None:
            fileName = os.path.join(self.projectDir, self.rfi.background)
        else:
            curDir = os.getcwd()
            fileName = wx.FileSelector("Import Background",
                                      flags = wx.OPEN | wx.FILE_MUST_EXIST)
            if fileName == "": return
            fileName = os.path.join(os.getcwd(), fileName)

        bitmap = wx.Bitmap(fileName)
        if not bitmap.Ok(): 
            wx.MessageBox("Cannot import image from file %s" % (fileName), "Error",
                        style = wx.OK | wx.ICON_ERROR, parent=self)
            return

        # If this is a new background and it loaded successfully, now copy file to project directory
        if event != None and not os.path.exists(os.path.join(self.projectDir, os.path.basename(fileName))):
            shutil.copy(fileName, self.projectDir)

        fileName = os.path.basename(fileName)
    
        self.backgroundImage = bitmap
        self.rfi.background = fileName
        if event != None:
            self.dirty = True
        self._adjustMenus()

        currentWidth = self.GetSize().GetWidth()    
        currentHeight = self.GetSize().GetHeight()    
        newWidth = bitmap.GetWidth() + 100
        newHeight = bitmap.GetHeight() + 100
        displayHeight = wx.GetDisplaySize().GetHeight() - 100
        displayWidth = wx.GetDisplaySize().GetWidth() - 100 

        if newWidth > currentWidth:
            currentWidth = newWidth
        if newHeight > currentHeight:
            currentHeight = newHeight
        if currentHeight > displayHeight:
            currentHeight = displayHeight
        if currentWidth > displayWidth:
            currentWidth = displayWidth 
        self.SetSize(wx.Size(currentWidth, currentHeight))
        self.Centre()

        self.drawPanel.Refresh()
    

    def doSave(self, event=None):
        """ Respond to the "Save" menu command.
        """
        if self.fileName != None:
            self.saveContents()
        else:
            self.doSaveAs(event)


    def doSaveAs(self, event):
        """ Respond to the "Save As" menu command.
        """
        if self.fileName == None:
            default = ""
        else:
            default = self.fileName

        curDir = os.getcwd()
        fileName = wx.FileSelector("Save File As", "Saving",
                                  default_filename=default,
                                  default_extension="regions",
                                  wildcard="*.regions",
                                  flags = wx.SAVE | wx.OVERWRITE_PROMPT)
        if fileName == "": return # User cancelled.
        fileName = os.path.join(os.getcwd(), fileName)
        os.chdir(curDir)

        title = os.path.basename(fileName)
        self.SetTitle("Region Editor - " + title)

        self.fileName = fileName
        self.saveContents()

        self.projectName = os.path.splitext(title)[0]
        self.projectDir = os.path.dirname(os.path.abspath(self.fileName))


    def doRevert(self, event):
        """ Respond to the "Revert" menu command.
        """
        if not self.dirty: return

        if wx.MessageBox("Discard changes made to this document?", "Confirm",
                        style = wx.OK | wx.CANCEL | wx.ICON_QUESTION,
                        parent=self) == wx.CANCEL: return
        self.loadContents()


    def doExit(self, event):
        """ Respond to the "Quit" menu command.
        """
        global _docList, _app
        for doc in _docList:
            if not doc.dirty: continue
            doc.Raise()
            if not doc.askIfUserWantsToSave("quitting"): return
            _docList.remove(doc)
            doc.Destroy()

        _app.ExitMainLoop()


    def doUndo(self, event):
        """ Respond to the "Undo" menu command.
        """
        if self.undoInfo == None: return

        undoData = self.undoInfo
        self._saveUndoInfo() # For undoing the undo...

        self.rfi.regions = []

        for type, data in undoData["contents"]:
            obj = DrawableRegion(type)
            obj.setData(data)
            self.rfi.regions.append(obj)

        self.selection = []
        for i in undoData["selection"]:
            self.selection.append(self.rfi.regions[i])

        self.dirty = True
        self.needsAdjacencyRecalc = True
        self.drawPanel.Refresh()
        self._adjustMenus()


    def doSelectAll(self, event):
        """ Respond to the "Select All" menu command.
        """
        self.selectAll()


    def doDuplicate(self, event):
        """ Respond to the "Duplicate" menu command.
        """
        self._saveUndoInfo()

        objs = []
        for obj in self.selection:
            newObj = DrawableRegion(obj.type)
            name = newObj.name
            newObj.setData(obj.getData())
            newObj.position = copy.deepcopy(obj.position)
            newObj.name = name
            objs.append(newObj)

        self.rfi.regions = objs + self.rfi.regions

        self.selectMany(objs)
        self.dirty = True
        self.needsAdjacencyRecalc = True
        self._adjustMenus()

    def doEditRegion(self, event=None):
        """ Respond to the "Edit Region" menu command.
        """
        if len(self.selection) != 1: return

        obj = self.selection[0]

        if obj.type not in [reg_RECT, reg_POLY]: return

        editor = EditRegionDialog(self, "Edit Region Name")
        editor.objectToDialog(obj)
        editor.Centre()

        while 1:
            if editor.ShowModal() == wx.ID_CANCEL:
                editor.Destroy()
                return

            if editor.textCtrl.GetValue() not in [r.name for r in self.rfi.regions]:
                break

            wx.MessageBox("Region with name \"%s\" already exists." % (editor.textCtrl.GetValue()), "Error", 
                           style = wx.OK | wx.ICON_ERROR)

        self._saveUndoInfo()
        editor.dialogToObject(obj)
        editor.Destroy()

        self.dirty = True
        self.drawPanel.Refresh()
        self._adjustMenus()
        

    def doDelete(self, event=None):
        """ Respond to the "Delete" menu command.
        """
        self._saveUndoInfo()

        for obj in self.selection:
            self.rfi.regions.remove(obj)
            del obj
        self.deselectAll()
        self.dirty = True
        self.needsAdjacencyRecalc = True
        self._adjustMenus()



    def doChooseSelectTool(self, event=None):
        """ Respond to the "Select Tool" menu command.
        """
        self.drawPanel.SetCursor(wx.STANDARD_CURSOR)
        self._setCurrentTool(self.selectIcon)


    def doChooseRectTool(self, event=None):
        """ Respond to the "Rect Tool" menu command.
        """
        self.drawPanel.SetCursor(wx.CROSS_CURSOR)
        self._setCurrentTool(self.rectIcon)


    def doChoosePolyTool(self, event=None):
        """ Respond to the "Poly Tool" menu command.
        """
        self.drawPanel.SetCursor(wx.CROSS_CURSOR)
        self._setCurrentTool(self.polyIcon)


    def doChooseAddPtTool(self, event=None):
        """ Respond to the "Poly Tool" menu command.
        """
        self.drawPanel.SetCursor(wx.CROSS_CURSOR)
        self._setCurrentTool(self.addPtIcon)


    def doChooseDelPtTool(self, event=None):
        """ Respond to the "Poly Tool" menu command.
        """
        self.drawPanel.SetCursor(wx.CROSS_CURSOR)
        self._setCurrentTool(self.delPtIcon)


    def doChooseCalibPtTool(self, event=None):
        """ Respond to the "Toggle Calibration Point Tool" menu command.
        """
        self.drawPanel.SetCursor(wx.CROSS_CURSOR)
        self._setCurrentTool(self.calibPtIcon)

    def doShowAbout(self, event):
        """ Respond to the "About Region Editor" menu command.
        """
        dialog = wx.Dialog(self, -1, "About Region Editor") # ,
                          #style=wx.DIALOG_MODAL | wx.STAY_ON_TOP)

        panel = wx.Panel(dialog, -1)

        panelSizer = wx.BoxSizer(wx.VERTICAL)

        boldFont = wx.Font(panel.GetFont().GetPointSize(),
                          panel.GetFont().GetFamily(),
                          wx.NORMAL, wx.BOLD)

        logo = wx.StaticBitmap(panel, -1, wx.Bitmap("images/logo.bmp",
                                                  wx.BITMAP_TYPE_BMP))

        lab1 = wx.StaticText(panel, -1, "LTLMoP Region Editor")
        lab1.SetFont(wx.Font(24, boldFont.GetFamily(), wx.ITALIC, wx.BOLD))
        lab1.SetSize(lab1.GetBestSize())

        imageSizer = wx.BoxSizer(wx.HORIZONTAL)
        imageSizer.Add(logo, 0, wx.ALL | wx.ALIGN_CENTRE_VERTICAL, 5)
        imageSizer.Add(lab1, 0, wx.ALL | wx.ALIGN_CENTRE_VERTICAL, 5)

        lab2 = wx.StaticText(panel, -1, "A simple polygonal region editing " + \
                                       "tool.")
        lab2.SetFont(boldFont)
        lab2.SetSize(lab2.GetBestSize())

        lab3 = wx.StaticText(panel, -1, "Region Editor is completely free " + \
                                       "software; please")
        lab3.SetFont(boldFont)
        lab3.SetSize(lab3.GetBestSize())

        lab4 = wx.StaticText(panel, -1, "feel free to adapt or use this " + \
                                       "in any way you like.")
        lab4.SetFont(boldFont)
        lab4.SetSize(lab4.GetBestSize())

        lab5 = wx.StaticText(panel, -1, "Based on pySketch by Erik Westra " + \
                                       "(ewestra@wave.co.nz)")
        lab5.SetFont(boldFont)
        lab5.SetSize(lab5.GetBestSize())

        lab6 = wx.StaticText(panel, -1, "Adapted by Cameron Finucane " + \
                                       "(cameronp@seas.upenn.edu)")
        lab6.SetFont(boldFont)
        lab6.SetSize(lab6.GetBestSize())

        btnOK = wx.Button(panel, wx.ID_OK, "OK")

        panelSizer.Add(imageSizer, 0, wx.ALIGN_CENTRE)
        panelSizer.Add((10, 10)) # Spacer.
        panelSizer.Add(lab2, 0, wx.ALIGN_CENTRE)
        panelSizer.Add((10, 10)) # Spacer.
        panelSizer.Add(lab3, 0, wx.ALIGN_CENTRE)
        panelSizer.Add(lab4, 0, wx.ALIGN_CENTRE)
        panelSizer.Add((10, 10)) # Spacer.
        panelSizer.Add(lab5, 0, wx.ALIGN_CENTRE)
        panelSizer.Add(lab6, 0, wx.ALIGN_CENTRE)
        panelSizer.Add((10, 10)) # Spacer.
        panelSizer.Add(btnOK, 0, wx.ALL | wx.ALIGN_CENTRE, 5)

        panel.SetAutoLayout(True)
        panel.SetSizer(panelSizer)
        panelSizer.Fit(panel)

        topSizer = wx.BoxSizer(wx.HORIZONTAL)
        topSizer.Add(panel, 0, wx.ALL, 10)

        dialog.SetAutoLayout(True)
        dialog.SetSizer(topSizer)
        topSizer.Fit(dialog)

        dialog.Centre()

        btn = dialog.ShowModal()
        dialog.Destroy()

    # =============================
    # == Object Creation Methods ==
    # =============================
    def createPoly(self, points):
        """ Create poly object.
        """
        self._saveUndoInfo()

        obj = DrawableRegion(reg_POLY,
                            points=points)
        self.rfi.setToDefaultName(obj)
        obj.recalcBoundingBox()
        self.checkSubfaces([obj], self.rfi.regions)
        self.checkSubfaces(self.rfi.regions, [obj])
        self.rfi.regions.insert(0, obj)
        self.dirty = True
        self.needsAdjacencyRecalc = True
        
        if obj.getDirection() == dir_CCW:
            obj.pointArray.reverse()

        self.doChooseSelectTool()
        self.lastCursor = self.drawPanel.GetCursor()
        self.select(obj)


    def createRect(self, x, y, width, height):
        """ Create a new rectangle object at the given position and size.
        """
        self._saveUndoInfo()

        obj = DrawableRegion(reg_RECT, position=wx.Point(x, y),
                            size=wx.Size(width, height))
        self.rfi.setToDefaultName(obj)
        self.checkSubfaces([obj], self.rfi.regions)
        self.checkSubfaces(self.rfi.regions, [obj])
        self.rfi.regions.insert(0, obj)
        self.dirty = True
        self.needsAdjacencyRecalc = True

        self.doChooseSelectTool()
        self.lastCursor = self.drawPanel.GetCursor()
        self.select(obj)

    # =======================
    # == Selection Methods ==
    # =======================

    def selectAll(self):
        """ Select every Region in our document.
        """
        self.selection = []
        for obj in self.rfi.regions:
            self.selection.append(obj)
        self.drawPanel.Refresh()
        self._adjustMenus()


    def deselectAll(self):
        """ Deselect every Region in our document.
        """
        self.selection = []
        self.drawPanel.Refresh()
        self._adjustMenus()


    def select(self, obj):
        """ Select the given Region within our document.
        """
        self.selection = [obj]
        self.drawPanel.Refresh()
        self._adjustMenus()


    def selectMany(self, objs):
        """ Select the given list of Regions.
        """
        self.selection = objs
        self.drawPanel.Refresh()
        self._adjustMenus()


    def selectByRectangle(self, x, y, width, height):
        """ Select every Region in the given rectangular region.
        """
        self.selection = []
        for obj in self.rfi.regions:
            if obj.objectWithinRect(x, y, width, height):
                self.selection.append(obj)
        self.drawPanel.Refresh()
        self._adjustMenus()

    # ======================
    # == File I/O Methods ==
    # ======================

    def loadContents(self):
        """ Load the contents of our document into memory.
            Note that the drawing order of regions is last to first.
        """
        if not self.rfi.readFile(self.fileName):
            wx.MessageBox("Cannot open region file %s" % (self.fileName), "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return False

        self.selection = []

        if self.rfi.background != "None":	
            self.doSetBackground()

        # Convert from Regions to DrawableRegions
        for i, region in enumerate(self.rfi.regions):
            obj = DrawableRegion(region.type)
            obj.setData(region.getData())
            self.rfi.regions[i] = obj
            del region

        self.dirty = False
        self.undoInfo  = None
        self.needsAdjacencyRecalc = True

        self.doChooseSelectTool()

        self.drawPanel.Refresh()
        self._adjustMenus()

        return True

    def saveContents(self):
        """ Save the contents of our document to disk.
        """

        self.rfi.thumb = self.makeScreenshot(minimal=False)

        self.rfi.writeFile(self.fileName)

        self.dirty = False
        self._adjustMenus()


    def askIfUserWantsToSave(self, action):
        """ Give the user the opportunity to save the current document.

            'action' is a string describing the action about to be taken.  If
            the user wants to save the document, it is saved immediately.  If
            the user cancels, we return False.
        """
        if not self.dirty: return True # Nothing to do.

        response = wx.MessageBox("Save changes before " + action + "?",
                                "Confirm", wx.YES_NO | wx.CANCEL, self)

        if response == wx.YES:
            if self.fileName == None:
                fileName = wx.FileSelector("Save File As", "Saving",
                                          default_extension="psk",
                                          wildcard="*.psk",
                                          flags = wx.SAVE | wx.OVERWRITE_PROMPT)
                if fileName == "": return False # User cancelled.
                self.fileName = fileName

            self.saveContents()
            return True
        elif response == wx.NO:
            return True # User doesn't want changes saved.
        elif response == wx.CANCEL:
            return False # User cancelled.

    # =====================
    # == Private Methods ==
    # =====================

    def _adjustMenus(self):
        """ Adjust our menus and toolbar to reflect the current state of the
            world.
        """
        canSave   = self.dirty
        canRevert = (self.fileName != None) and self.dirty
        canUndo   = self.undoInfo != None
        selection = len(self.selection) > 0
        onlyOne   = len(self.selection) == 1
        front     = onlyOne and (self.selection[0] == self.rfi.regions[0])
        back      = onlyOne and (self.selection[0] == self.rfi.regions[-1])

        # Enable/disable our menu items.

        self.fileMenu.Enable(wx.ID_SAVE,   canSave)
        self.fileMenu.Enable(wx.ID_REVERT, canRevert)

        self.editMenu.Enable(menu_UNDO,      canUndo)
        self.editMenu.Enable(menu_EDIT_REGION, onlyOne)
        self.editMenu.Enable(menu_DUPLICATE, selection)
        self.editMenu.Enable(menu_DELETE,    selection)

        self.toolsMenu.Check(menu_SELECT,   self.curTool == self.selectIcon)
        self.toolsMenu.Check(menu_RECT,     self.curTool == self.rectIcon)
        self.toolsMenu.Check(menu_POLY,     self.curTool == self.polyIcon)
        self.toolsMenu.Check(menu_ADD_PT,   self.curTool == self.addPtIcon)
        self.toolsMenu.Check(menu_DEL_PT,   self.curTool == self.delPtIcon)
        self.toolsMenu.Check(menu_CALIB_PT, self.curTool == self.calibPtIcon)

        # Enable/disable our toolbar icons.

        self.toolbar.EnableTool(wx.ID_NEW,           True)
        self.toolbar.EnableTool(wx.ID_OPEN,          True)
        self.toolbar.EnableTool(wx.ID_SAVE,         canSave)
        self.toolbar.EnableTool(menu_UNDO,          canUndo)
        self.toolbar.EnableTool(menu_DUPLICATE,     selection)


    def _setCurrentTool(self, newToolIcon):
        """ Set the currently selected tool.
        """
        if self.curTool != None:
            self.curTool.deselect()

        self.newPoints = []
        newToolIcon.select()
        self.curTool = newToolIcon

        self._adjustMenus()
        self.mouseMode = mouse_NONE
        self.sb.SetStatusText("", 1)
        self.drawPanel.Refresh()


    def _saveUndoInfo(self):
        """ Remember the current state of the document, to allow for undo.

            We make a copy of the document's contents, so that we can return to
            the previous contents if the user does something and then wants to
            undo the operation.
        """
        savedContents = []
        for obj in self.rfi.regions:
            savedContents.append([obj.type, obj.getData()])

        savedSelection = []
        for i in range(len(self.rfi.regions)):
            if self.rfi.regions[i] in self.selection:
                savedSelection.append(i)

        self.undoInfo = {"contents"  : savedContents,
                         "selection" : savedSelection}


    def _resizeObject(self, obj, anchorPt, oldPt, newPt):
        """ Resize the given object.

            'anchorPt' is the unchanging corner of the object, while the
            opposite corner has been resized.  'oldPt' are the current
            coordinates for this corner, while 'newPt' are the new coordinates.
            The object should fit within the given dimensions, though if the
            new point is less than the anchor point the object will need to be
            moved as well as resized, to avoid giving it a negative size.
        """
        self._saveUndoInfo()

        if obj.type == reg_POLY:
            for index, pt in enumerate(obj.getPoints()):
                if pt == oldPt:
                    obj.pointArray[index] = obj.pointArray[index] + (newPt - oldPt)
                    break

            obj.recalcBoundingBox()
        else:
            topLeft  = wx.Point(min(anchorPt.x, newPt.x),
                               min(anchorPt.y, newPt.y))
            botRight = wx.Point(max(anchorPt.x, newPt.x),
                               max(anchorPt.y, newPt.y))

            newWidth  = botRight.x - topLeft.x
            newHeight = botRight.y - topLeft.y

            # Finally, adjust the bounds of the object to match the new dimensions.
    
            obj.position = topLeft
            obj.size = wx.Size(botRight.x - topLeft.x, botRight.y - topLeft.y)

        self.checkSubfaces([obj], self.rfi.regions)
        self.checkSubfaces(self.rfi.regions, [obj])

        self.drawPanel.Refresh()
        self.dirty = True
        self.needsAdjacencyRecalc = True
        self._adjustMenus()


    def _moveObjects(self, offsetX, offsetY):
        """ Move the currently selected object(s) by the given offset.
        """
        self._saveUndoInfo()

        for obj in self.selection:
            pos = obj.position
            pos.x = pos.x + offsetX
            pos.y = pos.y + offsetY
            obj.position = pos

        self.drawPanel.Refresh()
        self.dirty = True
        self.needsAdjacencyRecalc = True
        self._adjustMenus()

    def _getEventCoordinates(self, event, snap=False):
        """ Return the coordinates associated with the given mouse event.

            The coordinates have to be adjusted to allow for the current scroll
            position.
        """
        if snap and self.snapCoords is not None:
            # For resizing cases, because we don't care where you grabbed the handle
            self.resizeOffsetX, self.resizeOffsetY = 0, 0
            return self.snapCoords

        originX, originY = self.drawPanel.GetViewStart()
        unitX, unitY = self.drawPanel.GetScrollPixelsPerUnit()
        return wx.Point(event.GetX() + (originX * unitX),
                       event.GetY() + (originY * unitY))



    def _getObjectAndSelectionHandleAt(self, pt, boundFunc = None):
        """ Return the object and selection handle at the given point.

            We draw selection handles (small rectangles) around the currently
            selected object(s).  If the given point is within one of the
            selection handle rectangles, we return the associated object and a
            code indicating which selection handle the point is in.  If the
            point isn't within any selection handle at all, we return the tuple
            (None, handle_NONE).
        """
        for obj in self.selection:
            handle = obj.getSelectionHandleContainingPoint(pt.x, pt.y, boundFunc)
            if handle != handle_NONE:
                return obj, handle

        return None, handle_NONE


    def _getObjectAt(self, pt):
        """ Return the first object found which is at the given point.
        """
        for obj in self.rfi.regions:
            # TODO: Do preliminary, less-expensive checking first
            if obj.objectContainsPoint(pt.x, pt.y):
                return obj
        return None


    def _drawObjectOutline(self, offsetX, offsetY):
        """ Draw an outline of the currently selected object.

            The selected object's outline is drawn at the object's position
            plus the given offset.

            Note that the outline is drawn by *inverting* the window's
            contents, so calling _drawObjectOutline twice in succession will
            restore the window's contents back to what they were previously.
        """
        if len(self.selection) == 0: return

        dc = wx.ClientDC(self.drawPanel)
        self.drawPanel.PrepareDC(dc)
        dc.BeginDrawing()
        dc.SetPen(wx.BLACK_DASHED_PEN)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.SetLogicalFunction(wx.INVERT)

        for i in xrange(len(self.selection)):
            position = self.selection[i].position
            size     = self.selection[i].size
    
            if self.selection[i].type == reg_POLY:
                dc.DrawLines(self.selection[i].pointArray + [self.selection[i].pointArray[0]],
                             position.x + offsetX, position.y + offsetY)
            else:
                dc.DrawRectangle(position.x + offsetX, position.y + offsetY,
                                 size.width, size.height)
    
        dc.EndDrawing()


    def _drawVisualFeedback(self, startPt, endPt, type, dashedLine):
        """ Draw visual feedback for a drawing operation.

            The visual feedback consists of a line, ellipse, or rectangle based
            around the two given points.  'type' should be one of the following
            predefined feedback type constants:

                feedback_RECT     ->  draw rectangular feedback.
                feedback_LINE     ->  draw line feedback.
                feedback_POLY     ->  draw poly (two-line) feedback.

            if 'dashedLine' is True, the feedback is drawn as a dashed rather
            than a solid line.

            Note that the feedback is drawn by *inverting* the window's
            contents, so calling _drawVisualFeedback twice in succession will
            restore the window's contents back to what they were previously.
        """
        dc = wx.ClientDC(self.drawPanel)
        self.drawPanel.PrepareDC(dc)
        dc.BeginDrawing()
        if dashedLine:
            dc.SetPen(wx.BLACK_DASHED_PEN)
        else:
            dc.SetPen(wx.BLACK_PEN)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.SetLogicalFunction(wx.INVERT)

        if type == feedback_RECT:
            dc.DrawRectangle(startPt.x, startPt.y,
                             endPt.x - startPt.x,
                             endPt.y - startPt.y)
        elif type == feedback_LINE:
            dc.DrawLine(startPt.x, startPt.y, endPt.x, endPt.y)
        elif type == feedback_POLY:
            dc.DrawLine(startPt[0].x, startPt[0].y, endPt.x, endPt.y)
            dc.DrawLine(startPt[1].x, startPt[1].y, endPt.x, endPt.y)

        dc.EndDrawing()

#----------------------------------------------------------------------------

class DrawableRegion(Region):
    """ Extends the Region class to allow drawing.
    """

    # ============================
    # == Object Drawing Methods ==
    # ============================

    def draw(self, dc, pdc, selected, scale=1.0, showAlignmentPoints=True, highlight=False):
        """ Draw this Region into our window.

            'dc' is the device context to use for drawing.  If 'selected' is
            True, the object is currently selected and should be drawn as such.
        """

        if self.name.lower() == "boundary":
            dc.SetPen(wx.Pen(self.color, 3, wx.SOLID))
            dc.SetBrush(wx.Brush(wx.Colour(self.color.Red(), self.color.Green(),
                         self.color.Blue(), 0), wx.TRANSPARENT))
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

class ToolPaletteIcon(GenBitmapButton):
    """ An icon appearing in the tool palette area of our sketching window.

        Note that this is actually implemented as a wx.Bitmap rather
        than as a wx.Icon.  wx.Icon has a very specific meaning, and isn't
        appropriate for this more general use.
    """

    def __init__(self, parent, iconID, iconName, toolTip):
        """ Standard constructor.

            'parent'   is the parent window this icon will be part of.
            'iconID'   is the internal ID used for this icon.
            'iconName' is the name used for this icon.
            'toolTip'  is the tool tip text to show for this icon.

            The icon name is used to get the appropriate bitmap for this icon.
        """
        bmp = wx.Bitmap("images/" + iconName + "Icon.bmp", wx.BITMAP_TYPE_BMP)
        GenBitmapButton.__init__(self, parent, iconID, bmp, wx.DefaultPosition,
                                wx.Size(bmp.GetWidth(), bmp.GetHeight()), wx.NO_BORDER)
        selBmp = wx.Bitmap("images/" + iconName + "IconSel.bmp",
                           wx.BITMAP_TYPE_BMP)
        self.SetBitmapSelected(selBmp)
        self.SetUseFocusIndicator(False)

        self.SetToolTip(wx.ToolTip(toolTip))

        self.iconID     = iconID
        self.iconName   = iconName
        self.isSelected = False


    def select(self):
        """ Select the icon.

            The icon's visual representation is updated appropriately.
        """
        if self.isSelected: return # Nothing to do!

        bmp = wx.Bitmap("images/" + self.iconName + "IconSel.bmp",
                       wx.BITMAP_TYPE_BMP)
        self.SetBitmapLabel(bmp)
        self.isSelected = True

        self.Refresh()


    def deselect(self):
        """ Deselect the icon.

            The icon's visual representation is updated appropriately.
        """
        if not self.isSelected: return # Nothing to do!

        bmp = wx.Bitmap("images/" + self.iconName + "Icon.bmp",
                       wx.BITMAP_TYPE_BMP)
        self.SetBitmapLabel(bmp)
        self.isSelected = False

        self.Refresh()


#----------------------------------------------------------------------------

class EditRegionDialog(wx.Dialog):
    """ Dialog box used to edit the properties of a region.

        The user can edit the region's name and color.
    """

    def __init__(self, parent, title):
        """ Standard constructor.
        """
        wx.Dialog.__init__(self, parent, -1, title)

        gap = wx.LEFT | wx.TOP | wx.RIGHT

        self.label1 = wx.StaticText(self, -1, "Name:")
        self.textCtrl = wx.TextCtrl(self, 1001, "", style=wx.TE_PROCESS_ENTER,
                                   validator=TextObjectValidator())
        extent = self.textCtrl.GetFullTextExtent("Hy")
        lineHeight = extent[1] + extent[3]
        self.textCtrl.SetSize(wx.Size(-1, lineHeight * 1))

        line1sizer = wx.BoxSizer(wx.HORIZONTAL)
        line1sizer.Add(self.label1, 0, gap, 5)
        line1sizer.Add(self.textCtrl, 0, gap, 5)

        self.label2 = wx.StaticText(self, -1, "Color:")

        self.colorPicker = wx.ColourPickerCtrl(self, 1002)

        line2sizer = wx.BoxSizer(wx.HORIZONTAL)
        line2sizer.Add(self.label2, 0, gap, 5)
        line2sizer.Add(self.colorPicker, 0, gap, 5)

        self.okButton     = wx.Button(self, wx.ID_OK,     "OK")
        self.okButton.SetDefault()
        self.cancelButton = wx.Button(self, wx.ID_CANCEL, "Cancel")

        btnSizer = wx.BoxSizer(wx.HORIZONTAL)
        btnSizer.Add(self.okButton,     0, gap)
        btnSizer.Add(self.cancelButton, 0, gap)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(line1sizer, 1, gap | wx.EXPAND,       5)
        #sizer.Add((10, 10)) # Spacer.
        sizer.Add(line2sizer, 1, gap | wx.ALIGN_CENTRE,       5)
        #sizer.Add((10, 10)) # Spacer.
        sizer.Add(btnSizer,      0, gap | wx.ALIGN_CENTRE, 5)
        sizer.Add((10, 10)) # Spacer.

        self.SetAutoLayout(True)
        self.SetSizer(sizer)
        sizer.Fit(self)

        self.textCtrl.SetFocus()


    def objectToDialog(self, obj):
        """ Copy the properties of the given text object into the dialog box.
        """
        self.textCtrl.SetValue(obj.name)
        self.textCtrl.SetSelection(0, len(obj.name))
        self.colorPicker.SetColour(obj.color)

    def dialogToObject(self, obj):
        """ Copy the properties from the dialog box into the given text object.
        """
        obj.name = self.textCtrl.GetValue()
        obj.color = self.colorPicker.GetColour()

#----------------------------------------------------------------------------

class TextObjectValidator(wx.PyValidator):
    """ This validator is used to ensure that the user has entered something
        into the text object editor dialog's text field.
    """
    def __init__(self):
        """ Standard constructor.
        """
        wx.PyValidator.__init__(self)


    def Clone(self):
        """ Standard cloner.

            Note that every validator must implement the Clone() method.
        """
        return TextObjectValidator()


    def Validate(self, win):
        """ Validate the contents of the given text control.
        """
        textCtrl = self.GetWindow()
        text = textCtrl.GetValue()

        if len(text) == 0:
            wx.MessageBox("You must give the region a name.", "Error")
            return False

        # TODO: Check to see if name already exists?
        
        return True


    def TransferToWindow(self):
        """ Transfer data from validator to window.

            The default implementation returns False, indicating that an error
            occurred.  We simply return True, as we don't do any data transfer.
        """
        return True # Prevent wx.Dialog from complaining.


    def TransferFromWindow(self):
        """ Transfer data from window to validator.

            The default implementation returns False, indicating that an error
            occurred.  We simply return True, as we don't do any data transfer.
        """
        return True # Prevent wx.Dialog from complaining.

#----------------------------------------------------------------------------

class ExceptionHandler:
    """ A simple error-handling class to write exceptions to a text file.

        Under MS Windows, the standard DOS console window doesn't scroll and
        closes as soon as the application exits, making it hard to find and
        view Python exceptions.  This utility class allows you to handle Python
        exceptions in a more friendly manner.
    """

    def __init__(self):
        """ Standard constructor.
        """
        self._buff = ""
        if os.path.exists("errors.txt"):
            os.remove("errors.txt") # Delete previous error log, if any.


    def write(self, s):
        """ Write the given error message to a text file.

            Note that if the error message doesn't end in a carriage return, we
            have to buffer up the inputs until a carriage return is received.
        """
        if (s[-1] != "\n") and (s[-1] != "\r"):
            self._buff = self._buff + s
            return

        try:
            s = self._buff + s
            self._buff = ""

            if s[:9] == "Traceback":
                # Tell the user than an exception occurred.
                wx.MessageBox("An internal error has occurred.\nPlease " + \
                             "refer to the 'errors.txt' file for details.",
                             "Error", wx.OK | wx.CENTRE | wx.ICON_EXCLAMATION)

            f = open("errors.txt", "a")
            f.write(s)
            f.close()
        except:
            pass # Don't recursively crash on errors.

#----------------------------------------------------------------------------

class SketchApp(wx.App):
    """ The main pySketch application object.
    """
    def OnInit(self):
        """ Initialise the application.
        """
        global _docList
        _docList = []

        # No file name was specified on the command line -> start with a
        # blank document.
        frame = DrawingFrame(None, -1, "Untitled")
        frame.Centre()
        frame.Show(True)
        _docList.append(frame)

        if len(sys.argv) > 1:
            # Load the file(s) specified on the command line.
            for arg in sys.argv[1:]:
                fileName = os.path.join(os.getcwd(), arg)
                frame._doOpenHelper(fileName, os.path.isfile(fileName))

        return True

#----------------------------------------------------------------------------

def main():
    """ Start up the pySketch application.
    """
    global _app

    # Redirect python exceptions to a log file.

    #sys.stderr = ExceptionHandler()

    # Create and start the pySketch application.

    _app = SketchApp(0)
    _app.MainLoop()


if __name__ == "__main__":
    main()


