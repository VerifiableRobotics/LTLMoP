#!/usr/bin/env python
#
# -*- coding: US-ASCII -*-
#
# generated by wxGlade HG on Wed Jan 16 09:31:15 2013
#

import wx
import wx.grid
import threading
import sys
import socket
import time

# begin wxGlade: dependencies
# end wxGlade

# begin wxGlade: extracode
# end wxGlade


class ActuatorDisplayFrame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: ActuatorDisplayFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.history_grid = wx.grid.Grid(self, wx.ID_ANY, size=(1, 1))

        self.__set_properties()
        self.__do_layout()
        # end wxGlade

        self.currentValues = {}

        self.history_grid.CreateGrid(0,1)
        self.history_grid.SetColLabelValue(0, "     Time     ")
        self.history_grid.SetColSize(0,-1)  # Auto-size
        self.history_grid.EnableEditing(False)

        # Create new thread to listen for commands from parent
        self.listenThread = threading.Thread(target = self._listen)
        self.listenThread.daemon = True
        self.listenThread.start()

        # Let everyone know we're ready
        host = 'localhost'
        port = 23559
        buf = 1024
        addr = (host,port)
        UDPSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        UDPSock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        UDPSock.sendto("Hello!\n",addr)
        UDPSock.close()

    def _listen(self):
        while True:
            # Wait for and receive a message
            data = raw_input().strip()

            if data == ':QUIT':  # EOF indicates that the connection has been destroyed
                print "Listen thread is shutting down."
                wx.CallAfter(self.Destroy())
                break

            name, val = data.split(",")
            if val == "init":
                wx.CallAfter(self.actuatorInitialize, name)
            else:
                wx.CallAfter(self.actuatorUpdate, name, val)

    def actuatorInitialize(self, name):
        # Set up the logging grid
        self.history_grid.AppendCols(1)
        i = self.history_grid.GetNumberCols() - 1
        self.history_grid.SetColLabelValue(i, " " + name + " ")
        self.history_grid.SetColSize(i,-1)  # Auto-size

    def actuatorUpdate(self, name, val):
        self.currentValues[name] = val

        self.history_grid.AppendRows(1)
        lastrow = self.history_grid.GetNumberRows()-1

        for i in range(self.history_grid.GetNumberCols()):
            if i == 0:
                # Time column
                #now = time.strftime("%Y-%m-%d %H:%M:%S") 
                now = time.strftime("%H:%M:%S")
                self.history_grid.SetCellValue(lastrow,i,now)
            else:
                # Actuator value column
                v = self.currentValues[self.history_grid.GetColLabelValue(i).strip()]

                if int(v) == 0:
                    self.history_grid.SetCellValue(lastrow,i,"False")
                    self.history_grid.SetCellBackgroundColour(lastrow,i,wx.Colour(255, 0, 0))
                elif int(v) == 1:
                    self.history_grid.SetCellValue(lastrow,i,"True")
                    self.history_grid.SetCellBackgroundColour(lastrow,i,wx.Colour(0, 255, 0))
                else:
                    self.history_grid.SetCellValue(lastrow,i,v)

        self.history_grid.ClearSelection()
        self.history_grid.MakeCellVisible(lastrow,0)
        self.history_grid.ForceRefresh()

    def __set_properties(self):
        # begin wxGlade: ActuatorDisplayFrame.__set_properties
        self.SetTitle("Dummy Actuator Handler")
        self.SetSize((511, 372))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: ActuatorDisplayFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_1.Add(self.history_grid, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_1)
        self.Layout()
        # end wxGlade

# end of class ActuatorDisplayFrame
class ActuatorHandlerApp(wx.App):
    def OnInit(self):
        wx.InitAllImageHandlers()
        frame_1 = ActuatorDisplayFrame(None, wx.ID_ANY, "")
        self.SetTopWindow(frame_1)
        frame_1.Show()
        return 1

# end of class ActuatorHandlerApp

if __name__ == "__main__":
    ActuatorHandler = ActuatorHandlerApp(0)
    ActuatorHandler.MainLoop()
