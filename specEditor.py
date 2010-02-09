#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" LTLMoP Toolkit - Specification Editor
    
    A development environment for specifications written in simple English,
    allowing for simulation in Stage, Gazebo, or with any Player-enabled robot

    This is completely free software; please feel free to adapt or use this in
    any way you like.

    Written by Cameron Finucane (cameronp@seas.upenn.edu)

    Last updated October 2008
"""

import re, sys, os, subprocess, time, copy
import wxversion
wxversion.select('2.8')
import wx, wx.richtext, wx.grid
from regions import *
from createTLVinput import createPFfile, createSMVfile
from parseEnglishToLTL import writeSpec
import fileMethods
import fcntl
import execute
import project

##################### WARNING! ########################
#     DO NOT EDIT GUI CODE BY HAND.  USE WXGLADE.     #
#   The .wxg file is located in the dev/ directory.   #
#######################################################



class CheckGrid(wx.grid.Grid):
    """
    A Grid subclass to allow one-click-toggle checkboxes in the grid.
    
    (See http://wiki.wxpython.org/Change_wxGrid_CheckBox_with_one_click)
    """

    def __init__(self, *args, **kwds):
        wx.grid.Grid.__init__(self, *args, **kwds)

        # We'll add rows and columns later in setLabels
        self.CreateGrid(0,0)

        self.Bind(wx.grid.EVT_GRID_CELL_LEFT_CLICK, self.onMouse)
        self.Bind(wx.grid.EVT_GRID_SELECT_CELL, self.onCellSelected)
        self.Bind(wx.grid.EVT_GRID_EDITOR_CREATED, self.onEditorCreated)

    def setLabels(self, rowValues, colValues):
        attr = wx.grid.GridCellAttr()
        attr.SetEditor(wx.grid.GridCellBoolEditor())
        attr.SetRenderer(wx.grid.GridCellBoolRenderer())

        self.AppendCols(len(colValues))
        for i, value in enumerate(colValues):
            self.SetColLabelValue(i, value)
            self.SetColAttr(i, attr)
            # FIXME: Autosize doesn't work?
            #self.AutoSizeColLabelSize(i)

        self.AppendRows(len(rowValues))
        for i, value in enumerate(rowValues):
            self.SetRowLabelValue(i, value)
            # FIXME: Autosize doesn't work?
            #self.AutoSizeRowLabelSize(i)

    def onMouse(self, evt):
        wx.CallLater(100, self.toggleCheckBox)
        evt.Skip()

    def toggleCheckBox(self):
        self.cb.Value = not self.cb.Value

    def onCellSelected(self, evt):
        wx.CallAfter(self.EnableCellEditControl)
        evt.Skip()

    def onEditorCreated(self,evt):
        self.cb = evt.Control
        evt.Skip()

class simSetupDialog(wx.Dialog):
    """
    The simulation configuration dialog.
    Essentially just an interface to the simSetup data structure.
    """

    def __init__(self, parent, *args, **kwds):
        # begin wxGlade: simSetupDialog.__init__
        kwds["style"] = wx.DEFAULT_DIALOG_STYLE
        wx.Dialog.__init__(self, *args, **kwds)
        self.sizer_22_staticbox = wx.StaticBox(self, -1, "Initial Conditions")
        self.sizer_26_staticbox = wx.StaticBox(self, -1, "Coordinate transformation (actual = pixels * scale + offset):")
        self.sizer_27_staticbox = wx.StaticBox(self, -1, "Experiment Settings")
        self.sizer_28_staticbox = wx.StaticBox(self, -1, "Experiment Name")
        self.list_box_experiment_name = wx.ListBox(self, -1, choices=[])
        self.button_sim_add = wx.Button(self, wx.ID_ADD, "")
        self.button_sim_delete = wx.Button(self, wx.ID_DELETE, "")
        self.button_sim_copy = wx.Button(self, wx.ID_COPY, "")
        self.label_9 = wx.StaticText(self, -1, "Experiment Name: ")
        self.text_ctrl_sim_experiment_name = wx.TextCtrl(self, -1, "")
        self.label_6 = wx.StaticText(self, -1, "Starting position: ")
        self.choice_startpos = wx.Choice(self, -1, choices=[])
        self.label_2 = wx.StaticText(self, -1, "Custom Propositions:")
        self.list_box_init_customs = wx.CheckListBox(self, -1, choices=["1", "2"])
        self.label_2_copy = wx.StaticText(self, -1, "Action Propositions:")
        self.list_box_init_actions = wx.CheckListBox(self, -1, choices=["3", "4"])
        self.label_8 = wx.StaticText(self, -1, "Sensor Activation")
        self.list_box_init_sensors = wx.CheckListBox(self, -1, choices=["5", "6"])
        self.label_7 = wx.StaticText(self, -1, "Simulation Environment: ")
        self.choice_simtype = wx.Choice(self, -1, choices=["Gazebo", "Stage", "None (Just connect to Player)"])
        self.label_7_copy = wx.StaticText(self, -1, "Robot Type:")
        self.choice_sim_robot = wx.Choice(self, -1, choices=[])
        self.label_4 = wx.StaticText(self, -1, "X Scale: ")
        self.text_ctrl_xscale = wx.TextCtrl(self, -1, "")
        self.label_5 = wx.StaticText(self, -1, "X Offset: ")
        self.text_ctrl_xoffset = wx.TextCtrl(self, -1, "")
        self.label_4_copy = wx.StaticText(self, -1, "Y Scale: ")
        self.text_ctrl_yscale = wx.TextCtrl(self, -1, "")
        self.label_5_copy = wx.StaticText(self, -1, "Y Offset: ")
        self.text_ctrl_yoffset = wx.TextCtrl(self, -1, "")
        self.button_sim_cali = wx.Button(self, -1, "Calibrate...")
        self.button_sim_apply = wx.Button(self, wx.ID_APPLY, "")
        self.button_sim_ok = wx.Button(self, wx.ID_OK, "")
        self.button_sim_cancel = wx.Button(self, wx.ID_CANCEL, "")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_LISTBOX, self.onSimLoad, self.list_box_experiment_name)
        self.Bind(wx.EVT_BUTTON, self.onSimAdd, self.button_sim_add)
        self.Bind(wx.EVT_BUTTON, self.onSimDelete, self.button_sim_delete)
        self.Bind(wx.EVT_BUTTON, self.onSimCopy, self.button_sim_copy)
        self.Bind(wx.EVT_TEXT, self.onSimNameEdit, self.text_ctrl_sim_experiment_name)
        self.Bind(wx.EVT_BUTTON, self.onClickCalibrate, self.button_sim_cali)
        self.Bind(wx.EVT_BUTTON, self.onClickApply, self.button_sim_apply)
        self.Bind(wx.EVT_BUTTON, self.onClickOK, self.button_sim_ok)
        # end wxGlade

        #########################################
        # Simulation configuration dialog setup #
        #########################################

        # Easier to store a ref to the parent than to pass tons of data
        self.parent = parent
        
        # Get a list of the enabled action propositions
        self.actions = []
        for i, action in enumerate(parent.list_box_actions.GetItems()):
            if parent.list_box_actions.IsChecked(i):
                self.actions.append(action)

        # Custom propositions can't be disabled, so just take all of them
        self.customs = parent.list_box_customs.GetItems()
        
        # Get a list of the enabled sensor
        self.sensors = []
        for i, sensor in enumerate(parent.list_box_sensors.GetItems()):
            if parent.list_box_sensors.IsChecked(i):
                self.sensors.append(sensor)

        # make a copy of the simSetup for temporarily store the data 
        self.tempSimSetup = copy.deepcopy(parent.simSetup)
        # Load simulation setup for each experiment

        for id, config in enumerate(parent.simSetup):
            try:
                print 'Loading experiment configuration %s...' %config['Name']
                self.list_box_experiment_name.Insert(config['Name'],id)
                self.loadSimSetup(id)
            except:
                print "Cannot load simulation setup for %s. Please check the spec file." % config['Name']
        self.list_box_experiment_name.Select(self.list_box_experiment_name.GetItems().index(parent.currentExperimentName))


    def __set_properties(self):
        # begin wxGlade: simSetupDialog.__set_properties
        self.SetTitle("Configure Simulation")
        self.SetSize((1000, 750))
        self.text_ctrl_sim_experiment_name.SetMinSize((300, 27))
        self.choice_startpos.SetMinSize((300, 29))
        self.list_box_init_customs.SetSelection(0)
        self.list_box_init_actions.SetSelection(0)
        self.list_box_init_sensors.SetSelection(0)
        self.choice_simtype.SetMinSize((150, 29))
        self.choice_simtype.SetSelection(1)
        self.choice_sim_robot.SetMinSize((150, 29))
        self.text_ctrl_xscale.SetMinSize((160, 27))
        self.text_ctrl_xoffset.SetMinSize((160, 27))
        self.text_ctrl_yscale.SetMinSize((160, 27))
        self.text_ctrl_yoffset.SetMinSize((160, 27))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: simSetupDialog.__do_layout
        sizer_6 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_12 = wx.BoxSizer(wx.VERTICAL)
        sizer_13 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_27 = wx.StaticBoxSizer(self.sizer_27_staticbox, wx.VERTICAL)
        sizer_18_copy = wx.BoxSizer(wx.VERTICAL)
        sizer_26 = wx.StaticBoxSizer(self.sizer_26_staticbox, wx.VERTICAL)
        sizer_20 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_19_copy = wx.BoxSizer(wx.HORIZONTAL)
        sizer_19 = wx.BoxSizer(wx.HORIZONTAL)
        grid_sizer_1 = wx.GridSizer(2, 2, 15, 5)
        sizer_22 = wx.StaticBoxSizer(self.sizer_22_staticbox, wx.VERTICAL)
        sizer_23 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_22_copy = wx.BoxSizer(wx.VERTICAL)
        sizer_17_copy = wx.BoxSizer(wx.VERTICAL)
        sizer_17 = wx.BoxSizer(wx.VERTICAL)
        sizer_21 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_30 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_29 = wx.BoxSizer(wx.VERTICAL)
        sizer_28 = wx.StaticBoxSizer(self.sizer_28_staticbox, wx.VERTICAL)
        sizer_29_copy = wx.BoxSizer(wx.HORIZONTAL)
        sizer_6.Add((20, 20), 0, 0, 0)
        sizer_29.Add((20, 20), 0, 0, 0)
        sizer_28.Add((20, 10), 0, 0, 0)
        sizer_28.Add(self.list_box_experiment_name, 1, wx.EXPAND, 0)
        sizer_28.Add((20, 20), 0, 0, 0)
        sizer_29_copy.Add(self.button_sim_add, 0, 0, 0)
        sizer_29_copy.Add((10, 20), 0, 0, 0)
        sizer_29_copy.Add(self.button_sim_delete, 0, 0, 0)
        sizer_29_copy.Add((10, 20), 0, 0, 0)
        sizer_29_copy.Add(self.button_sim_copy, 0, 0, 0)
        sizer_28.Add(sizer_29_copy, 0, wx.EXPAND, 0)
        sizer_28.Add((20, 10), 0, 0, 0)
        sizer_29.Add(sizer_28, 1, wx.EXPAND, 0)
        sizer_6.Add(sizer_29, 1, wx.EXPAND, 0)
        sizer_6.Add((20, 20), 0, 0, 0)
        sizer_12.Add((20, 20), 0, 0, 0)
        sizer_30.Add(self.label_9, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_30.Add((20, 20), 0, 0, 0)
        sizer_30.Add(self.text_ctrl_sim_experiment_name, 0, 0, 0)
        sizer_12.Add(sizer_30, 0, wx.EXPAND, 0)
        sizer_12.Add((20, 20), 0, 0, 0)
        sizer_21.Add((5, 20), 0, 0, 0)
        sizer_21.Add(self.label_6, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_21.Add(self.choice_startpos, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_22.Add(sizer_21, 1, wx.EXPAND, 0)
        sizer_23.Add((5, 20), 0, 0, 0)
        sizer_17.Add(self.label_2, 0, 0, 0)
        sizer_17.Add(self.list_box_init_customs, 1, wx.EXPAND, 0)
        sizer_23.Add(sizer_17, 1, wx.EXPAND, 0)
        sizer_23.Add((20, 20), 0, 0, 0)
        sizer_17_copy.Add(self.label_2_copy, 0, 0, 0)
        sizer_17_copy.Add(self.list_box_init_actions, 1, wx.EXPAND, 0)
        sizer_23.Add(sizer_17_copy, 1, wx.EXPAND, 0)
        sizer_23.Add((20, 20), 0, 0, 0)
        sizer_22_copy.Add(self.label_8, 0, 0, 0)
        sizer_22_copy.Add(self.list_box_init_sensors, 1, wx.EXPAND, 0)
        sizer_23.Add(sizer_22_copy, 1, wx.EXPAND, 0)
        sizer_23.Add((5, 20), 0, 0, 0)
        sizer_22.Add(sizer_23, 5, wx.EXPAND, 0)
        sizer_27.Add(sizer_22, 1, wx.ALL|wx.EXPAND, 10)
        grid_sizer_1.Add(self.label_7, 0, wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL, 0)
        grid_sizer_1.Add(self.choice_simtype, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        grid_sizer_1.Add(self.label_7_copy, 0, wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL, 0)
        grid_sizer_1.Add(self.choice_sim_robot, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_18_copy.Add(grid_sizer_1, 1, wx.EXPAND, 0)
        sizer_19.Add((20, 20), 0, 0, 0)
        sizer_19.Add(self.label_4, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_19.Add(self.text_ctrl_xscale, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_19.Add((20, 20), 0, 0, 0)
        sizer_19.Add(self.label_5, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_19.Add(self.text_ctrl_xoffset, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_26.Add(sizer_19, 1, wx.EXPAND, 0)
        sizer_19_copy.Add((20, 20), 0, 0, 0)
        sizer_19_copy.Add(self.label_4_copy, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_19_copy.Add(self.text_ctrl_yscale, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_19_copy.Add((20, 20), 0, 0, 0)
        sizer_19_copy.Add(self.label_5_copy, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_19_copy.Add(self.text_ctrl_yoffset, 0, wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_26.Add(sizer_19_copy, 1, wx.EXPAND, 0)
        sizer_20.Add((20, 20), 0, 0, 0)
        sizer_20.Add(self.button_sim_cali, 0, 0, 0)
        sizer_26.Add(sizer_20, 0, wx.EXPAND, 0)
        sizer_18_copy.Add(sizer_26, 1, wx.EXPAND, 0)
        sizer_27.Add(sizer_18_copy, 1, wx.ALL|wx.EXPAND, 10)
        sizer_12.Add(sizer_27, 1, wx.EXPAND, 0)
        sizer_13.Add(self.button_sim_apply, 0, 0, 0)
        sizer_13.Add((10, 20), 0, 0, 0)
        sizer_13.Add(self.button_sim_ok, 0, 0, 0)
        sizer_13.Add((10, 20), 0, 0, 0)
        sizer_13.Add(self.button_sim_cancel, 0, 0, 0)
        sizer_13.Add((10, 10), 0, 0, 0)
        sizer_12.Add(sizer_13, 0, wx.ALIGN_RIGHT, 0)
        sizer_12.Add((20, 10), 0, 0, 0)
        sizer_6.Add(sizer_12, 2, wx.EXPAND, 0)
        sizer_6.Add((20, 20), 0, 0, 0)
        self.SetSizer(sizer_6)
        self.Layout()
        # end wxGlade

    def loadSimSetup(self, id):
        """ Load the experiment config from the item with index = id in the list"""

        self.text_ctrl_sim_experiment_name.SetValue(self.tempSimSetup[id]['Name'])        

        # Set up the initial actions checklist as appropriate
        self.list_box_init_actions.Set([])
        for i, action in enumerate(self.actions):
            self.list_box_init_actions.Insert(action, i)
            if action in self.tempSimSetup[id]['InitialTruths']:
                self.list_box_init_actions.Check(i)

        # Set up the initial customs checklist as appropriate
        self.list_box_init_customs.Set([])
        for i, custom in enumerate(self.customs):
            self.list_box_init_customs.Insert(custom, i)
            if custom in self.tempSimSetup[id]['InitialTruths']:
                self.list_box_init_customs.Check(i)

        # Set up the initial sensors checklist as appropriate Jim (July 1)
        self.list_box_init_sensors.Set([])
        for i, sensor in enumerate(self.sensors):
            self.list_box_init_sensors.Insert(sensor, i)
            if sensor in self.tempSimSetup[id]['InitialTruths']:
                self.list_box_init_sensors.Check(i)
        
        # Set up the list of starting regions
        self.choice_startpos.Clear()
        if self.parent.rfi is not None:
            for region in self.parent.rfi.regions:
                self.choice_startpos.Append(region.name)
        self.choice_startpos.Select(self.tempSimSetup[id]["InitialRegion"])

        # Set up the list of robots
        self.choice_sim_robot.Clear()
        fileList = os.listdir(os.path.join(os.getcwd(),'robots'))

        for robotFile in fileList:
            if 'robot' == robotFile.split('.')[1] :
                self.choice_sim_robot.Append(robotFile.split('.')[0])
                if robotFile == self.tempSimSetup[id]['RobotFile']:
                    self.choice_sim_robot.Select(self.choice_sim_robot.GetItems().index(robotFile.split('.')[0]))

        
        # Select the simulation type
        if self.tempSimSetup[id]["LabFile"].split('.')[0].lower() == "gazebo":
            self.choice_simtype.Select(0)
        elif self.tempSimSetup[id]["LabFile"].split('.')[0].lower() == "playerstage":
            self.choice_simtype.Select(1)
        elif self.tempSimSetup[id]["LabFile"].split('.')[0].lower() == "cornell_asl":
            self.choice_simtype.Select(2)

        # Load in the coordinate transformation values
        self.text_ctrl_xscale.SetValue(str(self.tempSimSetup[id]["XScale"]))
        self.text_ctrl_xoffset.SetValue(str(self.tempSimSetup[id]["XOffset"]))
        self.text_ctrl_yscale.SetValue(str(self.tempSimSetup[id]["YScale"]))
        self.text_ctrl_yoffset.SetValue(str(self.tempSimSetup[id]["YOffset"]))

    def saveSimSetup(self, id):
        """Temperarily save the data to the copy of simSetup"""

        # Save the experiment name
        self.tempSimSetup[id]['Name'] = self.text_ctrl_sim_experiment_name.GetValue()

        # Add all checked propositions to InitialTruths
        self.tempSimSetup[id]['InitialTruths'] = []

        for i, action in enumerate(self.list_box_init_actions.GetItems()):
            if self.list_box_init_actions.IsChecked(i):
                self.tempSimSetup[id]['InitialTruths'].append(action)

        for i, custom in enumerate(self.list_box_init_customs.GetItems()):
            if self.list_box_init_customs.IsChecked(i):
                self.tempSimSetup[id]['InitialTruths'].append(custom)

        for i, sensor in enumerate(self.list_box_init_sensors.GetItems()):
            if self.list_box_init_sensors.IsChecked(i):
                self.tempSimSetup[id]['InitialTruths'].append(sensor)

        # Update initial region
        self.tempSimSetup[id]['InitialRegion'] = self.choice_startpos.GetSelection()

        # Update robot file name
        self.tempSimSetup[id]['RobotFile'] = self.choice_sim_robot.GetStringSelection()+".robot"

        # Update simulation type
        if self.choice_simtype.GetSelection() == 0:
            self.tempSimSetup[id]["LabFile"] = "Gazebo"
        elif self.choice_simtype.GetSelection() == 1:
            self.tempSimSetup[id]["LabFile"] = "playerstage"
        elif self.choice_simtype.GetSelection() == 2:
            self.tempSimSetup[id]["LabFile"] = "cornell_asl"

        # Update coordinate transformation values
        self.tempSimSetup[id]["XScale"] = float(self.text_ctrl_xscale.GetValue())
        self.tempSimSetup[id]["XOffset"] = float(self.text_ctrl_xoffset.GetValue())
        self.tempSimSetup[id]["YScale"] = float(self.text_ctrl_yscale.GetValue())
        self.tempSimSetup[id]["YOffset"] = float(self.text_ctrl_yoffset.GetValue())
        
        
    def onClickCalibrate(self, event): # wxGlade: simSetupDialog.<event_handler>
        """
        Lets you manually find the correct scale and offset values for the coordinate
        transformation.
        """
        
        # Make sure not simulating already
        # TODO: Update this part
        if self.parent.subprocess[PROCESS_PLAYER] is not None \
        or self.parent.subprocess[PROCESS_GAZEBO] is not None:
            wx.MessageBox("Please close the running simulation before calibrating.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        fileNamePrefix = os.path.join(self.parent.projectPath, self.parent.projectName)

        proc = subprocess.Popen(["python", "calibrate.py", fileNamePrefix + ".spec"],stderr=subprocess.PIPE)
        
        output = proc.stderr.readline().strip()
        while not output.startswith("CALIB"):
            if output != '':
                print output
            output = proc.stderr.readline().strip()

        output = output.split(":")[1]

        # Sanity check to make sure we got good data back
        if len(output.split("\t")) == 4:
            [xScale, xOffset, yScale, yOffset] = output.split("\t")
            self.text_ctrl_xscale.SetValue(xScale)
            self.text_ctrl_xoffset.SetValue(xOffset)
            self.text_ctrl_yscale.SetValue(yScale)
            self.text_ctrl_yoffset.SetValue(yOffset)
        else:
            print "Error getting calibration data from client. Good gracious!"

        event.Skip()
 

    def onClickOK(self, event): # wxGlade: simSetupDialog.<event_handler>
        """
        Adjusts the simSetup data structure to reflect the new configuration
        values.
        """

        ###########################################
        # Simulation configuration dialog cleanup #
        ###########################################

        # Save the current experiment config before load the new one
        name = self.text_ctrl_sim_experiment_name.GetValue()

        for id, config in enumerate(self.tempSimSetup):
            if name == config['Name']:
                self.saveSimSetup(id)

        self.parent.currentExperimentName = self.list_box_experiment_name.GetStringSelection()
        self.parent.simSetup = copy.deepcopy(self.tempSimSetup)  

        event.Skip()
  

    def onSimLoad(self, event): # wxGlade: simSetupDialog.<event_handler>
        """ Load the experiment config data into the configuration dialog when choose an experiment from the list. """

        # Save the current experiment config before load the new one
        name = self.text_ctrl_sim_experiment_name.GetValue()

        for id, config in enumerate(self.tempSimSetup):
            if name == config['Name']:
                self.saveSimSetup(id)
        selection = self.list_box_experiment_name.GetSelection()
        
        if selection == 0:
            self.button_sim_delete.Enable(False)
        else:
            self.button_sim_delete.Enable(True)
        try:
            self.loadSimSetup(selection)
        except:
            print "Cannot load simulation setup for %s. Please check the spec file." % self.list_box_experiment_name.GetStringSelection()
        event.Skip()

    def onSimAdd(self, event): # wxGlade: simSetupDialog.<event_handler>
        """ Add configuration for new experiment"""
        nums = []
        p = re.compile(r"^New\sExperiment(?P<num>\d+)$")
        for name in self.list_box_experiment_name.GetItems():
            m = p.match(name)
            if not m: continue
            nums.append(int(m.group('num')))
        nums.sort()

        # Find smallest available number
        last = 0
        for num in nums:
            if num == last + 1:
                last = num
            elif num == last:
                #print "Warning: Multiple experiment with name 'r%d'." % num
                continue
            else:
                break

        default_name = "New Experiment" + str(last + 1)

        self.list_box_experiment_name.Append(default_name)
        self.tempSimSetup.append(copy.deepcopy(self.tempSimSetup[0]))
        self.tempSimSetup[-1]['Name'] = default_name
        self.list_box_experiment_name.Select(self.list_box_experiment_name.GetCount()-1)
        self.loadSimSetup(self.list_box_experiment_name.GetSelection())
        self.button_sim_delete.Enable(True)
        event.Skip()

    def onSimDelete(self, event): # wxGlade: simSetupDialog.<event_handler>
        """ Delete selected configuration"""
        selection = self.list_box_experiment_name.GetSelection()
        # Remove the data from temperary copy of simSetup
        self.tempSimSetup.pop(selection)
        self.list_box_experiment_name.Delete(selection)


         # Select something reasonable now that the old item is gone
        if selection > 0:
            self.list_box_experiment_name.Select(selection-1)
        else:
            if self.list_box_experiment_name.GetCount() != 0:
                self.list_box_experiment_name.Select(0)



        # Load the simulation setup for the new expriment selected
        selection = self.list_box_experiment_name.GetSelection()
        try:
            self.loadSimSetup(selection)
        except:
            print "Cannot load simulation setup for %s. Please check the spec file." % self.list_box_experiment_name.GetStringSelection()

        event.Skip()

    def onSimCopy(self, event): # wxGlade: simSetupDialog.<event_handler>
        """ Copy an exist experiment configuration """
        selection = self.list_box_experiment_name.GetSelection()
        name = self.list_box_experiment_name.GetStringSelection()
        self.saveSimSetup(selection)

        self.tempSimSetup.append(copy.deepcopy(self.tempSimSetup[selection]))

        self.list_box_experiment_name.Append(name+"_copy")
        self.tempSimSetup[-1]['Name'] = name+"_copy"
        self.list_box_experiment_name.Select(self.list_box_experiment_name.GetCount()-1)
        self.loadSimSetup(self.list_box_experiment_name.GetSelection())
        event.Skip()

    def onClickApply(self, event): # wxGlade: simSetupDialog.<event_handler>
        """
        Adjusts the simSetup data structure to reflect the new configuration
        values.
        """

        # Save the current experiment config before load the new one
        name = self.text_ctrl_sim_experiment_name.GetValue()

        for id, config in enumerate(self.tempSimSetup):
            if name == config['Name']:
                self.saveSimSetup(id)

        self.parent.currentExperimentName = self.list_box_experiment_name.GetStringSelection()
        self.parent.simSetup = copy.deepcopy(self.tempSimSetup)  
        event.Skip()

    def onSimNameEdit(self, event): # wxGlade: simSetupDialog.<event_handler>
        """ Update the experiment name in the list when user edits  it. """
        newName = self.text_ctrl_sim_experiment_name.GetValue()
        selection = self.list_box_experiment_name.GetSelection()
        oldName = self.list_box_experiment_name.GetStringSelection()

        if self.text_ctrl_sim_experiment_name.IsModified() and newName != oldName:
            self.tempSimSetup[selection]['Name'] = newName
            self.list_box_experiment_name.SetString(selection, newName)
        event.Skip()



# end of class simSetupDialog


class MapDialog(wx.Dialog):
    """
    A silly little dialog that displays the regions on top of the map so that you can
    select a region visually instead of just choosing the name.
    """
    
    # TODO: Make the map actually clickable!
    # FIXME: Doesn't scroll on Windows???

    def __init__(self, parent, bitmap, *args, **kwds):
        self.parent = parent
        self.bitmap = bitmap
        # begin wxGlade: MapDialog.__init__
        kwds["style"] = wx.DEFAULT_DIALOG_STYLE
        wx.Dialog.__init__(self, *args, **kwds)
        self.panel_2 = wx.ScrolledWindow(self, -1, style=wx.TAB_TRAVERSAL)

        self.__set_properties()
        self.__do_layout()
        # end wxGlade
        

    def __set_properties(self):
        # begin wxGlade: MapDialog.__set_properties
        self.SetTitle("Select Region...")
        self.SetSize((932, 709))
        self.panel_2.SetScrollRate(10, 10)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MapDialog.__do_layout
        sizer_10 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_11 = wx.BoxSizer(wx.HORIZONTAL)
        bitmap_1 = wx.StaticBitmap(self.panel_2, -1, (self.bitmap))
        sizer_11.Add(bitmap_1, 1, wx.EXPAND, 0)
        self.panel_2.SetSizer(sizer_11)
        sizer_10.Add(self.panel_2, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_10)
        self.Layout()
        self.Centre()
        # end wxGlade

        # Bind to catch mouse clicks!
        bitmap_1.Bind(wx.EVT_LEFT_DOWN, self.onMapClick)

    def onMapClick(self, event):
        x, y = self.panel_2.CalcUnscrolledPosition(event.GetX(), event.GetY())
        for region in self.parent.rfi.regions:
            if region.objectContainsPoint(x, y):
                self.parent.text_ctrl_spec.WriteText(region.name)
                #self.EndModal(1)
                self.Close()
            break 
        event.Skip()

# end of class MapDialog


class SpecEditorFrame(wx.Frame):
    """
    The main application window!
    """

    # TODO: Are we gonna allow MDI?  What's the point?
    # TODO: Use the scintilla-style text editor. Select-all, search-replace, syntax highlighting, etc.

    def __init__(self, *args, **kwds):
        # begin wxGlade: SpecEditorFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.window_1 = wx.SplitterWindow(self, -1, style=wx.SP_3D|wx.SP_BORDER|wx.SP_LIVE_UPDATE)
        self.window_1_pane_2 = wx.Panel(self.window_1, -1)
        self.notebook_1 = wx.Notebook(self.window_1_pane_2, -1, style=0)
        self.notebook_1_pane_2 = wx.Panel(self.notebook_1, -1)
        self.notebook_1_pane_1 = wx.Panel(self.notebook_1, -1)
        self.window_1_pane_1 = wx.Panel(self.window_1, -1)
        self.panel_1 = wx.ScrolledWindow(self.window_1_pane_1, -1, style=wx.TAB_TRAVERSAL)
        
        # Menu Bar
        self.frame_1_menubar = wx.MenuBar()
        global MENU_IMPORT; MENU_IMPORT = wx.NewId()
        global MENU_IMPORT_ROBOT; MENU_IMPORT_ROBOT = wx.NewId()
        global MENU_IMPORT_REGION; MENU_IMPORT_REGION = wx.NewId()
        global MENU_COMPILE; MENU_COMPILE = wx.NewId()
        global MENU_SIMCONFIG; MENU_SIMCONFIG = wx.NewId()
        global MENU_SIMULATE; MENU_SIMULATE = wx.NewId()
        global MENU_DOTTY; MENU_DOTTY = wx.NewId()
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(wx.ID_NEW, "&New\tCtrl-N", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_OPEN, "&Open...\tCtrl-O", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_CLOSE, "&Close\tCtrl-W", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu_sub = wx.Menu()
        wxglade_tmp_menu_sub.Append(MENU_IMPORT_ROBOT, "Robot &Description File...\tCtrl-D", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu_sub.Append(MENU_IMPORT_REGION, "&Region File...\tCtrl-R", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.AppendMenu(MENU_IMPORT, "&Import", wxglade_tmp_menu_sub, "")
        wxglade_tmp_menu.Append(wx.ID_SAVE, "&Save\tCtrl-S", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_SAVEAS, "Save &As...", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_EXIT, "&Quit\tCtrl-Q", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&File")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(wx.ID_UNDO, "&Undo\tCtrl-Z", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_REDO, "&Redo\tCtrl-Shift-Z", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.AppendSeparator()
        wxglade_tmp_menu.Append(wx.ID_CUT, "Cu&t\tCtrl-X", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_COPY, "&Copy\tCtrl-C", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_PASTE, "&Paste\tCtrl-V", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Edit")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(MENU_COMPILE, "&Compile\tF5", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_SIMCONFIG, "Confi&gure Simulation...\tShift-F6", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_SIMULATE, "&Simulate\tF6", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_DOTTY, "View &Automaton\tF9", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Run")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(wx.ID_ABOUT, "&About Specification Editor...", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Help")
        self.SetMenuBar(self.frame_1_menubar)
        # Menu Bar end
        self.text_ctrl_spec = wx.richtext.RichTextCtrl(self.window_1_pane_1, -1, "", style=wx.TE_PROCESS_ENTER|wx.TE_PROCESS_TAB|wx.TE_MULTILINE)
        self.label_1 = wx.StaticText(self.panel_1, -1, "Regions:")
        self.list_box_regions = wx.ListBox(self.panel_1, -1, choices=[], style=wx.LB_SINGLE)
        self.button_map = wx.Button(self.panel_1, -1, "Select from Map...")
        self.button_edit_regions = wx.Button(self.panel_1, -1, "Edit Regions...")
        self.label_1_copy = wx.StaticText(self.panel_1, -1, "Sensors:")
        self.list_box_sensors = wx.CheckListBox(self.panel_1, -1, choices=[], style=wx.LB_SINGLE)
        self.label_1_copy_1 = wx.StaticText(self.panel_1, -1, "Actions:")
        self.list_box_actions = wx.CheckListBox(self.panel_1, -1, choices=[], style=wx.LB_SINGLE)
        self.label_1_copy_2 = wx.StaticText(self.panel_1, -1, "Custom Propositions:")
        self.list_box_customs = wx.ListBox(self.panel_1, -1, choices=[], style=wx.LB_SINGLE)
        self.button_custom_new = wx.Button(self.panel_1, -1, "New...")
        self.button_custom_delete = wx.Button(self.panel_1, -1, "Delete")
        self.text_ctrl_log = wx.richtext.RichTextCtrl(self.notebook_1_pane_1, -1, "", style=wx.TE_MULTILINE|wx.TE_READONLY)
        self.text_ctrl_LTL = wx.TextCtrl(self.notebook_1_pane_2, -1, "", style=wx.TE_MULTILINE|wx.TE_READONLY)

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_MENU, self.onMenuNew, id=wx.ID_NEW)
        self.Bind(wx.EVT_MENU, self.onMenuOpen, id=wx.ID_OPEN)
        self.Bind(wx.EVT_MENU, self.doClose, id=wx.ID_CLOSE)
        self.Bind(wx.EVT_MENU, self.onImportRobot, id=MENU_IMPORT_ROBOT)
        self.Bind(wx.EVT_MENU, self.onImportRegion, id=MENU_IMPORT_REGION)
        self.Bind(wx.EVT_MENU, self.onMenuSave, id=wx.ID_SAVE)
        self.Bind(wx.EVT_MENU, self.onMenuSaveAs, id=wx.ID_SAVEAS)
        self.Bind(wx.EVT_MENU, self.onMenuQuit, id=wx.ID_EXIT)
        self.Bind(wx.EVT_MENU, self.onMenuUndo, id=wx.ID_UNDO)
        self.Bind(wx.EVT_MENU, self.onMenuRedo, id=wx.ID_REDO)
        self.Bind(wx.EVT_MENU, self.onMenuCut, id=wx.ID_CUT)
        self.Bind(wx.EVT_MENU, self.onMenuCopy, id=wx.ID_COPY)
        self.Bind(wx.EVT_MENU, self.onMenuPaste, id=wx.ID_PASTE)
        self.Bind(wx.EVT_MENU, self.onMenuCompile, id=MENU_COMPILE)
        self.Bind(wx.EVT_MENU, self.onMenuConfigSim, id=MENU_SIMCONFIG)
        self.Bind(wx.EVT_MENU, self.onMenuSimulate, id=MENU_SIMULATE)
        self.Bind(wx.EVT_MENU, self.onMenuViewAut, id=MENU_DOTTY)
        self.Bind(wx.EVT_MENU, self.onMenuAbout, id=wx.ID_ABOUT)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_regions)
        self.Bind(wx.EVT_BUTTON, self.onMapSelect, self.button_map)
        self.Bind(wx.EVT_BUTTON, self.onClickEditRegions, self.button_edit_regions)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_sensors)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_actions)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_customs)
        self.Bind(wx.EVT_BUTTON, self.onCustomNew, self.button_custom_new)
        self.Bind(wx.EVT_BUTTON, self.onCustomDelete, self.button_custom_delete)
        # end wxGlade

        # Set up extra event bindings
        self.Bind(wx.EVT_CLOSE, self.doClose)
        self.Bind(wx.EVT_END_PROCESS, self.onProcessTerminate)

        # Initialize values
        self.mapDialog = None
        self.fileName = None
        self.projectName = None
        self.projectPath = ""
        self.projectFiles = {}
        self.rfi = None
        self.subprocess = [None] * 4
        self.simSetup = []
        self.currentExperimentName = ""
        self.currentExperimentConfig = {}

        global PROCESS_REGED; PROCESS_REGED = 0
        global PROCESS_PLAYER; PROCESS_PLAYER = 1
        global PROCESS_GAZEBO; PROCESS_GAZEBO = 2
        global PROCESS_DOTTY; PROCESS_DOTTY = 3

        # Set default values
        self.setDefaults()

    def __set_properties(self):
        # begin wxGlade: SpecEditorFrame.__set_properties
        self.SetTitle("Specification Editor - Untitled")
        self.SetSize((900, 700))
        self.text_ctrl_spec.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL, 0, ""))
        self.button_map.Enable(False)
        self.list_box_sensors.SetMinSize((123, 75))
        self.list_box_actions.SetMinSize((123, 75))
        self.list_box_customs.SetMinSize((123, 75))
        self.button_custom_delete.Enable(False)
        self.panel_1.SetScrollRate(10, 10)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: SpecEditorFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_9 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_3 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5 = wx.BoxSizer(wx.VERTICAL)
        sizer_8 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_7 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4.Add(self.text_ctrl_spec, 2, wx.EXPAND, 0)
        sizer_5.Add(self.label_1, 0, wx.LEFT|wx.TOP|wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_regions, 2, wx.LEFT|wx.EXPAND, 4)
        sizer_7.Add(self.button_map, 0, wx.TOP, 5)
        sizer_7.Add((5, 20), 0, 0, 0)
        sizer_7.Add(self.button_edit_regions, 0, wx.TOP, 5)
        sizer_5.Add(sizer_7, 0, wx.LEFT|wx.EXPAND, 4)
        sizer_5.Add(self.label_1_copy, 0, wx.LEFT|wx.TOP|wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_sensors, 2, wx.LEFT|wx.EXPAND, 4)
        sizer_5.Add(self.label_1_copy_1, 0, wx.LEFT|wx.TOP|wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_actions, 2, wx.LEFT|wx.EXPAND, 4)
        sizer_5.Add(self.label_1_copy_2, 0, wx.LEFT|wx.TOP|wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_customs, 2, wx.LEFT|wx.EXPAND, 4)
        sizer_8.Add(self.button_custom_new, 0, wx.TOP, 5)
        sizer_8.Add((5, 20), 0, 0, 0)
        sizer_8.Add(self.button_custom_delete, 0, wx.TOP, 5)
        sizer_5.Add(sizer_8, 0, wx.LEFT|wx.EXPAND, 4)
        self.panel_1.SetSizer(sizer_5)
        sizer_4.Add(self.panel_1, 1, wx.EXPAND, 0)
        self.window_1_pane_1.SetSizer(sizer_4)
        sizer_3.Add(self.text_ctrl_log, 1, wx.ALL|wx.EXPAND, 2)
        self.notebook_1_pane_1.SetSizer(sizer_3)
        sizer_9.Add(self.text_ctrl_LTL, 1, wx.EXPAND, 0)
        self.notebook_1_pane_2.SetSizer(sizer_9)
        self.notebook_1.AddPage(self.notebook_1_pane_1, "Compiler Log")
        self.notebook_1.AddPage(self.notebook_1_pane_2, "LTL Output")
        sizer_2.Add(self.notebook_1, 1, wx.EXPAND, 0)
        self.window_1_pane_2.SetSizer(sizer_2)
        self.window_1.SplitHorizontally(self.window_1_pane_1, self.window_1_pane_2, 500)
        sizer_1.Add(self.window_1, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_1)
        self.Layout()
        self.Centre()
        # end wxGlade
        
        # Make it so that the log window doesn't change height when the window is resized
        # NOTE: May not work on older versions of wxWidgets
        self.window_1.SetSashGravity(1.0)

    def onPropositionDblClick(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Add the proposition name to the spec when you double click the name
        """

        caller = event.GetEventObject()
        if caller in [self.list_box_sensors, self.list_box_actions] \
           and not caller.IsChecked(caller.GetSelection()):
            # Only allow adding of enabled propositions
            return
        self.text_ctrl_spec.WriteText(caller.GetStringSelection())

        event.Skip()

    def onCustomNew(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Display a dialog asking for a proposition name and then add it to the custom proposition list.
        """

        # Find any existing custom propositions of name customX
        nums = []
        p = re.compile(r"^custom(?P<num>\d+)$")
        for name in self.list_box_customs.GetItems():
            m = p.match(name)
            if not m: continue
            nums.append(int(m.group('num')))
        nums.sort()

        # Find smallest available number
        last = 0
        for num in nums:
            if num == last + 1:
                last = num
            elif num == last:
                #print "Warning: Multiple propositions with name 'r%d'." % num
                continue
            else:
                break

        default_name = "custom" + str(last + 1)

        # Ask the user for a proposition name, suggesting the next available one as default
        name = wx.GetTextFromUser("Name:", "New Custom Proposition", default_name)
        
        if name != "":
            # If it's valid, add it, select it and enable it
            self.list_box_customs.Insert(name, self.list_box_customs.GetCount())
            self.list_box_customs.Select(self.list_box_customs.GetCount()-1)
            self.button_custom_delete.Enable(True)

        event.Skip()

    def onCustomDelete(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Remove the selected custom proposition.
        """

        selection = self.list_box_customs.GetSelection()
        self.list_box_customs.Delete(selection)
        
        # Select something reasonable now that the old item is gone
        if selection > 0:
            self.list_box_customs.Select(selection-1)
        else:
            if self.list_box_customs.GetCount() != 0:
                self.list_box_customs.Select(0)

        # Disable the Delete button if there's nothing to delete
        if self.list_box_customs.GetCount() == 0:
            self.button_custom_delete.Enable(False)

        event.Skip()

    def onMapSelect(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Show the map with overlayed regions so that the user can select a region name visually.
        """

        if self.mapDialog is not None:
            self.mapDialog.Show()
        # TODO: Finish me

        event.Skip()

    def onImportRobot(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Ask the user for a robot description file and then import it.
        """

        fileName = wx.FileSelector("Import Robot Description File", default_extension="robot",
                                  wildcard="Robot description files (*.robot)|*.robot",
                                  flags = wx.OPEN | wx.FILE_MUST_EXIST)
        if fileName == "": return
        self.readRobotFile(fileName)

    def readRobotFile(self, fileName):
        """
        Parse in a robot description file!
        """

        data = fileMethods.readFromFile(fileName)

        # Make sure the file is good:
        if data is None or 'Sensors' not in data or 'Actions' not in data:
            wx.MessageBox("Cannot open robot file %s" % (fileName), "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        # Add the sensors and actions to the listboxes;
        # assume everything is enabled by default
        self.list_box_sensors.SetItems(data['Sensors'])
        for i in xrange(self.list_box_sensors.GetCount()):
            self.list_box_sensors.Check(i)

        self.list_box_actions.SetItems(data['Actions'])
        for i in xrange(self.list_box_actions.GetCount()):
            self.list_box_actions.Check(i)

        # Remember the robot description file name
        self.projectFiles['RobotFile'] = os.path.basename(fileName)

    def onImportRegion(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Ask the user for a region file and then import it.
        """
        fileName = wx.FileSelector("Import Region File", default_extension="regions",
                                  wildcard="Region files (*.regions)|*.regions",
                                  flags = wx.OPEN | wx.FILE_MUST_EXIST)
        if fileName == "": return
        
        self.loadRegionFile(fileName)

    def loadRegionFile(self, fileName):
        """
        Parse in a region file!
        """

        # Create a RFI if necessary
        if self.rfi is None:
            self.rfi = RegionFileInterface()

        # Try loading the file
        if not self.rfi.readFile(fileName):
            wx.MessageBox("Cannot open region file %s" % (fileName), "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        # Update the path information
        # NOTE: At least for now, all project files must be in the same directory.
        self.projectFiles["RegionFile"] = os.path.basename(fileName)

        # Add the regions to the region listbox
        self.list_box_regions.Set([])
        for i, region in enumerate(self.rfi.regions):
            self.list_box_regions.Insert(region.name, i)        
        
        # If we are working with an unsaved spec, assume everything is in the same dir
        # as the regions file
        if self.projectPath == "":
            self.projectPath = os.path.dirname(os.path.abspath(fileName))

        # Create the map selection dialog
        #image = wx.ImageFromData(*rfi.thumb)
        bitmap = wx.EmptyBitmap(1, 1)
        bitmap.LoadFile(os.path.join(self.projectPath, self.rfi.thumb), wx.BITMAP_TYPE_PNG)
        if self.mapDialog is not None:
            self.mapDialog.Destroy()
        self.mapDialog = MapDialog(frame_1, bitmap, frame_1)
        frame_1.button_map.Enable(True)

    def onMenuNew(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Create a new specification.
        """

        # TODO: Make new window here?
        pass

        event.Skip()

    def onMenuOpen(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Ask the user for a specification file to open, and then open it.
        """

        fileName = wx.FileSelector("Open File", default_extension="spec",
                                  wildcard="Specification files (*.spec)|*.spec",
                                  flags = wx.OPEN | wx.FILE_MUST_EXIST)
        if fileName == "": return

        #event.Skip()

    def onMenuSave(self, event=None): # wxGlade: SpecEditorFrame.<event_handler>
        """
        If the file has been saved already, save it quietly.
        Else, ask for a filename and then save it.
        """

        if self.fileName == None:
            self.onMenuSaveAs(event)
        else:
            self.saveFile(self.fileName)

    def onMenuSaveAs(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Ask the user for a filename to save the specification as, and then save it.
        """

        # If this isn't the first save, set the current filename as default
        if self.fileName == None:
            default = ""
        else:
            default = self.fileName

        # Get a filename
        fileName = wx.FileSelector("Save File As", "Saving",
                                  default_filename=default,
                                  default_extension="spec",
                                  wildcard="Specification files (*.spec)|*.spec",
                                  flags = wx.SAVE | wx.OVERWRITE_PROMPT)
        if fileName == "": return # User cancelled.

        # Force a .spec extension.  How mean!!!
        if os.path.splitext(fileName)[1] != ".spec":
            fileName = fileName + ".spec"
        
        # Update the window title
        title = os.path.basename(fileName)
        self.SetTitle("Specification Editor - " + title)

        # Update paths
        self.fileName = fileName
        self.projectName = os.path.splitext(title)[0]
        self.projectPath = os.path.dirname(os.path.abspath(fileName))

        # Save data to the file
        self.saveFile(fileName)

    def saveFile(self, fileName):
        """
        Write all data out to a file.
        """

        if fileName is None:
            return
        
        # Only store enabled sensors
        sensorList = []
        for i, sensor in enumerate(self.list_box_sensors.GetItems()):
            if self.list_box_sensors.IsChecked(i):
                sensorList.append(sensor)
                

        data = {'SPECIFICATION':{}, 'SETTINGS':{}}
        data['SPECIFICATION'] = {'Spec': str(self.text_ctrl_spec.GetValue())}
        data['SETTINGS'] = {"RegionFile": self.projectFiles["RegionFile"],
                            "Sensors": self.dumpListBox(self.list_box_sensors),
                            "Actions": self.dumpListBox(self.list_box_actions),
                            "Customs": self.dumpListBox(self.list_box_customs),
                            "currentExperimentName": self.currentExperimentName,
                            }
        if len(self.simSetup)>1:
            for id, config in enumerate(self.simSetup):
                if config['Name'] != 'Default':
                    expConfig = {   "Name": config['Name'],
                                    "RobotFile": config['RobotFile'],
                                    "Lab": config['LabFile'],
                                    "Calibration": ",".join(map(str,[config["XScale"], config["XOffset"],
                                                        config["YScale"], config["YOffset"]])),
                                    "InitialTruths": config['InitialTruths'],
                                    "InitialRegion": config['InitialRegion']
                                }
                    data['EXPERIMENT CONFIG '+str(id)] = expConfig


        comments = {"FILE_HEADER": "This is a specification definition file for the LTLMoP toolkit.\n" +
                                   "Format details are described at the beginning of each section below.\n" +
                                   "Note that all values are separated by *tabs*.",
                    "RegionFile": "Relative path of region description file",
                    "RobotFile": "Relative path of robot description file",
                    "Sensors": "List of sensors and their state (enabled = 1, disabled = 0)",
                    "Actions": "List of actions and their state (enabled = 1, disabled = 0)",
                    "Customs": "List of custom propositions",
                    "InitialTruths": "List of initially true propositions",
                    "InitialRegion": "Initial region number",
                    "Calibration": "Coordinate transformation between map and experiment: XScale, XOffset, YScale, YOffset",
                    "Lab": 'Lab configuration file',
                    "Spec": "Specification in simple English",
                    "Name": 'Name of the experiment'}

        fileMethods.writeToFile(fileName, data, comments)
            
    def dumpListBox(self, list):
        data = []
        for i, item in enumerate(list.GetItems()):
            if hasattr(list, "IsChecked"):
                if list.IsChecked(i):
                    check = "1"
                else:
                    check = "0"
                data.append(item+","+check)
            else:
                data.append(item)

        return data

    def loadList(self, data, list):
        if len(data) == 0:
            return

        list.SetItems([])
        for i, item in enumerate(data):
            if ',' in item:
                [name, check] = item.split(',')
                list.Insert(name, i)
                if check == "1":
                    list.Check(i)
            else:
                list.Insert(item, i)

    def setDefaults(self):
        """
        Set some pretty reasonable default values.
        """
        self.projectFiles["RegionFile"] = ""
        self.projectFiles["RobotFile"] = ""
        self.currentExperimentName = 'Default'
        self.simSetup = [{  "Name": "Default",
                            "RobotFile": "",
                            "LabFile": "",
                            "XScale": 1.0,
                            "XOffset": 0.0,
                            "YScale": 1.0,
                            "YOffset": 0.0,
                            "InitialTruths": [],
                            "InitialRegion": 0 }]
        self.text_ctrl_spec.SetValue("")

        # Null the subprocess values
        self.subprocess[PROCESS_REGED] = None
        self.subprocess[PROCESS_PLAYER] = None
        self.subprocess[PROCESS_GAZEBO] = None
        self.subprocess[PROCESS_DOTTY] = None

    def openFile(self, fileName):

        if fileName is None:
            return

        proj = project.Project()
        proj.loadProject(fileName)
 
        #data = fileMethods.readFromFile(fileName )
        data = proj.spec_data
        if data is None:
            wx.MessageBox("Cannot open specification file %s" % (fileName), "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return


        self.setDefaults()

        filePath = os.path.dirname(os.path.abspath(fileName))

        for name, content in data.iteritems():
            if name == 'SPECIFICATION' and 'Spec' in content and len(content['Spec']) > 0: 
                self.text_ctrl_spec.SetValue("\n".join(content['Spec']))

            elif name =='SETTINGS':
                if 'RegionFile' in content and len(content['RegionFile']) > 0:
                    self.loadRegionFile(os.path.join(filePath, content['RegionFile'][0]))
                if 'Actions' in content:
                    self.loadList(content['Actions'], self.list_box_actions)
                if 'Customs' in content:
                    self.loadList(content['Customs'], self.list_box_customs)
                    if len(content['Customs']) > 0:
                        self.button_custom_delete.Enable(True)
                        self.list_box_customs.Select(0)
                if 'Sensors' in content:
                    self.loadList(content['Sensors'], self.list_box_sensors)
                if 'currentExperimentName' in content and len(content['currentExperimentName']) > 0:
                    self.currentExperimentName = content['currentExperimentName'][0]
                    

            elif 'EXPERIMENT CONFIG' in name:
                self.simSetup.append({})
                if 'Name' in content and len(content['Name']) > 0:
                    self.simSetup[-1]["Name"] = content['Name'][0]
                if 'RobotFile' in content and len(content['RobotFile']) > 0:
                    self.simSetup[-1]["RobotFile"] = content['RobotFile'][0]
                if 'InitialTruths' in content:
                    self.simSetup[-1]['InitialTruths'] = content['InitialTruths']
                if 'InitialRegion' in content and len(content['InitialRegion']) > 0:
                    self.simSetup[-1]['InitialRegion'] = int(content['InitialRegion'][0])
                if 'Calibration' in content and len(content['Calibration'][0].split(","))==4:
                    [xScale, xOffset, yScale, yOffset] = content['Calibration'][0].split(",")
                    self.simSetup[-1]['XScale'] = float(xScale)
                    self.simSetup[-1]['XOffset'] = float(xOffset)
                    self.simSetup[-1]['YScale'] = float(yScale)
                    self.simSetup[-1]['YOffset'] = float(yOffset)
                      
                if 'Lab' in content and len(content['Lab']) > 0:
                    self.simSetup[-1]['LabFile'] = content['Lab'][0]

        # Update the window title
        title = os.path.basename(fileName)
        self.SetTitle("Specification Editor - " + title)

        # Update paths
        self.fileName = fileName
        self.projectName = os.path.splitext(title)[0]
        self.projectPath = filePath
    
    def doClose(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Respond to the "Close" menu command.
        """

        # TODO: if self.dirty:
        if not self.askIfUserWantsToSave("closing"): return
        
        # Detach from any running subprocesses
        for process in self.subprocess: 
            if process is not None:
                process.Detach()

        self.Destroy()

    def askIfUserWantsToSave(self, action):
        """ Give the user the opportunity to save the current document.

            'action' is a string describing the action about to be taken.  If
            the user wants to save the document, it is saved immediately.  If
            the user cancels, we return False.
            
            From pySketch example
        """

        response = wx.MessageBox("Save changes before " + action + "?",
                                "Confirm", wx.YES_NO | wx.CANCEL, self)

        if response == wx.YES:
            self.onMenuSave()
            return True
        elif response == wx.NO:
            return True # User doesn't want changes saved.
        elif response == wx.CANCEL:
            return False # User cancelled.

    def onMenuUndo(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanUndo():
            self.text_ctrl_spec.Undo()
        event.Skip()

    def onMenuRedo(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanRedo():
            self.text_ctrl_spec.Redo()
        event.Skip()

    def onMenuCut(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanCut():
            self.text_ctrl_spec.Cut()
        event.Skip()

    def onMenuCopy(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanCopy():
            self.text_ctrl_spec.Copy()
        event.Skip()

    def onMenuPaste(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanPaste():
            self.text_ctrl_spec.Paste()
        event.Skip()

    def onMenuCompile(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # Let's make sure we have everything!

        if self.rfi is None:
            wx.MessageBox("Please define regions before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return
    
        if self.text_ctrl_spec.GetValue() == "":
            wx.MessageBox("Please write a specification before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return


        # Clear the log so we can start fresh grocer
        self.text_ctrl_log.Clear()

        # Redirect all output to the log
        redir = RedirectText(self.text_ctrl_log)

        sys.stdout = redir
        sys.stderr = redir

        ###################
        # Create SMV File #
        ###################

        self.appendLog("Creating SMV file...\n", "BLUE")
        wx.Yield()

        numRegions = len(self.rfi.regions)

        sensorList = []
        for i, sensor in enumerate(self.list_box_sensors.GetItems()):
            if self.list_box_sensors.IsChecked(i):
                sensorList.append(sensor)

        robotPropList = []
        for i, action in enumerate(self.list_box_actions.GetItems()):
            if self.list_box_actions.IsChecked(i):
                robotPropList.append(action)
        robotPropList.extend(self.list_box_customs.GetItems())

        fileNamePrefix = os.path.join(self.projectPath, self.projectName)

        createSMVfile(fileNamePrefix, numRegions, sensorList, robotPropList)

        ##################
        # Create PF File #
        ##################

        self.appendLog("Creating PF file...\n", "BLUE")
        wx.Yield()
        text = self.text_ctrl_spec.GetValue()
        regionList = [x.name for x in self.rfi.regions]

        spec = writeSpec(text, sensorList, regionList, robotPropList)
        # TODO: Catch errors here
        adjData = self.rfi.transitions

        createPFfile(fileNamePrefix, sensorList, robotPropList, adjData, spec)
        if os.path.exists(fileNamePrefix+".pf"):
            f = open(fileNamePrefix+".pf","r")
            ltl = "".join(f.readlines())
            f.close()
            self.text_ctrl_LTL.SetValue(ltl)

        ####################
        # Create automaton #
        ####################

        self.appendLog("Creating automaton...\n", "BLUE")
        wx.Yield()

        wd = os.getcwd()
        os.chdir(os.path.normpath(os.path.join(".","TLV")))

        # Oh how many hours I've wasted trying to get this to output to the log live.
        # I give up.


        cmd = subprocess.Popen([os.path.join(".","tlv"), fileNamePrefix + ".pf",
              fileNamePrefix + ".smv"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True)
        cmd.stdin.write("1\n")
        cmd.stdin.close()
    
        while cmd.poll():
            wx.Yield()

        realizable = False

        # For some reason TLV outputs a huge number of ^M characters, which we need to
        # filter out.  I hope this doesn't cause problems on Windows.
        for line in cmd.stdout:
            if "\015" not in line:
                self.appendLog("\t"+line)
            if "Specification is realizable." in line:
               realizable = True

        cmd.stdout.close()
        print "\n"

        if realizable:
            self.appendLog("Automaton successfully synthesized.\n", "GREEN")
        else:
            self.appendLog("ERROR: Specification was unrealizable.\n", "RED")

        os.chdir(wd)

        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__


    def appendLog(self, text, color="BLACK"):
        self.text_ctrl_log.BeginTextColour(color)
        #self.text_ctrl_log.BeginBold()
        self.text_ctrl_log.AppendText(text)
        #self.text_ctrl_log.EndBold()
        self.text_ctrl_log.EndTextColour()
        self.text_ctrl_log.ShowPosition(self.text_ctrl_log.GetLastPosition())

    def writeSimConfig(self, calib=False):
        fileNamePrefix = os.path.join(self.projectPath, self.projectName)

        fwd_coordmap = lambda pt: (self.simSetup['XScale'] * pt.x + self.simSetup['XOffset'],
                                   self.simSetup['YScale'] * pt.y + self.simSetup['YOffset'])

        if calib:
            # Seems like a reasonable place to start, no?
            startpos = wx.Point(0,0)
        else:
            # Start in the center of the defined initial region
            startpos = fwd_coordmap(self.rfi.regions[self.simSetup['InitialRegion']].getCenter())

        # Choose an appropriate background image
        if self.simSetup['BackgroundOverlay']:
            bgFile = fileNamePrefix + "_simbg.png"
        else:
            bgFile = os.path.normpath(os.path.join(self.projectPath, self.rfi.background))

        f_world = open(fileNamePrefix + ".world", "w")

        if self.simSetup['SimType'].lower() == 'stage':

            ####################
            # Stage world file #
            ####################

            f_world.write("""
# Just a really simple robot abstraction
define pointbot position
(
  # Actual size
  size [0.33 0.33]

  # Show the front
  gui_nose 1

  # Just a silly box
  polygons 1
  polygon[0].points 4
  polygon[0].point[0] [  0  0 ]
  polygon[0].point[1] [  0  1 ]
  polygon[0].point[2] [  1  1 ]
  polygon[0].point[3] [  1  0 ]

  # Simplify as holonomic robot
  drive "omni"
)

# Defines "map" object used for floorplans
define map model
(
  # sombre, sensible, artistic
  color "black"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 1
  gui_movemask 0
  gui_outline 0

  gripper_return 0
)

# size of the world in meters
size [16 12]

# set the resolution of the underlying raytrace model in meters
resolution 0.02

# update the screen every 10ms 
gui_interval 20

# configure the GUI window
window
(
  size [ 591.000 638.000 ]
  center [0 0]
  scale 0.028
)

# load an environment bitmap
map
(
  bitmap "%s"
  size [16 12]
  name "example1"
  boundary 0
  obstacle_return 0
)

# create a robot
pointbot
(
  name "robot1"
  color "red"
  pose [%f %f 0]

  localization "gps"
  localization_origin [0 0 0]
)
        """ % (bgFile, startpos[0], startpos[1]))
            if self.simSetup["VertexMarkers"]:
                f_world.write("""          
define puck model(
  size [ 0.08 0.08 ]
  gripper_return 1
  gui_movemask 3
  gui_nose 0
)
                """)
                pts = []
                for region in self.rfi.regions:
                    for pt in region.getPoints():
                        if pt not in pts:
                            f_world.write("puck( pose [%f %f 0.0 ] color \"red\" )\n" % fwd_coordmap(pt))

                # TODO: Draw me some sensors!?

        elif self.simSetup['SimType'].lower() == 'gazebo':

            #####################
            # Gazebo world file #
            #####################

            f_world.write("""\
<?xml version="1.0"?>

<gz:world 
  xmlns:gz='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz'
  xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model'
  xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor'
  xmlns:window='http://playerstage.sourceforge.net/gazebo/xmlschema/#window'
  xmlns:param='http://playerstage.sourceforge.net/gazebo/xmlschema/#params'
  xmlns:ui='http://playerstage.sourceforge.net/gazebo/xmlschema/#params'>

  <param:Global>
    <skyColor>0 0 0</skyColor>
  </param:Global>

  <model:ObserverCam>
    <id>userCam0</id>
    <xyz>0 0 16.0</xyz>
    <rpy>0 90 90</rpy>
    <imageSize>640 480</imageSize>
    <updateRate>10</updateRate>
  </model:ObserverCam>

  <model:LightSource>
    <id>light1</id>
    <xyz>10.0 -10.0 100.0</xyz>
  </model:LightSource>

  <model:ImmovableObject>
    <plugin>immovableObject.so</plugin>
    <id>map_picture</id>
    <xyz>0.0 0.0 0.0</xyz>
    <rpy>0 0 0</rpy>
    <size>24 18 0.0</size>
    <textureFile>%s</textureFile>
  </model:ImmovableObject>

  <model:Scarab>
    <plugin>Scarab.so</plugin>
    <xyz>%f %f 0.23</xyz>
    <rpy>0.0 0.0 0.0</rpy>
    <id>robot1</id>
    <bodyColor>0 1 0</bodyColor>
    <wheelColor>1 0 0</wheelColor>
    <model:TruthWidget>
      <id>robot1_truth</id>
    </model:TruthWidget>
  </model:Scarab>
            """ % (bgFile, startpos[0], startpos[1]))

            # TODO: Account for aspect ratio of background image!

            if self.simSetup["VertexMarkers"]:
                pts = []
                for region in self.rfi.regions:
                    for pt in region.getPoints():
                        if pt not in pts:
                            f_world.write("""
<model:ImmovableObject>
    <xyz>%f %f 0.0</xyz>
    <rpy>0 0 0</rpy>
    <size>0.1 0.1 0.1</size>
    <color>1 0 0</color>
</model:ImmovableObject>
                            """ % fwd_coordmap(pt))
            f_world.write("</gz:world>\n")


                # TODO: Draw me some sensors!?

        f_world.close()

        f_cfg = open(fileNamePrefix + ".cfg", "w")

        if self.simSetup['SimType'].lower() == 'stage':

            ############################
            # Stage configuration file #
            ############################

            f_cfg.write("""
# Load the Stage plugin simulation driver
driver
(
  name "stage"
  provides ["simulation:0" ]
  plugin "libstageplugin"

  # load the named file into the simulator
  worldfile "%s.world"
)

driver
(
  name "stage"
  provides ["map:0"]
  model "example1"
)

# Create a Stage driver and attach position2d interfaces 
# to the model "robot1"
driver
(
  name "stage"
  provides ["position2d:0"]
  model "robot1"
)
""" % fileNamePrefix)

        elif self.simSetup['SimType'].lower() == 'gazebo':

            #############################
            # Gazebo configuration file #
            #############################
            f_cfg.write("""
driver
(
  name "gazebo"
  provides ["simulation:0"]
  plugin "libgazeboplugin"
  server_id "default"
  alwayson 1
)

driver
(
  name "gazebo"
  provides ["position2d:0"]
  gz_id "robot1"
)

driver
(
  name "gazeboTrack"
  plugin "gazeboTrack"
  provides ["fiducial:0"]
  num_target 1
  # Make sure that you are adding gazebo truth widgets to the robots you wish 
  # track in gazebo.
  # Format:
  # target_id_<index> "id from gazebo truth model"
  target_id_0 "robot1_truth"
)
            """)
        
        f_cfg.close()

    def runSimEnvironment(self):
        """
        Start Gazebo/Player/Stage as necessary.
        """


        fileNamePrefix = os.path.join(self.projectPath, self.projectName)

        if self.simSetup['SimType'].lower() == 'gazebo':
            # Run Gazebo first if necessary
            self.subprocess[PROCESS_GAZEBO] = wx.Process(self, PROCESS_GAZEBO)
            self.appendLog("Starting Gazebo...\n")

            self.subprocess[PROCESS_GAZEBO].Redirect()
            wx.Execute("wxgazebo %s.world" % fileNamePrefix, wx.EXEC_ASYNC, self.subprocess[PROCESS_GAZEBO])
            time.sleep(1) # HACK: Wait for Gazebo to start up
            
        if self.simSetup['SimType'].lower() != 'none':
            # Run Player server unless using real robot
            self.subprocess[PROCESS_PLAYER] = wx.Process(self, PROCESS_PLAYER)
            self.appendLog("Starting Player...\n")
            self.playerPID = wx.Execute("player %s.cfg" % fileNamePrefix, wx.EXEC_ASYNC, self.subprocess[PROCESS_PLAYER])
            time.sleep(1)

    def onMenuSimulate(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """ Run the simulation with current experiment configration. """

        # TODO: Maybe we shouldn't be recalculating fileNamePrefix so much...
        fileNamePrefix = os.path.join(self.projectPath, self.projectName)

        if self.projectName is None or not os.path.isfile(fileNamePrefix+".aut"):
            # TODO: Deal with case where aut file exists but is lame
            wx.MessageBox("Cannot find automaton for simulation.  Please make sure compilation completed successfully.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        redir = RedirectText(self.text_ctrl_log)

        sys.stdout = redir
        sys.stderr = redir

        subprocess.Popen(["python", "execute.py", "-a", fileNamePrefix + ".aut", "-s", fileNamePrefix + ".spec"])


        """
        if self.simSetup['SimType'].lower() != 'none':
            # only need this if running a simulation - Stage/Gazebo
            self.writeSimConfig()
            #TODO: if dirty:
            #wx.MessageBox("Please save before running a simulation.", "Error",
            #              style = wx.OK | wx.ICON_ERROR)
            self.onMenuSave()
            self.runSimEnvironment()

            # Run the simulation 
            self.appendLog("Starting client...\n")
            cmd = subprocess.Popen(["python", "playerClient.py", fileNamePrefix + ".spec", fileNamePrefix + ".aut", "localhost"])
            #os.system("./playerClient.py %s.spec %s.aut" % (os.path.join(wd,self.projectName), os.path.join(wd,self.projectName)))
        else:
            # Running on a real robot - don't need to run player, just the client
            # need to pass the host name
            self.appendLog("Starting client...\n")
            cmd = subprocess.Popen(["python", "orcaClient.py", fileNamePrefix + ".spec", fileNamePrefix + ".aut","scarab-6"])

            while cmd.poll():
                wx.Yield()

            # TODO: Detect quit, make interface
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__
        """


    def onClickEditRegions(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # Provide visual feedback and ake sure multiple copies of the Region Editor
        # can't be open at the same time
        self.button_edit_regions.Enable(False)
        self.button_edit_regions.SetLabel("Starting Region Editor...")

        # Force window update to fit new text length in button
        # FIXME: Doesn't change size on Windows??
        self.Layout()

        # Spawn asynchronous subprocess
        self.subprocess[PROCESS_REGED] = wx.Process(self, PROCESS_REGED)
        if self.projectFiles['RegionFile'] != "":
            # If we already have a region file defined, open it up for editing
            fileName = os.path.join(self.projectPath, self.projectFiles['RegionFile'])
            self.lastRegionModTime = os.path.getmtime(fileName)
            wx.Execute("python regionEditor.py %s" % fileName, wx.EXEC_ASYNC, self.subprocess[PROCESS_REGED])
        else:
            # Otherwise let's create a new region file
            if self.projectName is None:
                # First we need a project name, though
                wx.MessageBox("Please save first to give the project a name.", "Error",
                            style = wx.OK | wx.ICON_ERROR)
                self.onMenuSave()
                if self.projectName is None:
                    # If the save was cancelled, forget it
                    self.button_edit_regions.SetLabel("Edit Regions...")
                    self.Layout()
                    self.button_edit_regions.Enable(True)
                    return
            # We'll name the region file with the same name as our project
            fileName = os.path.join(self.projectPath, self.projectName+".regions")
            wx.Execute("python regionEditor.py %s" % fileName, wx.EXEC_ASYNC, self.subprocess[PROCESS_REGED])

    
    def onProcessTerminate(self, event):
        id = event.GetId()
        if id == PROCESS_REGED:
            ###############################
            # After Region Editor returns #
            ###############################

            # Re-enable the Edit Regions button
            self.button_edit_regions.SetLabel("Edit Regions...")
            self.Layout()
            self.button_edit_regions.Enable(True)

            # If we were editing a new region file
            if self.projectFiles['RegionFile'] == "":
                fileName = os.path.join(self.projectPath, self.projectName+".regions")
                # Check whether the user actually saved or not
                if os.path.isfile(fileName):
                    self.loadRegionFile(fileName)
                    self.projectFiles['RegionFile']=os.path.basename(fileName)
            # Or if it was a pre-existing region file
            else:
                fileName = os.path.join(self.projectPath, self.projectFiles['RegionFile'])
                # Check to see if its modification time has changed; no point in reloading otherwise
                if os.path.getmtime(fileName) != self.lastRegionModTime:
                    self.loadRegionFile(fileName)
        elif id == PROCESS_PLAYER:
            ########################
            # After Player returns #
            ########################
            self.appendLog("Player/Stage closed.\n")
            pass
        elif id == PROCESS_GAZEBO:
            ########################
            # After Gazebo returns #
            ########################
            self.appendLog("Gazebo closed.\n")
            if self.subprocess[PROCESS_PLAYER] is not None:
                # HACK! Player doesn't seem to want to close by itself... :(
                self.subprocess[PROCESS_PLAYER].Kill(self.playerPID)
        elif id == PROCESS_DOTTY:
            ########################
            # After Dotty returns #
            ########################

            # TODO: Check exists & mtime to make sure it didn't die
            self.appendLog("Export complete!\n", "GREEN")
        else:
            print "Unknown PID"
            return

        if self.subprocess[id] != None:
            self.subprocess[id].Destroy()
            self.subprocess[id] = None

    def onMenuConfigSim(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        setupDialog = simSetupDialog(self, self)
        setupDialog.ShowModal()

    def onMenuViewAut(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        fileNamePrefix = os.path.join(self.projectPath, self.projectName) 

        if self.projectName is None or self.rfi is None or not os.path.isfile(fileNamePrefix+".aut"):
            wx.MessageBox("Cannot find automaton for viewing.  Please make sure compilation completed successfully.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return
        
        if self.subprocess[PROCESS_DOTTY] is not None: 
            wx.MessageBox("Dotty is already running.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        self.appendLog("Generating PostScript file from automaton...\n", "BLUE")
        aut = fsa.Automaton(fileNamePrefix+".aut", self.rfi.regions, self.list_box_sensors.GetItems())
        aut.writeDot(fileNamePrefix+".dot")
        self.subprocess[PROCESS_DOTTY] = wx.Process(self, PROCESS_DOTTY)
        wx.Execute("dot -Tps2 -o%s.ps2 %s.dot" % (fileNamePrefix, fileNamePrefix),
                    wx.EXEC_ASYNC, self.subprocess[PROCESS_DOTTY])

    def onMenuQuit(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.doClose(event)
        #event.Skip()

    def onMenuAbout(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        print "Event handler `onMenuAbout' not implemented"
        event.Skip()

# end of class SpecEditorFrame

class RedirectText:
    """
    http://mail.python.org/pipermail/python-list/2007-June/445795.html
    """

    def __init__(self,aWxTextCtrl):
        self.out=aWxTextCtrl

    def write(self,string):
        self.out.BeginTextColour("BLACK")
        self.out.AppendText("\t"+string)
        self.out.EndTextColour()
        self.out.ShowPosition(self.out.GetLastPosition())

if __name__ == "__main__":
    SpecEditor = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    frame_1 = SpecEditorFrame(None, -1, "")
    SpecEditor.SetTopWindow(frame_1)
    frame_1.Show()

    if len(sys.argv) > 1:
        frame_1.openFile(sys.argv[1])

    SpecEditor.MainLoop()
