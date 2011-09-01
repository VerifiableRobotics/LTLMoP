#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" ====================================
    specEditor.py - Specification Editor
    ====================================
    
    A development environment for specifications written in structured English,
    allowing for editing, compilation, and execution/simulation
"""

import re, sys, os, subprocess, time, copy
import wxversion
#wxversion.select('2.8')
import wx, wx.richtext, wx.stc
from lib.regions import *
from lib.createJTLVinput import createLTLfile, createSMVfile
from lib.parseEnglishToLTL import writeSpec
import lib.fileMethods as fileMethods
import lib.project as project
import lib.fsa as fsa
import lib.parseLP as parseLP
import lib.mapRenderer as mapRenderer
from lib.convert import createAnzuFile
from lib.simulator.ode.ckbot import CKBotLib # added by Sarah
#import lib.recolorLTL as recolorLTL

##################### WARNING! ########################
#     DO NOT EDIT GUI CODE BY HAND.  USE WXGLADE.     #
#   The .wxg file is located in the dev/ directory.   #
#######################################################

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
        self.choice_sim_lab = wx.Choice(self, -1, choices=[])
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
        self.Bind(wx.EVT_CHOICE, self.onSimRobot, self.choice_sim_robot)
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
        self.loadSimSetup(self.list_box_experiment_name.GetSelection())
        self.map = {}


    def __set_properties(self):
        # begin wxGlade: simSetupDialog.__set_properties
        self.SetTitle("Configure Simulation")
        self.SetSize((1000, 750))
        self.text_ctrl_sim_experiment_name.SetMinSize((300, 27))
        self.choice_startpos.SetMinSize((300, 29))
        self.list_box_init_customs.SetSelection(0)
        self.list_box_init_actions.SetSelection(0)
        self.list_box_init_sensors.SetSelection(0)
        self.choice_sim_lab.SetMinSize((150, 29))
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
        grid_sizer_1.Add(self.choice_sim_lab, 0, wx.ALIGN_CENTER_VERTICAL, 0)
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
                if not (region.isObstacle or region.name == "boundary"):
                    self.choice_startpos.Append(region.name)
        self.choice_startpos.SetStringSelection(self.parent.rfi.regions[self.tempSimSetup[id]["InitialRegion"]].name)

        # Set up the list of robots
        self.choice_sim_robot.Clear()
        fileList = os.listdir(os.path.join(os.getcwd(),'robots')) + \
                   os.listdir(self.parent.proj.project_root)

        for robotFile in fileList:
            if robotFile.endswith('.robot'):
                self.choice_sim_robot.Append(robotFile.split('.')[0])
                if robotFile == self.tempSimSetup[id]['RobotFile']:
                    self.choice_sim_robot.Select(self.choice_sim_robot.GetItems().index(robotFile.split('.')[0]))

        # Set up the list of labs
        self.choice_sim_lab.Clear()
        fileList = os.listdir(os.path.join(os.getcwd(),'labs'))

        for labFile in fileList:
            if labFile.endswith('.lab'):
                self.choice_sim_lab.Append(labFile.split('.')[0])
                if labFile == self.tempSimSetup[id]['LabFile']:
                    self.choice_sim_lab.Select(self.choice_sim_lab.GetItems().index(labFile.split('.')[0]))

        # Load in the coordinate transformation values
        self.text_ctrl_xscale.SetValue(str(self.tempSimSetup[id]["XScale"]))
        self.text_ctrl_xoffset.SetValue(str(self.tempSimSetup[id]["XOffset"]))
        self.text_ctrl_yscale.SetValue(str(self.tempSimSetup[id]["YScale"]))
        self.text_ctrl_yoffset.SetValue(str(self.tempSimSetup[id]["YOffset"]))

    def saveSimSetup(self, id):
        """Temporarily save the data to the copy of simSetup"""

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
        self.tempSimSetup[id]['InitialRegion'] = self.parent.rfi.indexOfRegionWithName(self.choice_startpos.GetStringSelection())

        # Update robot file name
        self.tempSimSetup[id]['RobotFile'] = self.choice_sim_robot.GetStringSelection()+".robot"

        # Update lab file name
        self.tempSimSetup[id]['LabFile'] = self.choice_sim_lab.GetStringSelection()+".lab"

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

        name = self.text_ctrl_sim_experiment_name.GetValue()


        for id, config in enumerate(self.tempSimSetup):
            if name == config['Name']:
                self.saveSimSetup(id)

        # Make sure not simulating already
        # TODO: Update this part
        #if self.parent.subprocess[PROCESS_PLAYER] is not None \
        #or self.parent.subprocess[PROCESS_GAZEBO] is not None:
        #    wx.MessageBox("Please close the running simulation before calibrating.", "Error",
        #                style = wx.OK | wx.ICON_ERROR)
        #    return

        fileNamePrefix = os.path.join(self.parent.projectPath, self.parent.projectName)

        # TODO: save the config first
        self.parent.onMenuSave()
        proc = subprocess.Popen(["python", os.path.join("lib","calibrate.py"), fileNamePrefix + ".spec", self.list_box_experiment_name.GetStringSelection()],stderr=subprocess.PIPE)
        
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



    def onSimRobot(self, event): # wxGlade: simSetupDialog.<event_handler>
        """
            Warn user when the selected robot does not have all sensor and actions needed.
        """
        rdf_name = self.choice_sim_robot.GetStringSelection()
        #if not rdf_name.endswith('.robot'):
        #    rdf_name = rdf_name+'.robot' 
        #rdf_data = fileMethods.readFromFile(os.path.join(os.path.join(os.getcwd(),'robots'), rdf_name))
        # TODO: Use project.py everywhere, less hackily
        temp_proj = copy.deepcopy(self.parent.proj)
        rdf_data = temp_proj.loadRobotFile({'RobotFile':[rdf_name]})

        warning = False
        
        # Check sensors 
        for sensor in self.list_box_init_sensors.GetItems():
            if sensor not in rdf_data['Sensors']:
                selection = wx.MessageBox('Sorry.  The selected robot does not have all required sensors.\n Do you want to continue?', 'Sensor Unavailable!', wx.YES_NO | wx.NO_DEFAULT | wx.ICON_EXCLAMATION)
                if selection == wx.NO:
                    self.choice_sim_robot.Select(self.choice_sim_robot.GetItems().index(self.tempSimSetup[self.list_box_experiment_name.GetSelection()]['RobotFile'].split('.')[0]))
                    warning = True
                    break
                else:
                    break

        if not warning:
            # Check actions 
            for action in self.list_box_init_actions.GetItems():
                if action not in rdf_data['Actions']:
                    selection = wx.MessageBox('Sorry. The selected robot does not have all required actions.\n Do you want to continue?', 'Action Unavailable!', wx.YES_NO | wx.NO_DEFAULT | wx.ICON_EXCLAMATION)
                    if selection == wx.NO:
                        self.choice_sim_robot.Select(self.choice_sim_robot.GetItems().index(self.tempSimSetup[self.list_box_experiment_name.GetSelection()]['RobotFile'].split('.')[0]))
                        break
                    else:
                        break

                    
        event.Skip()

# end of class simSetupDialog


class MapDialog(wx.Dialog):
    """
    A silly little dialog that displays the regions on top of the map so that you can
    select a region visually instead of just choosing the name.
    """
    
    # FIXME: Doesn't scroll on Windows???

    def __init__(self, parent, *args, **kwds):
        self.parent = parent
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
        sizer_10.Add(self.panel_2, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_10)
        self.Layout()
        self.Centre()
        # end wxGlade

        self.panel_2.SetBackgroundColour(wx.WHITE)   
        self.panel_2.Bind(wx.EVT_PAINT, self.drawMap)

        # Bind to catch mouse clicks!
        self.panel_2.Bind(wx.EVT_LEFT_DOWN, self.onMapClick)

    def drawMap(self, event):
        mapRenderer.drawMap(self.panel_2, self.parent.proj, scaleToFit=False)

    def onMapClick(self, event):
        x, y = self.panel_2.CalcUnscrolledPosition(event.GetX(), event.GetY())
        for region in self.parent.rfi.regions:
            if region.objectContainsPoint(x, y):
                self.parent.text_ctrl_spec.AppendText(region.name)
                #self.EndModal(1)
                self.Close()
                break 
        event.Skip()

# end of class MapDialog


class SpecEditorFrame(wx.Frame):
    """
    The main application window!
    """

    # TODO: Use the scintilla-style text editor. Select-all, search-replace, syntax highlighting, etc.

    def __init__(self, *args, **kwds):
        # begin wxGlade: SpecEditorFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.window_1 = wx.SplitterWindow(self, -1, style=wx.SP_3D|wx.SP_BORDER|wx.SP_LIVE_UPDATE)
        self.window_1_pane_2 = wx.Panel(self.window_1, -1)
        self.notebook_1 = wx.Notebook(self.window_1_pane_2, -1, style=0)
        self.notebook_1_pane_3 = wx.Panel(self.notebook_1, -1)
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
        global MENU_ANALYZE; MENU_ANALYZE = wx.NewId()
        global MENU_MOPSY; MENU_MOPSY = wx.NewId()
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
        wxglade_tmp_menu.Append(MENU_ANALYZE, "&Analyze\tF8", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_MOPSY, "&Visualize Counterstrategy...\tF10", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Debug")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(wx.ID_ABOUT, "&About Specification Editor...", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Help")
        self.SetMenuBar(self.frame_1_menubar)
        # Menu Bar end
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
        self.label_locphrases = wx.StaticText(self.notebook_1_pane_3, -1, "Active locative phrases:")
        self.list_box_locphrases = wx.ListBox(self.notebook_1_pane_3, -1, choices=[], style=wx.LB_ALWAYS_SB)
        self.checkbox_regionlabel = wx.CheckBox(self.notebook_1_pane_3, -1, "Show region names")
        self.panel_locmap = wx.Panel(self.notebook_1_pane_3, -1, style=wx.SUNKEN_BORDER|wx.TAB_TRAVERSAL|wx.FULL_REPAINT_ON_RESIZE)

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
        self.Bind(wx.EVT_MENU, self.onMenuAnalyze, id=MENU_ANALYZE)
        self.Bind(wx.EVT_MENU, self.onMenuMopsy, id=MENU_MOPSY)
        self.Bind(wx.EVT_MENU, self.onMenuAbout, id=wx.ID_ABOUT)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_regions)
        self.Bind(wx.EVT_BUTTON, self.onMapSelect, self.button_map)
        self.Bind(wx.EVT_BUTTON, self.onClickEditRegions, self.button_edit_regions)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_sensors)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_actions)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_customs)
        self.Bind(wx.EVT_BUTTON, self.onCustomNew, self.button_custom_new)
        self.Bind(wx.EVT_BUTTON, self.onCustomDelete, self.button_custom_delete)
        self.Bind(wx.EVT_LISTBOX, self.onLocPhraseSelect, self.list_box_locphrases)
        self.Bind(wx.EVT_CHECKBOX, self.onRegionLabelToggle, self.checkbox_regionlabel)
        # end wxGlade

		# Create the StyledTextControl for the specification area manually, since wxGlade doesn't support it
        self.text_ctrl_spec = wx.stc.StyledTextCtrl(self.window_1_pane_1, -1, style=wx.TE_PROCESS_ENTER|wx.TE_PROCESS_TAB|wx.TE_MULTILINE|wx.WANTS_CHARS)
        self.text_ctrl_spec.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL, 0, ""))
        self.window_1_pane_1.GetSizer().Insert(0, self.text_ctrl_spec, 2, wx.EXPAND, 0)
        self.window_1_pane_1.GetSizer().Layout()

        self.text_ctrl_spec.SetMarginWidth(0, 40)
        self.text_ctrl_spec.SetMarginType(0, wx.stc.STC_MARGIN_NUMBER)
        self.text_ctrl_spec.SetMarginType(1, wx.stc.STC_MARGIN_SYMBOL)

        global MARKER_INIT, MARKER_SAFE, MARKER_LIVE
        MARKER_INIT, MARKER_SAFE, MARKER_LIVE = range(3)
        self.text_ctrl_spec.MarkerDefine(MARKER_INIT,wx.stc.STC_MARK_ARROW,"white","red") 
        self.text_ctrl_spec.MarkerDefine(MARKER_SAFE,wx.stc.STC_MARK_ARROW,"white","blue") 
        self.text_ctrl_spec.MarkerDefine(MARKER_LIVE,wx.stc.STC_MARK_ARROW,"white","green") 
        
        # Set up locative phrase map
        self.panel_locmap.SetBackgroundColour(wx.WHITE)   
        self.panel_locmap.Bind(wx.EVT_PAINT, self.drawLocMap)

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
        global PROCESS_DOTTY; PROCESS_DOTTY = 1

        # Set default values
        self.setDefaults()
        self.proj = project.Project()
        self.proj.regionMapping = {}
        self.parser = None
		
        # HACK: This is an undocumented hack you can uncomment to help kill stuck copies of speceditor on windows
        # If in use, requires spec file argument on command line
        #if sys.argv[-1] != "-dontbreak":
        #    os.system("taskkill /im python.exe /f && " + " ".join(sys.argv) + " -dontbreak")

        # Initialize library -- Sarah
        self.library = CKBotLib.CKBotLib()
       
    def __set_properties(self):
        # begin wxGlade: SpecEditorFrame.__set_properties
        self.SetTitle("Specification Editor - Untitled")
        self.SetSize((900, 700))
        self.button_map.Enable(False)
        self.list_box_sensors.SetMinSize((123, 75))
        self.list_box_actions.SetMinSize((123, 75))
        self.list_box_customs.SetMinSize((123, 75))
        self.button_custom_delete.Enable(False)
        self.panel_1.SetScrollRate(10, 10)
        self.checkbox_regionlabel.SetValue(1)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: SpecEditorFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_14 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_15 = wx.BoxSizer(wx.VERTICAL)
        sizer_9 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_3 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5 = wx.BoxSizer(wx.VERTICAL)
        sizer_8 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_7 = wx.BoxSizer(wx.HORIZONTAL)
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
        sizer_14.Add((5, 20), 0, 0, 0)
        sizer_15.Add((20, 20), 0, 0, 0)
        sizer_15.Add(self.label_locphrases, 0, 0, 0)
        sizer_15.Add(self.list_box_locphrases, 1, wx.EXPAND, 0)
        sizer_15.Add((20, 20), 0, 0, 0)
        sizer_15.Add(self.checkbox_regionlabel, 0, wx.EXPAND, 0)
        sizer_15.Add((20, 20), 0, 0, 0)
        sizer_14.Add(sizer_15, 1, wx.EXPAND, 0)
        sizer_14.Add((5, 20), 0, 0, 0)
        sizer_14.Add(self.panel_locmap, 2, wx.EXPAND, 0)
        self.notebook_1_pane_3.SetSizer(sizer_14)
        self.notebook_1.AddPage(self.notebook_1_pane_1, "Compiler Log")
        self.notebook_1.AddPage(self.notebook_1_pane_2, "LTL Output")
        self.notebook_1.AddPage(self.notebook_1_pane_3, "Workspace Decomposition")
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

    def drawLocMap(self, event):
        """ Respond to a request to redraw the contents of our drawing panel.
        """

        # Nothing to draw if there are no regions loaded yet
        if self.parser is None or self.proj.regionMapping is None:
            return

        highlightList = self.parser.proj.regionMapping[self.list_box_locphrases.GetStringSelection()]

        mapRenderer.drawMap(self.panel_locmap, self.parser.proj, scaleToFit=True, drawLabels=self.checkbox_regionlabel.GetValue(), highlightList=highlightList)

    def onPropositionDblClick(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Add the proposition name to the spec when you double click the name
        """

        caller = event.GetEventObject()
        if caller in [self.list_box_sensors, self.list_box_actions] \
           and not caller.IsChecked(caller.GetSelection()):
            # Only allow adding of enabled propositions
            return
        self.text_ctrl_spec.AppendText(caller.GetStringSelection())

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

        # TODO: The Project module needs to be properly integrated here...

        # Create a RFI if necessary
        if self.rfi is None:
            self.rfi = RegionFileInterface()

        # Try loading the file
        if not self.rfi.readFile(fileName):
            wx.MessageBox("Cannot open region file %s" % (fileName), "Error",
                        style = wx.OK | wx.ICON_ERROR)
            self.rfi = None
            return

        # Update the path information
        # NOTE: At least for now, all project files must be in the same directory.
        self.projectFiles["RegionFile"] = os.path.basename(fileName)

        # Add the regions to the region listbox
        self.list_box_regions.Set([])
        for region in self.rfi.regions:
            if not (region.isObstacle or region.name == "boundary"):
                self.list_box_regions.Insert(region.name, 0)        
        
        # If we are working with an unsaved spec, assume everything is in the same dir
        # as the regions file
        if self.projectPath == "":
            self.projectPath = os.path.dirname(os.path.abspath(fileName))

        # Create the map selection dialog
        if self.mapDialog is not None:
            self.mapDialog.Destroy()
        self.mapDialog = MapDialog(frame_1, frame_1)
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

        self.openFile(fileName)  # FIXME: This is really ugly
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
        fileName = wx.FileSelector("Save File As", "examples",
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
        
        regionMappingData = ''
        if len(self.proj.regionMapping)>0:
            mappingData = ''
            for nameOfRegion,regionList in self.proj.regionMapping.iteritems():
                mappingData = '='.join([nameOfRegion,','.join(regionList)])
                regionMappingData = '\n'.join([regionMappingData,mappingData])
        else:
            regionMappingData='Null=Null'
        
        view = self.text_ctrl_spec.GetText()
        specOnly = ""
        for line in view:
            if (line == "RESULT") : break
            else : specOnly += line
            
            
        data['SPECIFICATION'] = {'Spec': str(specOnly),'RegionMapping': regionMappingData}
        data['SETTINGS'] = {"RegionFile": self.projectFiles["RegionFile"],
                            "Sensors": self.dumpListBox(self.list_box_sensors),
                            "Actions": self.dumpListBox(self.list_box_actions),
                            "Customs": self.dumpListBox(self.list_box_customs),
                            "currentExperimentName": self.currentExperimentName,
                            }
        if len(self.simSetup)>0:
            for id, config in enumerate(self.simSetup):
                #if config['Name'] != 'Default': # TODO: Ask Jim why he put this here
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

        # If this is the first time saving, reload the Project object
        if self.proj.project_basename is None:
            self.proj.loadProject(fileName)
            
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
        self.text_ctrl_spec.SetText("")

        # Null the subprocess values
        self.subprocess[PROCESS_REGED] = None
        self.subprocess[PROCESS_DOTTY] = None

    def openFile(self, fileName):#, recolor, sysColor, envColor):          

        if fileName is None:
            return

        self.proj.loadProject(fileName)
 
        data = self.proj.spec_data

        self.setDefaults()

        filePath = os.path.dirname(os.path.abspath(fileName))

        for name, content in data.iteritems():
            if name == 'SPECIFICATION' and 'Spec' in content and len(content['Spec']) > 0:
                for i, line in enumerate(content['Spec']): 
                            line = line + "\n"
                            self.text_ctrl_spec.AppendText(line)

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
                else:
                    self.simSetup[-1]["Name"]=""
                if 'RobotFile' in content and len(content['RobotFile']) > 0:
                    self.simSetup[-1]["RobotFile"] = content['RobotFile'][0]
                else:
                    self.simSetup[-1]["RobotFile"]=""
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
                else:
                    self.simSetup[-1]["LabFile"]=""

        # Remove default if we have other configs
        if len(self.simSetup) > 1:
            del self.simSetup[0]

        # Update the window title
        title = os.path.basename(fileName)
        self.SetTitle("Specification Editor - " + title)

        # Update paths
        self.fileName = fileName
        self.projectName = os.path.splitext(title)[0]
        self.projectPath = filePath

        self.text_ctrl_spec.EmptyUndoBuffer()
    
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
        self.text_ctrl_spec.Cut()
        event.Skip()

    def onMenuCopy(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.text_ctrl_spec.Copy()
        event.Skip()

    def onMenuPaste(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanPaste():
            self.text_ctrl_spec.Paste()
        #event.Skip()

    def onMenuCompile(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # Let's make sure we have everything!
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_INIT)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_SAFE)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_LIVE)
        if self.rfi is None:
            wx.MessageBox("Please define regions before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return
    
        if self.text_ctrl_spec.GetText() == "":
            wx.MessageBox("Please write a specification before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        if self.proj.project_basename is None:
            wx.MessageBox("Please save your project before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        # Clear the log so we can start fresh grocer
        self.text_ctrl_log.Clear()

        # Redirect all output to the log
        redir = RedirectText(self.text_ctrl_log)

        sys.stdout = redir
        sys.stderr = redir
        
        ##########################
        # Create new region File #
        ##########################
        
        self.appendLog("Parsing locative prepositions...\n", "BLUE")
        wx.Yield()
        self.saveFile(self.fileName)
        self.parser = parseLP.parseLP()
        self.parser.main(self.fileName)

        # Remove all references to any obstacle regions at this point
        for r in self.proj.rfi.regions:
            if r.isObstacle:
                # Delete corresponding decomposed regions
                for sub_r in self.parser.proj.regionMapping[r.name]:
                    del self.parser.proj.rfi.regions[self.parser.proj.rfi.indexOfRegionWithName(sub_r)]
                # Remove from mapping
                del self.parser.proj.regionMapping[r.name]

        self.proj.rfi.regions = filter(lambda r: not (r.isObstacle or r.name == "boundary"), self.proj.rfi.regions)
                    
        # save the regions into new region file
        fileName = self.proj.getFilenamePrefix()+'_decomposed.regions'
        self.parser.proj.rfi.recalcAdjacency()
        self.parser.proj.rfi.writeFile(fileName)

        self.proj.regionMapping = self.parser.proj.regionMapping
        self.saveFile(self.fileName)
        
        # Update workspace decomposition listbox
        self.list_box_locphrases.Set(self.proj.regionMapping.keys())
        self.list_box_locphrases.Select(0)
        
        # update the rfi for new regions
        rfi = self.parser.proj.rfi
        
        # substitute the regions name in specs
        text = self.text_ctrl_spec.GetText()
        for m in re.finditer(r'near (?P<rA>\w+)', text):
            text=re.sub(r'near (?P<rA>\w+)', "("+' or '.join(self.parser.proj.regionMapping['near$'+m.group('rA')+'$'+str(50)])+")", text)
        for m in re.finditer(r'within (?P<dist>\d+) (from|of) (?P<rA>\w+)', text):
            text=re.sub(r'within ' + m.group('dist')+' (from|of) '+ m.group('rA'), "("+' or '.join(self.parser.proj.regionMapping['near$'+m.group('rA')+'$'+m.group('dist')])+")", text)
        for m in re.finditer(r'between (?P<rA>\w+) and (?P<rB>\w+)', text):
            text=re.sub(r'between ' + m.group('rA')+' and '+ m.group('rB'),"("+' or '.join(self.parser.proj.regionMapping['between$'+m.group('rA')+'$and$'+m.group('rB')+"$"])+")", text)
        for r in self.proj.rfi.regions:
            text=re.sub('\\b' + r.name + '\\b', "("+' or '.join(self.parser.proj.regionMapping[r.name])+")", text)

        print "===== New Specs ====="
        print
        print text
        print "====================="
        ###################
        # Create SMV File #
        ###################

        self.appendLog("Creating SMV file...\n", "BLUE")
        wx.Yield()

        numRegions = len(rfi.regions)

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

        ###################
        # Create LTL File #
        ###################

        self.appendLog("Creating LTL file...\n", "BLUE")
        wx.Yield()
        regionList = [x.name for x in rfi.regions]

        spec,self.map = writeSpec(text, sensorList, regionList, robotPropList)
        # TODO: Catch errors here
        adjData = rfi.transitions

        createLTLfile(fileNamePrefix, sensorList, robotPropList, adjData, spec)
        createAnzuFile(fileNamePrefix, sensorList, robotPropList, adjData, spec)
               
        #self.appendLog(self.proj.ltlmop_root)
        #arg="python \lib\convert_anzu_input_to_marduk.py -r -i "+fileNamePrefix + ".anzu "+ "-o "+ fileNamePrefix + ".rat"
        #os.system(arg)

        classpath = "\"" + os.path.join(self.proj.ltlmop_root, "lib", "convert_anzu_input_to_marduk.py\"")
        fNP = "\""+ fileNamePrefix + "\""
        
        arg="python "+classpath+" -r -i " + fNP+ ".anzu "+ "-o " +  fNP + ".rat"
        
        os.system(arg)
        
        #wx.Execute(arg, wx.EXEC_ASYNC, self.subprocess[PROCESS_REGED])
        

        if os.path.exists(fileNamePrefix+".ltl"):
            f = open(fileNamePrefix+".ltl","r")
            ltl = "".join(f.readlines())
            f.close()
            self.text_ctrl_LTL.SetValue(ltl)

			
		################################################
		# Add a check for Empty Gaits - added by Sarah #
		################################################
        self.appendLog("\nChecking for empty gaits...\n")
        #print robotPropList
        err = 0
        libs = self.library
        libs.readLibe()
		# Check that each individual trait has a corresponding config-gait pair
        for act in robotPropList:
            act = act.strip("u's.")
            if act[0] == "T":
                act = act.strip("T_")
                #print act
                words = act.split("_and_")
                #print words
                config = libs.findGait(words)
                #print config
                if type(config) == type(None):
                    err_message = "WARNING: No config-gait pair for actuator T_" + act + "\n"
                    self.appendLog(err_message)
                    err = 1
			
        ####################
        # Create automaton #
        ####################

        self.appendLog("Creating automaton...\n", "BLUE")
        wx.Yield()

        # Windows uses a different delimiter for the java classpath
        if os.name == "nt":
            classpath = os.path.join(self.proj.ltlmop_root, "etc\jtlv", "jtlv-prompt1.4.0.jar") + ";" + os.path.join(self.proj.ltlmop_root, "etc\jtlv", "GROne")
        else:
            classpath = os.path.join(self.proj.ltlmop_root, "etc/jtlv", "jtlv-prompt1.4.0.jar") + ":" + os.path.join(self.proj.ltlmop_root, "etc/jtlv", "GROne")

        cmd = subprocess.Popen(["java", "-ea", "-Xmx512m", "-cp", classpath, "GROneMain", fileNamePrefix + ".smv", fileNamePrefix + ".ltl", "--safety"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False)
        
        # TODO: Make this output live
        while cmd.poll():
            wx.Yield()

        realizable = False

        for line in cmd.stdout:
            self.appendLog("\t"+line)
            if "Specification is realizable" in line:
               realizable = True
               
        cmd.stdout.close()
        print "\n"

        if realizable:
            self.appendLog("Automaton successfully synthesized.\n", "GREEN")
        else:
            self.appendLog("ERROR: Specification was unsynthesizable (unrealizable/unsatisfiable).\n", "RED")

        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__


    def appendLog(self, text, color="BLACK"):
        self.text_ctrl_log.BeginTextColour(color)
        #self.text_ctrl_log.BeginBold()
        self.text_ctrl_log.WriteText(text)
        #self.text_ctrl_log.EndBold()
        self.text_ctrl_log.EndTextColour()
        self.text_ctrl_log.ShowPosition(self.text_ctrl_log.GetLastPosition())



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

        subprocess.Popen(["python", os.path.join("lib","execute.py"), "-a", fileNamePrefix + ".aut", "-s", fileNamePrefix + ".spec"])

        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__


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

        self.appendLog("Generating PDF file from automaton...\n", "BLUE")

        aut = fsa.Automaton(self.parser.proj.rfi.regions, self.parser.proj.regionMapping, None, None, None) 

        aut.loadFile(fileNamePrefix+".aut", self.list_box_sensors.GetItems(), self.list_box_actions.GetItems(), self.list_box_customs.GetItems())
        aut.writeDot(fileNamePrefix+".dot")
        self.subprocess[PROCESS_DOTTY] = wx.Process(self, PROCESS_DOTTY)
        #wx.Execute("dot -Tps2 -o%s.ps2 %s.dot" % ("\""+fileNamePrefix+"\"", "\""+fileNamePrefix+"\""),
                    #wx.EXEC_ASYNC, self.subprocess[PROCESS_DOTTY])
        wx.Execute("dot -Tpdf -o%s.pdf\" %s.dot\"" % ("\""+fileNamePrefix, "\""+fileNamePrefix),
                    wx.EXEC_ASYNC, self.subprocess[PROCESS_DOTTY])
        #self.subprocess[PROCESS_DOTTY].Destroy()
        self.subprocess[PROCESS_DOTTY] = None

    def onMenuQuit(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.doClose(event)
        #event.Skip()

    def onMenuAbout(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        print "Event handler `onMenuAbout' not implemented"
        event.Skip()

    def onRegionLabelToggle(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.panel_locmap.Refresh()
        event.Skip()

    def onLocPhraseSelect(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.panel_locmap.Refresh()
        event.Skip()

    def onMenuAnalyze(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.onMenuCompile(event)
        self.onMenuViewAut(event)
        
        fileNamePrefix = os.path.join(self.projectPath, self.projectName)

        output = "\nRESULT\n"
        f = open(fileNamePrefix+".debug","r")
        realizable = False    
        for dline in f.readlines():
          if "REALIZABLE" in dline:   
            realizable = True            
            self.appendLog("Automaton successfully synthesized.\n", "GREEN")
            aut = fsa.Automaton(self.parser.proj.rfi.regions, self.parser.proj.regionMapping, None, None, None) 
            aut.loadFile(fileNamePrefix+".aut", self.list_box_sensors.GetItems(), self.list_box_actions.GetItems(), self.list_box_customs.GetItems())
            aut.writeDot(fileNamePrefix+".dot")
            f = open(fileNamePrefix+".dot","r")
            nonTrivial = False
            for autline in f.readlines():
                if "->" in autline:
                    nonTrivial = True 
            if nonTrivial:
                self.appendLog("Synthesized automaton is non-trivial.\n", "GREEN")
                output += "Synthesized automaton is non-trivial.\n"
            else:
                self.appendLog("Synthesized automaton is trivial.\n", "GREEN")
                output += "Synthesized automaton is trivial.\n"
          if "SysInit UNSAT" in dline:
           output = output + "System initial condition is unsatisfiable.\n" 
           for l in self.self.map['SysInit']: self.text_ctrl_spec.MarkerAdd(l,MARKER_INIT)
          if "SysTrans UNSAT" in dline:
            output = output + "System transition relation is unsatisfiable.\n" 
            for l in self.map['SysTrans']: self.text_ctrl_spec.MarkerAdd(l, MARKER_SAFE)
          if "SysGoals UNSAT" in dline:
               output = output + "System highlighted goal(s) unsatisfiable \n"
               for l in (dline.strip()).split()[2:]:
                    self.text_ctrl_spec.MarkerAdd(self.map['SysGoals'][int(l)],MARKER_LIVE)           
          if "SysGoalsTrans UNSAT" in dline:
               output = output + "System highlighted goal(s) inconsistent with transition relation. \n"
               for l in (dline.strip()).split()[2:]:
                    self.text_ctrl_spec.MarkerAdd(self.map['SysGoals'][int(l)],MARKER_LIVE)           
               for l in self.map['SysTrans']: self.text_ctrl_spec.MarkerAdd(l,MARKER_SAFE)
          if "SysInitTrans UNSAT" in dline:
               output = output + "System initial condition inconsistent with transition relation. \n"
               for l in self.map['SysInit']: self.text_ctrl_spec.MarkerAdd(l,MARKER_INIT)
               for l in self.map['SysTrans']: self.text_ctrl_spec.MarkerAdd(l,MARKER_SAFE)
           
        
           
        
          if "EnvInit UNSAT" in dline:
            output = output + "Environment initial condition is unsatisfiable.\n" 
            for l in self.map['EnvInit']: self.text_ctrl_spec.MarkerAdd(l,MARKER_INIT)
          if "EnvTrans UNSAT" in dline:
            output = output + "Environment transition relation is unsatisfiable.\n" 
            for l in self.map['EnvTrans']: self.text_ctrl_spec.MarkerAdd(l,MARKER_SAFE)
          if "EnvGoals UNSAT" in dline:
               output = output + "Environment highlighted goal(s) unsatisfiable \n"
               for l in (dline.strip()).split()[2:]:
                    self.text_ctrl_spec.MarkerAdd(self.map['EnvGoals'][int(l)],MARKER_LIVE)
          if "EnvGoalsTrans UNSAT" in dline:
               output = output + "Environment highlighted goal(s) inconsistent with transition relation. \n"
               for l in (dline.strip()).split()[2:]:
                    self.text_ctrl_spec.MarkerAdd(self.map['EnvGoals'][int(l)],MARKER_LIVE)           
               for l in self.map['EnvTrans']: self.text_ctrl_spec.MarkerAdd(l,MARKER_SAFE)
          if "EnvInitTrans UNSAT" in dline:
               output = output + "Environment initial condition inconsistent with transition relation. \n"
               for l in self.map['EnvInit']: self.text_ctrl_spec.MarkerAdd(l,MARKER_INIT)
               for l in self.map['EnvTrans']: self.text_ctrl_spec.MarkerAdd(l,MARKER_SAFE)
           
        
            
          if "SysTrans UNREAL" in dline:
                output = output + "System is unrealizable because the environment can force a safety violation.\n" 
                #for l in self.map['SysTrans']: self.text_ctrl_spec.MarkerSetBackground(l,"RED")
                for l in self.map['SysTrans']: self.text_ctrl_spec.MarkerAdd(l, MARKER_SAFE)
          if "SysGoals UNREAL" in dline:
               output = output + "System highlighted goal(s) unrealizable \n"
               for l in (dline.strip()).split()[2:]:
                    self.text_ctrl_spec.MarkerAdd(self.map['SysGoals'][int(l)],MARKER_LIVE)           
               #for l in self.map['SysTrans']: self.text_ctrl_spec.MarkerSetBackground(l,"RED")
               for l in self.map['SysTrans']: self.text_ctrl_spec.MarkerAdd(l, MARKER_SAFE)
            
        
          if "EnvTrans UNREAL" in dline:
                output = output + "Environment is unrealizable because the environment can force a safety violation.\n" 
                for l in self.map['EnvTrans']: self.text_ctrl_spec.MarkerAdd(l, MARKER_SAFE)
          if "EnvGoals UNREAL" in dline:
               output = output + "Environment highlighted goal(s) unrealizable \n"
               for l in (dline.strip()).split()[2:]:
                    self.text_ctrl_spec.MarkerAdd(self.map['EnvGoals'][int(l)],MARKER_LIVE)           
               for l in self.map['EnvTrans']: self.text_ctrl_spec.MarkerAdd(l, MARKER_SAFE)#self.text_ctrl_spec.MarkerSetBackground(l,"RED")
            
        if not realizable:
            #self.appendLog("ERROR: Specification was unrealizable.\n", "RED")
            output += "No automaton synthesized.\n"        
          
        self.appendLog(output)
        
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__

    def onMenuMopsy(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # TODO: check for failed compilation before allowing this
        subprocess.Popen(["python", os.path.join(self.proj.ltlmop_root,"etc","utils","mopsy.py"), self.fileName])

# end of class SpecEditorFrame

class RedirectText:
    """
    A class that lets the output of a stream be directed into a text box.

    http://mail.python.org/pipermail/python-list/2007-June/445795.html
    """

    def __init__(self,aWxTextCtrl):
        self.out=aWxTextCtrl

    def write(self,string):
        self.out.BeginTextColour("BLACK")
        self.out.WriteText("\t"+string)
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
