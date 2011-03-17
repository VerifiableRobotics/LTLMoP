#!/usr/bin/env python

import pygame
import math, time, copy, sys
sys.path.append("../../../../../Downloads/CKBot/trunk")
from ckbot.logical import Cluster

info = """CKBotSim

CKBot Runtime for LTLMoP
[Sebastian Castro - Cornell University]
[Autonomous Systems Laboratory - 2010]

"""

class CKBotRun:
	"""
	CKBot Hardware Runtime Class
	"""

	def __init__(self,robotfile):
		"""
		Initialize the simulator.
		"""

		# Load robot data		
		self.loadRobotData(robotfile)
		self.gait = 0

		# Initialize the cluster
		tempc = Cluster()
		tempc.populate()
		print tempc

		# HARDCODED CONFIG DEFINITIONS:
		# Snake configuration.
		if self.config == "Snake":
			self.cluster = [tempc[199], tempc[143], tempc[216], tempc[197] ]

		# Initialize the time
		self.starttime = time.time()
		

	def loadRobotData(self,filename):
		"""
		Loads full robot information from a text file that specifies:
		1. Configuration Matrix.
		2. Relative rotation and translation from the origin, in CKBot length units.
		3. A set of gaits and target gait execution times.
		"""

		# Clear all previous information
		self.config = "none"
		self.connM = []
		self.gaits = []

		# Open the text file.
		data = open(filename,"r")

		# Go through the text file line by line.
		reading = "none"
		for line in data:
			linesplit = line.split()

			# If we are currently reading nothing, then we are looking for the next attribute to read.
			if reading == "none" and linesplit != []:
				if linesplit[0] == "ConfigName:":
					reading = "name"
				elif linesplit[0] == "ConnMatrix:":
					reading = "matrix"
				elif linesplit[0] == "ForwardVector:":
					reading = "fwd_vector"
				elif linesplit[0] == "Gaits:":
					reading = "gait"

			# If we are currently reading something, the continue to do so until finished.

			# If we are reading the name of the configuration, then it is simply the word that makes the line under "ConfigName:"
			elif reading == "name":
				self.config = linesplit[0]
				reading = "none"

			# If we are reading the connectivity matrix, then read the matrix row by row until we reach whitespace.
			elif reading == "matrix":
				if linesplit == []:
					reading = "none"
				else:
					temprow = []
					for num in linesplit:
						temprow.append(int(num))
					self.connM.append(temprow)


			# Forward vector reading thingy
			elif reading == "fwd_vector":
				if linesplit == []:
					reading = "none"
				else:
					sign = linesplit[0]
					axis = linesplit[1]
					if sign == "+":
						coeff = 1
					else:
						coeff = -1
					
					if axis == "x":
						self.fwdvec = [coeff*1, 0, 0]
					elif axis == "y":
						self.fwdvec = [0, coeff*1, 0]
					elif axis == "z":
						self.fwdvec = [0, 0, coeff*1]

			# If we are reading gaits, it is more complicated. We must ensure to read all the gaits specified.
			# For each gait, we must be able to tell whether the gait is Periodic or Fixed type and 
			# parse it accordingly.

			elif reading == "gait":

				# Read the Proportional control gain specified in the text file.
				if linesplit[0] == "Gain":
					self.gain = float(linesplit[1])

				# Figure out whether the gaits are fixed or periodic
				if linesplit[0] == "Type":
					self.gaittype = linesplit[1]
					if self.gaittype == "Periodic":
						reading = "periodic_gait"
					else:
						reading = "fixed_gait"				  

			# If we are reading periodic gaits, we know our gait table is just 3 lines.
			# The first line is the set of amplitudes (in degrees*100) of each hinge.
			# The second line is the set of frequencies (in rad/s) of each hinge.
			# The third line is the set of phase angles (in degrees*100) of each hinge.
			elif reading == "periodic_gait" and linesplit != []:
				if linesplit[0] == "Gait":
					amplitudes = []
					frequencies = []
					phases = []
					reading = "amplitude"

			elif reading == "amplitude":
				for elem in linesplit:
					amplitudes.append( float(elem)*(math.pi/180.0)*(1/100.0) )
				reading = "frequency"

			elif reading == "frequency":
				for elem in linesplit:
					frequencies.append( float(elem) )
				reading = "phase"

			elif reading == "phase":
				for elem in linesplit:
					phases.append( float(elem)*(math.pi/180.0)*(1/100.0) )
				tempgait = [amplitudes]
				tempgait.append(frequencies)
				tempgait.append(phases)
				self.gaits.append(tempgait)
				reading = "periodic_gait"

			# If we are reading fixed gaits, the gait table can be an arbitrary number of lines.
			# For each gait we will read all the steps until we find the last line for the gait 
			# (which the time that the gait should loop in).
			elif reading == "fixed_gait" and linesplit != []:
				if linesplit[0] == "Gait":
					gaitrows = []
					gaittime = 0
					reading = "fixed_gait_rows"

			elif reading == "fixed_gait_rows":
				if len(linesplit)==1:
					gaittime = [float(linesplit[0])]
					gaittime.extend(gaitrows)
					self.gaits.append(gaittime)
					reading = "fixed_gait"
				else:
					temprow = []
					for elem in linesplit:
						temprow.append( float(elem)*(math.pi/180.0)*(1/100.0) )
					gaitrows.append(temprow)		    




	def setGait(self,gait):
		"""
		Set the gait number for simulation
		"""
		self.gait = gait


	def rungait(self):
		"""
		Runs the gait specified by the object variable "self.gait"
		"""

		t = time.time() - self.starttime
		gait = self.gaits[self.gait - 1]

		# If the gait is set to zero, stop moving all hinges.
		if self.gait == 0:
			pass

		else:
	
			#If the gait is of periodic type, read the rows in the following format.
			#ROW 1: Amplitudes
			#ROW 2: Frequencies
			#ROW 3: Phases
			if self.gaittype == "Periodic":
				for module_idx in range(len(self.connM)):
					amplitude = gait[0][module_idx]
					frequency = gait[1][module_idx]
					phase = gait[2][module_idx]

					ref_ang = amplitude*math.sin(frequency*t*0.25 + phase)

					ref_ang = ref_ang*(18000.0/math.pi)

					# Fix for uncalibrated head module

					if module_idx==3:
						ref_ang = ref_ang - 1400
		
					self.cluster[module_idx].set_pos(ref_ang)

			# If the gait is of fixed type, run the function "gaitangle" to interpolate between
			# reference hinge angles at the current time.
			else:
				for module_idx in range(len(self.connM)):   
					ref_ang = self.gaitangle(gait,t,module_idx)
	
					ref_ang = ref_ang*(18000.0/math.pi)

					# Fix for uncalibrated head module
					if self.config=="Snake" and module_idx=="3":
						ref_ang = ref_ang - 1400
		
					self.cluster[module_idx].set_pos(ref_ang)

	def gaitangle(self,gait,time,module):
		"""
		Takes in a gait matrix and returns the reference angle at that point in time.
		"""

		nummoves = len(gait)-1
		gaittime = gait[0]
		singletime = float(gaittime)/(nummoves-1)

		timearray = []
		for i in range(0,nummoves):
			if i==0:
				timearray.append(0)
			else:
				timearray.append(timearray[i-1]+singletime)
	
		currenttime = (time%gaittime)/singletime
		globalref = gait[int(math.ceil(currenttime))+1][module]
		globalprev = gait[int(math.floor(currenttime))+1][module]

		if globalref==globalprev:
			# If the current time coincides with the time of a given gait angle direction, then there is no need to interpolate.
			localref = globalref
		else:
			# Linear interpolation step.
			interp = (currenttime*singletime - timearray[int(math.floor(currenttime))])/(timearray[int(math.ceil(currenttime))] - timearray[int(math.floor(currenttime))])
			localref = globalprev + interp*(globalref-globalprev)

		return localref


	def run(self):
		"""
		Start the demo. This method will block until the demo exits.
		This method is used if the simulator is run stand-alone.
		"""

		self._running = True

		# Receive Locomotion commands for all the hinges from LTLMoP.
		# Use these commands as reference angles for simple P-controlled servos.	
		while self._running:
			self.rungait()


	def run_once(self):
		"""
		Run one simulation step -- used for LTLMoP integration.
		"""

		self._running = True

		# Receive Locomotion commands for all the hinges from LTLMoP.
		# Use these commands as reference angles for simple P-controlled servos.
		self.rungait()


# Main method for standalone mode.
if (__name__ == '__main__'):
	"""
	Instantiates a simulator and runs it in stand-alone mode with all default arguments.
	"""

	print info

	robotfile = "config/" + sys.argv[1] + ".ckbot"

	inst = CKBotRun(robotfile)

	if len(sys.argv)==3:
        	inst.gait = int(sys.argv[2])
	else:
		inst.gait = 1

	inst.run()
