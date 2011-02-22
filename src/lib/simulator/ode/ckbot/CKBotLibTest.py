# CKBotLib.py
# Takes word specifications and choose a gait

import os

class CKBotLib:

	def __init__(self):
	
		# Make a list of words that describe gaits
		#self.words = ["tall","low","fast","action gait","1D motion","non-holonomic turning",
		#		 "holonomic","handles steps","handles rough surfaces"];
		self.words = []
				 
		# Define Gaits
		# Can add this to text files and parse later
		#gait0 = Gait("Biped-walk",["tall","fast","non-holonomic turning"])
		#gait0.name = "Biped-walk"
		#gait0.words = ["tall","fast","non-holonomic turning"]
		#gait1 = Gait("Biped-split",["action gait"])
		#gait2 = Gait("Loop-roll",["tall","fast","non-holonomic turning"])
		#gait3 = Gait("Loop-somersault",["1D motion","fast"])
		#gait4 = Gait("Plus-crawl",["holonomic","low"])
		#gait5 = Gait("Slinky-slink",["handles steps","tall"])
		#gait6 = Gait("Twist-twist",["low","non-holonomic turning"])
		#gait7 = Gait("Snake-crawl",["non-holonomic turning","low"])
		#gait8 = Gait("Hexapod-run",["fast","handles rough surfaces"])
		#self.poss_gaits = [gait0,gait1, gait2, gait3, gait4, gait5, gait6, gait7, gait8]
		self.poss_gaits = []	
                

	def readLibe(self):
		"""
		Parse library file
		"""
		f = open('library/CKBotTraits.libe','r')

		reading_trait = 0
		gait_number = 0
		cgpair = []
		
		
		# Make temporary gaitnames variable
		gaitnames = [] # Will have same indices of self.poss_gaits, but is a list of names, not gaits
		
		for line in f:
			# Find Config-Gait pairs in a trait
			# if we just read a trait, add config-pairs to self.poss_gaits
			if (reading_trait == 1):
				if line == "\n":
					reading_trait = 0
					# Loop through cgpairs with this specific trait so we can build self.poss_gaits[]
					for pair in cgpair:
						# Make a new gait object if one with the same name doesn't exist
						if (pair not in gaitnames):
							# specify current cgpair with last trait in self.words
							gaitnames.append(pair) 
							self.poss_gaits.append(Gait(pair,[self.words[-1]]))
						# Add trait to already made gait object self.poss_gaits[i].words
						else: 
							idx = gaitnames.index(pair)
							self.poss_gaits[idx].words.append(self.words[-1])
					# Restart cgpair
					cgpair = []
				# Add config-gait to cgpair list
				elif (line[0] != "#") and not (line.strip() in cgpair):
					cgpair.append(line.strip())
			# Find Gait Traits
			if (line.split().count("Trait:")>0): 
				info = line.split(": ")
				self.words.append(info[1].strip())
				gait_number = gait_number + 1
				reading_trait = 1


class Gait:
	def __init__(self,name,words):
		self.name = "Config-gait"
		self.words = None
                self.name = name
                self.words = words

if (__name__ == '__main__'):
	# Test list		 
	libs = CKBotLib()
	libs.readLibe()
	print "These are available definitions:"
	print libs.words

	# Get user input
	desired_gait = raw_input('Enter gait specifications: ')
	#print "desired gait is", desired_gait

	# Parse input
	desired_words = desired_gait.split(" and ")
	#print desired_words

	#print "\nAll possible gaits: "
	#for idx in range(len(libs.poss_gaits)):
	#	print libs.poss_gaits[idx].name

	# Find gaits that meet specification
	gait_match = 0	# does a specific gait match?
	gait_found = 0  # is there at least one gait that matches
	goodgaits = []
	for gait in libs.poss_gaits:
		for word in desired_words:
			if word in gait.words:
				#print "gait", gait.name,"has trait",word
				gait_match = gait_match + 1
		        if gait_match == len(desired_words):
					print "gait is", gait.name
					goodgaits.append(gait.name)
					gait_found = 1
		gait_match = 0

	# If no gaits were found, print "gait not found"
	if gait_found == 0:
		print "gait not found"
	# If a gait was found, run it in the simulator
	else:
		# Use first goodgait if we have more than one gait
		[config,gait] = goodgaits[0].split("-")
		print config
		print gait
		os.system("python lib/simulator/ode/ckbot/CKBotSim.py " + config)

	
