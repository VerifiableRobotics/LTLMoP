#!/usr/bin/env python

import math, time, copy, sys
sys.path.append("../../../../../Downloads/CKBot/trunk")
from ckbot.logical import Cluster

"""
CKBot Hardware Calibration Script
[Sebastian Castro - Cornell University]
[Autonomous Systems Laboratory - 2011]
"""

if (__name__ == '__main__'):
	
	# Instantiate a Cluster
	print "Generating Cluster...\n"
	c = Cluster()
	c.populate()

	# Prompt the user to confirm is the number of modules detected is correct.
	print "There are %d modules in the current cluster." %len(c)
	okay = raw_input("Is this OK? Y/N: ")
	
	# Quit if the user does not agree with the number of modules detected in the cluster.
	if (okay.lower()=="n"):
		pass
	else:
		# Continue prompting the user to enter yes(Y) or no(N) if they enter anything else.
		while(okay.lower()!="y"):
			okay = raw_input("Please enter Y/N: ")
	
		# Loop through all the modules and prompt the user to say what number the module is.
		module_counter = 0
		entered_numbers = []

		# Prompt the user to enter the configuration being calibrated
		config_name = raw_input("\nWhich configuration is being calibrated?: ")

		print "\n"
		for idx in c:

			# Move the module around for the user to see.
			print "Calibrating Module %d, ID #%d" %(module_counter, idx)
			c[idx].set_pos(3000)
			time.sleep(1)
			c[idx].set_pos(-3000)
			time.sleep(1)
			c[idx].set_pos(0)
			time.sleep(1)

			# If the user already entered this number prompt again to enter a valid one.
			module_number = int(raw_input("Which module is this? (0 to %d): " %(len(c)-1)))
			while (module_number in entered_numbers):
				module_number = int(raw_input("Already entered this number. Please try another module number (0 to %d): " %(len(c)-1)))
			entered_numbers.append(module_number)

		
		# Find the ordered sequence of Cluster dictionary keys
		key_orders = []
		print "\nRearranging Cluster...\n"
		for idx in range(len(c)):
			new_idx = entered_numbers.index(idx)
			key_orders.append(c.keys()[new_idx])
		print "The Cluster key orders are:"
		print key_orders

		# Generate the string to write to the text file.
		string_to_add = config_name
		for key in key_orders:
			string_to_add = string_to_add + " " + str(key)

		# Loop through the text file and add the key order for the configuration.
		# Also delete any old data about the configuration if it exists.
		f_read = open("KeyOrders.txt", 'r')

		# Keep only the lines that do not need to be deleted (i.e. old configuration data).
		lines_to_keep = []
		for line in f_read:
			linesplit = line.split()
			if len(linesplit) > 0:
				if (linesplit[0] != config_name):
					lines_to_keep.append(line)

		# Build the contents of the new file to write.
		write_string = ""
		for line in lines_to_keep:
			write_string = write_string + line + "\n"
		write_string = write_string + string_to_add	

		# Close the file reading object and rewrite the file, then close the file writing object.
		f_read.close()
		f_write = open("KeyOrders.txt", 'w')
		f_write.write(write_string)
		f_write.close()




