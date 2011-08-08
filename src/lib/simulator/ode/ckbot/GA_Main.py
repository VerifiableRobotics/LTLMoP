import sys, random, math
from copy import *

import CKBotSimEngine 
import CKBotSim
from fitness_function import *
from CKBotSimHelper import *

# Main method.
if (__name__ == '__main__'):

	# Important Parameters to define for GA.
	POPULATION_SIZE = 25
	GENERATIONS = 30
	SIMULATION_STEPS = 350
	CROSSOVER_RATE = 0.5
	MUTATION_RATE = 0.15

	# Rigid modules array to reduce the search space.
	# TODO: Make this more user-friendly to enter?
	rigid_modules = []
	
	filename = raw_input("\nEnter the desired file name ('none' for no saving): ")
	if filename != "none":
		# FILE 1: Gene and score informations.
		f_gene = open("GA_Data/"+filename+".genes", 'w')
		# FILE 2: Pose information for post-processing.
		f_pose = open("GA_Data/"+filename+".poses", 'w')
	
	# Look at the arguments passed in. The first argument is the configuration file and all the others
	# correspond to the traits that will define the fitness function.
	robotfile = "config/" + sys.argv[1] + ".ckbot"
	traits = []
	for i in range(2,len(sys.argv)):
		traits.append(sys.argv[i])
	
	# Write the traits and other information to the text file.
	if filename != "none":
		trait_string = ""
		for i in range(len(traits)):
			if i == len(traits) - 1:
				trait_string = trait_string + traits[i]
			else:
				trait_string = trait_string + traits[i] + ", "
		f_gene.write(trait_string+"\n")
		f_gene.write(str(POPULATION_SIZE) + "\n")
		f_gene.write(str(GENERATIONS) + "\n")
		
	# STATE REPRESENTATION NOTES [PERIODIC GAIT GA]
	
	# For each module, the piece of the genome is as follows:
	#	XYZ, where "X" is the amplitude, "Y" is the frequency and "Z" is the phase.
	
	#   AMPLITUDE: 00 = 0 degrees to 75 = 75 degrees
	#		The amplitude is in degrees.
	
	#	FREQUENCY: 0 = 0 rad/s to 9 = 9 rad/s
	#		Since frequencies are relative then any multiplier can just be added later.
	
	#   PHASE:     0 = 0 degrees to 7 = 315 degrees 
	#		To get the phase, multiply the value of the chromosome by 45.
	
	# Initialize a population.
	sim = CKBotSimEngine.CKBotSim(robotfile)
	num_modules = len(sim.connM)
	population = []
	scorelist = []
	poselist = []
	
	# Use the list of rigid modules to make a list of free modules.
	free_modules = range(num_modules)
	for elem in rigid_modules:
		free_modules.remove(elem)
	
	# Initialize output parameters for post-processing
	best_score = 0
	best_gene = None
	best_generation = 0
	best_member = 0
	
	for i in range(POPULATION_SIZE):
		temprow = []
		for j in range(len(free_modules)):
			temprow.extend([5*random.randint(0,13), random.randint(0,5), random.randint(0,7)])
		population.append(temprow)
		
	##############	
	# MAIN LOOP: #
	##############
	for idx in range(GENERATIONS):
	
		# Run each population member and score it.
		scores = []
		for i in range(POPULATION_SIZE):
			gene = population[i]
			instance = CKBotSimEngine.CKBotSim(robotfile)
			set_periodic_gait_from_GA(instance, gene, instance.gain, free_modules)
			instance.run(SIMULATION_STEPS)
			fitness = fitness_function(instance, traits)
			scores.append(fitness)
			poses = instance.pose_info
			
			# Write all the information to text files for post_processing.
			if filename != "none":
			
				# Write gene/fitness information.
				str_out = ""
				for j in range(len(gene)):
					str_out = str_out + str(gene[j]) + " "
				f_gene.write(str_out+"\n")
				f_gene.write(str(fitness)+"\n")
				
				# Write pose information
				for j in range(len(poses)):
					temppose = poses[j][0]	# Each "temppose" is a single pose for the base module.
					f_pose.write(str(temppose[0]) + "\n" + str(temppose[1]) + "\n" + str(temppose[2]) + "\n" + str(temppose[3]) + "\n")
			
		# Update to see if we can find a new best gene.
		for i in range(POPULATION_SIZE):
			if scores[i] > best_score:
				best_score = copy.deepcopy(scores[i])
				best_gene = population[i]
				best_generation = idx
				best_member = i
				
		### RESAMPLING STEP ###
		# Do this every step but the last one.
		if idx != GENERATIONS - 1:
		
			# Ensure there are no negative scores because this messes up the weighting.
			minscore = min(scores)
			if minscore < 0:
				for elem in scores:
					elem = elem - minscore
					
			# Create new population by roulette selection based on scores.
			# We also have crossover and mutation rates to keep track of.
			new_members = []
		
			# Build the selection array.
			selection_array = [scores[0]]
			for i in range(1,len(scores)):
				selection_array.append(selection_array[i-1] + scores[i])
			
			for i in range(POPULATION_SIZE):
				
				# STEP 1: Roulette select one member of the population.
				rand_num = random.uniform(0,sum(scores))
				chosen = False
				counter = 0
				while not chosen:
					if rand_num <= selection_array[counter]:
						chosen = True
					else:
						counter = counter + 1
				new_member = population[counter]
				
				# STEP 2: CROSSOVER
				# 2a. If there is crossover, pick a second member that isn't the same as the first
				if (random.random) > CROSSOVER_RATE:
					new_scores = deepcopy(scores)
					new_population = deepcopy(population)
					len_p = len(new_population)
					for j in range(len_p):
						if new_population[len_p - 1 - j] == new_member and len(new_population)>1:
							new_scores.pop(len_p - 1 - j)
							new_population.pop(len_p - 1 - j)
							
					
					new_selection_array = [new_scores[0]]
					for j in range(1,len(new_scores)):
						new_selection_array.append(new_selection_array[j-1] + new_scores[j])
						
					rand_num = random.uniform(0,sum(new_scores))
					chosen = False
					counter = 0
					while not chosen:
						if rand_num <= new_selection_array[counter]:
							chosen = True
						else:
							counter = counter + 1
					crossover_member = new_population[counter]
								
				# 2b. Pick a random crossover point and build the new member.
					crossover_index = random.randint(len(new_member))
					direction = random.random()
					temp = []
					
					# Direction 1: First member, then second member.
					if direction >= 0.5:
						for k in range(crossover_index):
							temp.append(new_member[k])
						for k in range(crossover_index,len(new_member)):
							temp.append(crossover_member[k])
					# Direction 2: Second member, then first member.
					else:
						for k in range(crossover_index):
							temp.append(crossover_member[k])
						for k in range(crossover_index,len(new_member)):
							temp.append(new_member[k])
					new_member = temp
				
				# STEP 3: If there is mutation, mutate a random value.
				if (random.random) > MUTATION_RATE:
					mutation_index = random.randint(len(new_member))
					
					# If we pick a value to mutate corresponding to the first amplitude value, we only go from 0 to 7.
					if mod(mutation_index,3) == 0:
						mutated = False
						while not mutated:
							randnum = 5*random.randint(0,13)
							if randnum != new_member[mutation_index]:
								mutated = True
								new_member[mutation_index] = randnum	
								
													
					# If we pick a value to mutate corresponding to the phase value, we only go from 0 to 7.
					elif mod(mutation_index,3) == 2:
						mutated = False
						while not mutated:
							randnum = random.randint(0,7)
							if randnum != new_member[mutation_index]:
								mutated = True
								new_member[mutation_index] = randnum				
					
					else:
						mutated = False
						while not mutated:
							randnum = random.randint(0,5)
							if randnum != new_member[mutation_index]:
								mutated = True
								new_member[mutation_index] = randnum
					
				# STEP 4: Add to the new generation.
				new_members.append(new_member)
		
			population = new_members
		
		print "GENERATION " + str(idx+1)
		print "Maximum Score: " + str(max(scores))
		print "Average Score: " + str(mean(scores))
	
	# Print the best score and generation it occured in.
	print "\nBest Score: " + str(best_score)
	print "Generation: " + str(best_generation) + "\n"
	
	# Print (and write) the best gait for copying to a .ckbot file.
	s = CKBotSim.CKBotSim(robotfile, standalone=1)
	best_gait = set_periodic_gait_from_GA(s, best_gene, s.gain, free_modules)	
	
	f_gait = open("GA_Data/"+filename+".gait", 'w')
	print "Best Gait (Copy to .ckbot file):\n"
	print "Type Periodic"
	f_gait.write("Type Periodic\n")
	counter = 0
	for row in best_gait[0][1:4]:
		str_out = ""
		for elem in row:
			# Convert amplitudes and phases to 100s of degrees.
			if counter != 1:
				elem = elem*18000.0/math.pi
			str_out = str_out + str(int(elem)) + " "
		counter = counter + 1
		print str_out
		f_gait.write(str_out + "\n")
	f_gait.close()
				
	if filename != "none":
		# Finish writing
		f_gene.close()
		
		f_pose.write(str(SIMULATION_STEPS+1)+"\n")
		f_pose.write(str(best_generation)+"\n"+str(best_member)+"\n")
		f_pose.close()

	# Finally, simulate the best gait.
	selected = False
	while not selected:
		option = raw_input("\nWould you like to simulate the best gait? (Y/N): ")
		if option.lower() == "y" or option.lower() == "n":
			selected = True
	if option.lower() == "y":
		s.run()