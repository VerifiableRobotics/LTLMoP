import sys, random, math
from copy import *

import CKBotSimEngine 
import CKBotSim
from fitness_function import *
from CKBotSimHelper import *

# Main method.
if (__name__ == '__main__'):

	# Important Parameters to define for GA.
	POPULATION_SIZE = 30
	GENERATIONS = 20
	SIMULATION_STEPS = 350
	CROSSOVER_RATE = 0.65
	MUTATION_RATE = 0.15

	# Look at the arguments passed in. The first argument is the configuration file and all the others
	# correspond to the traits that will define the fitness function.
	robotfile = "config/" + sys.argv[1] + ".ckbot"
	traits = []
	for i in range(2,len(sys.argv)):
		traits.append(sys.argv[i])
		
	# STATE REPRESENTATION NOTES [PERIODIC GAIT GA]
	
	# For each module, the piece of the genome is as follows:
	#	XXYZ, where "XX" is the amplitude, "Y" is the frequency and "Z" is the phase.
	
	#   AMPLITUDE: 00 = 0 degrees to 61 = 65 degrees
	#		The first number is 10 degrees and the second number is 5 degrees.
	#		To reduce the state space, the second number will only be 0 or 1 (where 1 = 5 degrees)
	
	#	FREQUENCY: 0 = 0 rad/s to 9 = 9 rad/s
	#		Since frequencies are relative then any multiplier can just be added later.
	
	#   PHASE:     0 = 0 degrees to 7 = 315 degrees 
	#		To get the phase, multiply the value of the chromosome by 45.
	
	# Initialize a population.
	sim = CKBotSimEngine.CKBotSim(robotfile)
	num_modules = len(sim.connM)
	populations = []
	scorelist = []
	poselist = []
	
	temp_population = []
	for i in range(POPULATION_SIZE):
		temprow = []
		for j in range(num_modules):
			temprow.extend([random.randint(7), random.randint(2), random.randint(10), random.randint(8)])
		temp_population.append(temprow)
	populations.append(temp_population)
		
	##############	
	# MAIN LOOP: #
	##############
	for idx in range(GENERATIONS):
	
		# Run each population member and score it.
		scores = []
		poses = []
		for i in range(POPULATION_SIZE):
			instance = CKBotSimEngine.CKBotSim(robotfile)
			set_periodic_gait_from_GA(instance, populations[idx][i])
			instance.run(SIMULATION_STEPS)
			scores.append(fitness_function(instance, traits))
			poses.append(instance.pose_info)
			
		# Do this every step but the last one.
		if idx != GENERATIONS - 1:
		
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
				new_member = populations[idx][counter]
				
				# STEP 2: CROSSOVER
				# 2a. If there is crossover, pick a second member that isn't the same as the first
				if (random.random) > CROSSOVER_RATE:
					new_scores = deepcopy(scores)
					new_scores.pop(counter)
					new_population = deepcopy(populations[idx])
					new_population.pop(counter)
					
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
					
					# If we pick a value to mutate corresponding to the first amplitude value, we only go from 0 to 6.
					if mod(mutation_index,4) == 0:
						mutated = False
						while not mutated:
							randnum = random.randint(7)
							if randnum != new_member[mutation_index]:
								mutated = True
								new_member[mutation_index] = randnum	
								
					# If we pick a value to mutate corresponding to the second amplitude value, only toggle between 0 and 5.
					# Do this only if the amplitude is not 90, since 95 is not a valid value.
					elif mod(mutation_index,4) == 1:
						if new_member[mutation_index] == 0:
							new_member[mutation_index] = 1
						else:
							new_member[mutation_index] = 0
													
					# If we pick a value to mutate corresponding to the phase value, we only go from 0 to 7.
					elif mod(mutation_index,4) == 3:
						mutated = False
						while not mutated:
							randnum = random.randint(8)
							if randnum != new_member[mutation_index]:
								mutated = True
								new_member[mutation_index] = randnum				
					
					else:
						mutated = False
						while not mutated:
							randnum = random.randint(10)
							if randnum != new_member[mutation_index]:
								mutated = True
								new_member[mutation_index] = randnum
					
				# STEP 4: Add to the new generation.
				new_members.append(new_member)
			
			populations.append(new_members)
		scorelist.append(deepcopy(scores))
		poselist.append(deepcopy(poses))
		
		print "GENERATION " + str(idx+1)
		print "Maximum Score: " + str(max(scores))
		print "Average Score: " + str(mean(scores))
	
	# Find the best gait as per the scores assigned.
	best_row = 0
	best_column = 0
	max_score = 0
	gene = []
	for i in range(len(populations)):
		temprow = populations[i]
		for j in range(len(temprow)):
			if scorelist[i][j] > max_score:
				gene = populations[i][j]
				max_score = scorelist[i][j]
				best_row = i
				best_column = j
	print "\nBest Score: " + str(max_score)
	print "Generation: " + str(best_row) + "\n"
	
	s = CKBotSim.CKBotSim(robotfile, standalone=1)
	best_gait = set_periodic_gait_from_GA(s, gene)
	
	# Print the best gait for copying to a .ckbot file.
	print "Best Gait (Copy to .ckbot file):\n"
	print "Type Periodic"
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
	
	# Write all the information to text files for post_processing.
	
	# FILE 1: Gene and score informations.
	filename = raw_input("\nEnter the desired file name ('none' for no saving): ")
	
	if filename != "none":
		f = open("GA_Data/"+filename+".genes", 'w')
		
		trait_string = ""
		for i in range(len(traits)):
			if i == len(traits) - 1:
				trait_string = trait_string + traits[i]
			else:
				trait_string = trait_string + traits[i] + ", "
		f.write(trait_string+"\n")
		
		f.write(str(POPULATION_SIZE)+"\n")
		for i in range(len(populations)):
			temprow = populations[i]
			for j in range(len(temprow)):
				gene = temprow[j]
				str_out = ""
				for k in range(len(gene)):
					str_out = str_out + str(gene[k]) + " "
				f.write(str_out+"\n")
				f.write(str(scorelist[i][j])+"\n")
				
		f.close()
		
		# FILE 2: Pose information for post-processing.
		f = open("GA_DATA/"+filename+".poses", 'w')
		
		f.write(str(SIMULATION_STEPS+1)+"\n")
		f.write(str(best_row)+"\n"+str(best_column)+"\n")
		for i in range(len(poselist)):
			temprow = poselist[i]				# Each "temprow" is all the poses for a generation.
			for j in range(len(temprow)):
				tempposes = temprow[j]			# Each "tempposes" is all the poses for a single member.
				for k in range(len(tempposes)):
					temppose = tempposes[k][0]	# Each "temppose" is a single pose for the base module.
					f.write(str(temppose[0]) + "\n" + str(temppose[1]) + "\n" + str(temppose[2]) + "\n" + str(temppose[3]) + "\n")
			
		f.close()
		
	s.run()