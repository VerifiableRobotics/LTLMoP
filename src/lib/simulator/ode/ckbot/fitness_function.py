# Genetic Algorithm fitness function package.

from math import *
from numpy import *

def fitness_function(instance, traits):
	"""
	Score the performance of a configuration/gait pair based on all the prescribed traits.
	"""
	
	score = 1.0
	for trait in traits:
		score = score*trait_score(instance, trait)
	return score

	
def trait_score(instance, trait):
	"""
	Score the performance of a configuration/gait pair based on a single trait.
	"""
	
	# Unpack pose data
	data = instance.pose_info
	
	# Reference for indexing the "data" variable:
	# Index 1: Time step 
	# Index 2: Module number 
	# Index 3: [ X-coordinate, Y-coordinate, Angle, Z-coordinate ]
	
	# FAST: Score higher on total distance moved by base module
	if trait == "Fast":
		initial_data = data[0][0]
		final_data = data[len(data)-1][0]
		score = pow(final_data[0]-initial_data[0],2) + pow(final_data[1]-initial_data[1],2)
				
	# 1D_MOTION: Score is high if the orientation of the robot does not change much (low mean and standard deviation)
	elif trait == "1DMotion":
		angles = []
		for i in range(len(data)):
			angles.append( data[i][0][2] )
		score = 1.0/pow((std(angles)*abs(mean(angles))),0.5)
	
	# FORWARD: Score is high if there is motion in the X direction and not much in the Y direction
	elif trait == "Forward":
		initial_x = data[0][0][0]
		initial_y = data[0][0][0]
		final_x = data[len(data)-1][0][0]
		y_data = []
		for i in range(len(data)):
			y_data.append( data[i][0][1] )
		max_y = max(y_data)
		min_y = min(y_data)
		if final_x > initial_x:
			score = pow(final_x - initial_x, 2)/pow(max(max_y, abs(min_y) - initial_y) , 2)
		else:
			score = 0
	
	# BACKWARD: Score is high if there is motion in the -X direction and not much in the Y direction
	elif trait == "Backward":
		initial_x = data[0][0][0]
		initial_y = data[0][0][0]
		final_x = data[len(data)-1][0][0]
		y_data = []
		for i in range(len(data)):
			y_data.append( data[i][0][1] )
		max_y = max(y_data)
		min_y = min(y_data)
		if final_x < initial_x:
			score = pow(initial_x - final_x, 2)/pow(max(max_y, abs(min_y) - initial_y) , 2)
		else:
			score = 0
			
	# TURN IN PLACE: Score is high if the robot changes in angle and its base module does not move much.
	elif trait == "TurnInPlaceLeft":
		initial_data = data[0][0]
		final_data = data[len(data)-1][0]
		distance_score = pow(1.0/(pow(final_data[0]-initial_data[0],2) + pow(final_data[1]-initial_data[1],2)),0.5)
		score = distance_score*trait_score(instance, "TurnLeft")

	elif trait == "TurnInPlaceRight":
		initial_data = data[0][0]
		final_data = data[len(data)-1][0]
		distance_score = pow(1.0/(pow(final_data[0]-initial_data[0],2) + pow(final_data[1]-initial_data[1],2)),0.5)
		score = distance_score*trait_score(instance, "TurnRight")
	
	
	# TURN: Score is high if the robot changes in angle. Base module translation has no effect
	elif trait == "TurnLeft":
		changes = []
		for i in range(10,len(data),10):
			changes.append(data[i][0][2]-data[i-10][0][2])
		score = mean(changes)
		if score < 0:
			score = 0
	
	elif trait == "TurnRight":
		changes = []
		for i in range(10,len(data),10):
			changes.append(data[i][0][2]-data[i-10][0][2])
		score = -mean(changes)
		if score < 0:
			score = 0	
			
	# TALL: Score is high if the base module remains as far from the ground as possible.
	elif trait == "Tall":
		heights = []
		for i in range(len(data)):
			heights.append( data[i][0][3] )
		score = pow(mean(heights),2)
	
	# LOW: Score is high if all modules remain as close to the ground as possible.
	elif trait == "Low":
		heights = []
		for i in range(len(data)):
			temprow = data[i]
			for j in range(len(temprow)):
				heights.append( data[i][j][3] )
		score = 1.0/pow(mean(heights),2)
	
	else:
		score = 0.0

	return score