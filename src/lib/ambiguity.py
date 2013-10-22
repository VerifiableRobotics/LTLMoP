import nltk
from collections import deque

def showParseDiffs(parseTuples):

	nParses = len(parseTuples)

	treesDiverged = False

	#Initialize a list of queues of nodes, one for each parse tree
	nodeQueues = []
	for parseTuple in parseTuples:
		nodeQueues.append(deque(parseTuple[0]))

	#Initialize lists representing the current tier of each parse tree during the breadth-first search
	currentTiers = []
	for parseTuple in parseTuples:
		currentTiers.append(parseTuple[0][:])

	while not treesDiverged:
		#Execute one step of a breadth-first search on all parse trees
		for i in range(nParses):
			currentNode = nodeQueues[i].popleft()
			currentTiers[i] = []
			for child in currentNode: 
				if isinstance(child, nltk.tree.Tree):
					nodeQueues[i].append(child)
					currentTiers[i].append(child)

		#Check current tiers to see if trees have diverged yet
		for i in range(nParses - 1):
			#If two tiers have a different number of elements, the trees have diverged
			if len(currentTiers[i]) != len(currentTiers[i + 1]):
				treesDiverged = True
				break

			for j in range(len(currentTiers[i])):
				#If any nodes in two tiers differ in type, the trees have diverged
				if currentTiers[i][j].node.values()[1] != currentTiers[i + 1][j].node.values()[1]:
					treesDiverged = True
					break

	#When loop exits, we know that the trees in question diverge at the current tier

	revisedSemStrings = ['' for currentTier in currentTiers]

	for i in range(nParses):
		for node in currentTiers[i]:
			if len(node) > 1:
				revisedSemStrings[i] += '(' + ' '.join(node.leaves()) + ') '
			else:
				revisedSemStrings[i] += ' '.join(node.leaves()) + ' '


	print('\nDid you mean: \n' + '\n~or~\n'.join(revisedSemStrings))
	print('\nIf the first option is not your intended meaning, please revise your sentence with parentheses.')