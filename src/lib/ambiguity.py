import nltk
from nltk.featstruct import TYPE

def showParseDiffs(parseTuples):
	''' Takes in a list of tuples of nltk.tree.Tree objects and strings (semantic interpretations)
		and prints a line for each tree showing its disambiguated interpretation using parentheses
	'''

	revisedStrings = []
	trimmedTrees = removeCommonRoots([tree for (tree, semrep) in parseTuples])
	for tree in trimmedTrees:
		revisedStrings.append(makeExplicitScope(tree)[1:-1])

	print('\nDid you mean: \n' + '\n~or~\n'.join(revisedStrings))
	print('\nIf the first option is not your intended meaning, please revise your sentence with parentheses.')

def removeCommonRoots(parseTrees):
	'''	Takes in a list of nltk.tree.Tree items and returns a list of the same trees after removing
		all top-level items that are common to every tree in the list
	'''

	#Our tree position indices will be based on the first parse tree we're given.
	#The indices from treepositions() follow a depth-first traversal, but by sorting the
	#indices in order of their length (depth in tree) we obtain the breadth-first indices.
	positions = sorted(parseTrees[0].treepositions(), key=lambda tree: len(tree))

	for position in positions:
		refItem = getTreeItem(parseTrees[0], position)
		for parseTree in parseTrees:
			if getTreeItem(parseTree, position) != refItem:
				return [tree[position[:-1]] for tree in parseTrees]
	return parseTrees

def makeExplicitScope(tree):
	revString = ''
	if isinstance(tree, nltk.tree.Tree):
		if len(tree) > 1:
			revString = '('
			for subTree in tree:
				revString += makeExplicitScope(subTree)
			if revString[-1] == ' ': revString = revString[:-1]
			revString += ') '
		else:
			revString = makeExplicitScope(tree[0])
	else:
		revString = tree+' '

	return revString

def getTreeItem(tree, indexTuple):
	for i in indexTuple:
		if i >= len(tree) or not isinstance(tree, nltk.tree.Tree):
			return None
		else:
			tree = tree[i]
	if isinstance(tree, nltk.tree.Tree):
		return tree.node[TYPE]
	else:
		return tree

def diffParses(parseTrees, startPosition = ()):

	#Our tree position indices will be based on the first parse tree we're given.
	#The indices from treepositions() follow a depth-first traversal, but by sorting the
	#indices in order of their length (depth in tree) we obtain the breadth-first indices.
	positions = sorted(parseTrees[0].treepositions(), key=lambda tree: len(tree))

	if startPosition in positions:
		positions = positions[positions.index(startPosition):]

	#List of tuples (parseTree, revisedString)
	revisedTuples = []

	#We proceed through every position in the tree..
	for positionTuple in positions:
		#A dictionary maps values at the current tree position to parse tuples
		ValuesToParses = {}
		#We evaluate each parse tree at the current position..
		for synTree in parseTrees:
			value = getTreeItem(synTree, positionTuple)
			print('Value at '+str(positionTuple)+': '+str(value))
			if value in ValuesToParses:
				ValuesToParses[value].append(synTree)
			else:
				ValuesToParses[value] = [synTree]

		if len(ValuesToParses.keys()) > 1:
			#Trees diverge at this position!
			print('Trees diverge at this position!')
			for value in ValuesToParses.keys():
				locallyEqualParses = ValuesToParses[value]
				if len(locallyEqualParses) > 1:
					#Get disambiguated strings from recursing
					print('Making inner diff on trees with value:\''+str(value)+'\' at '+str(positionTuple))
					innerRevisedTuples = diffParses(locallyEqualParses,positionTuple)
					for (tree, string) in innerRevisedTuples:
						print('Returning to outer diff on trees with value:\''+str(value)+'\' at '+str(positionTuple))
						print('Inner string at this position is: '+string)
						revisedString = makeRevisedString(tree, positionTuple,'['+string+']')
						revisedTuples.append((tree, revisedString))
						print("Revised string: "+revisedString)
				else:
					#Generate a disambiguated string by grouping siblings of current node
					print("Making revised string from tree with local value "+str(value)+"...")
					revisedString = makeRevisedString(locallyEqualParses[0], positionTuple)
					revisedTuples.append((locallyEqualParses[0], revisedString))
					print("Revised string: "+revisedString)
			break;


	return revisedTuples

def getTreeSiblings(tree, indexTuple):
	positions = tree.treepositions();
	i = 1
	siblings = filter(lambda x: x[0:-1] == indexTuple[0:-i], positions)

	while len(siblings) < 2:
		i = i + 1
		siblings  = filter(lambda x: x[0:-1] == indexTuple[0:-i], positions)

	return siblings

def makeRevisedString(tree, indexTuple, partialString = None):
	revString = ''
	siblings = getTreeSiblings(tree, indexTuple)
	print("Found siblings: "+str(siblings))
	siblingItems = []
	for index in siblings:
		if index == indexTuple and partialString is not None:
			siblingItems.append([partialString])
		elif isinstance(tree[index], nltk.tree.Tree):
			siblingItems.append(tree[index].leaves())
		else:
			print("Some sibling is not a tree... this shouldn't happen! Item is: "+str(tree[index]))
			siblingItems.append([tree[index]])
	for item in siblingItems:
		if len(item) > 1:
			revString += '(' + ' '.join(item) + ') '
		else:
			revString += ' '.join(item) + ' '
	return revString
