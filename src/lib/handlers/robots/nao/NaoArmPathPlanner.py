import numpy as np
import random
import time
import euclid
class PathPlanner:
    """
    Note:
        Order of limb actuators: is shoulderPitch, shoulderRoll, elbowYaw, and elbowRoll
        Most of the angle parameters for the functions are in degrees
    """
    # NA0 arm angle ranges
    shoulPitchRange = [-119, 119]
    shoulRollRange = [-94, 0]
    elbYawRange = [-119, 119]
    elbRollRange = [2, 88]
    wristRange = [-104, 104]
    armRanges = [shoulPitchRange, shoulRollRange, elbYawRange, elbRollRange, wristRange]
    def __init__(self, safetyRadius = 0, stepRange = [5,15], stepInt = 5, attrPow = 1,
                closeEnough = 50):
        """
        @Param safetyRadious (double): The minimum clearance between limbs
        @Param stepRange (1x2 list): The allowed range of change for each RRT step. In
            in the "order of limb actuators".
        @Param stepInt (double): The interval at which to check for collisions in longer
            paths
        @Param attrPow (double): The attraction towards goal bias
        @Param closeEnough (double): The minimum distance that the end effector must be
            from the goal in order to considered it finished (m)
        """
        Shape3D.safetyRadius = safetyRadius
        self.naoBody = Body()
        self.rrtParam = {"stepRange":stepRange, "stepInt":stepInt, "attrPow":attrPow,
                         "closeEnough":closeEnough}
##    def grabBallMotion(self, robotAngles, ballObject, otherObstacles=None):
##        """ Returns an Nx4 list with the motion for grabbing a ball
##
##        @Param robotAngles (1x26 list): All of the body angles in degrees in the
##        order specified by Alderbaran
##        http://www.aldebaran-robotics.com/documentation/naoqi/motion/control-joint.html#body-chain-def
##        @Param ballObject (Shape3D): A sphere object represinting the ball
##        @Param otherObstacles (list): A list of other Shape3D obstacles to avoid
##        """
##        bodyParts = self.naoBody.getBodyParts(robotAngles)
##        del bodyParts["rightArmParts"]
##        bodyPartsList = []
##        for shapes in bodyParts.values():
##            bodyPartsList.extend(shapes)
##
##        goalXYZ = ballObject.points + np.array([0,0,ballObject.radius + 10])
##        shiftedBP = ballObject.points - \
##                    np.array([0,0,self.naoBody.lowerArmRad + 10])
##        shiftedBall = Shape3D(shiftedBP, ballObject.radius, "sphere")
##
##        armAngles = robotAngles[20:24]
##        if otherObstacles == None:
##            obstacles = bodyPartsList + [shiftedBall]
##        else:
##            obstacles = bodyPartsList + [shiftedBall] + otherObstacles
##        path = self.getPath(armAngles, goalXYZ, obstacles, timeout=60)
##
##        if path == None:
##            return None
##
##        goalWrist = self.alignWrist(path[-1] + [robotAngles[-2]], \
##                                      np.array([0,0,-1]))
##
##        wristDiff = (goalWrist - robotAngles[-2]) / (len(path) - 1)
##        wristAngles = [(robotAngles[-2] + wristDiff*i) for i in range(len(path))]
##
##        fullMotion = [(path[i] + [wristAngles[i]]) for i in range(len(path))]
##
##        return fullMotion
    def grabBallMotion(self, robotAngles, ballObject, otherObstacles=None, timeout=30):
            """ Returns an Nx5 list with the motion for grabbing a ball from a human's
            hand.
            @Param robotAngles (1x26 list): All of the body angles in degrees in the
            order specified by Alderbaran
            http://www.aldebaran-robotics.com/documentation/naoqi/motion/control-joint.html#body-chain-def
            @Param ballObject (Shape3D): A sphere object represinting the ball
            @Param otherObstacles (list): A list of other Shape3D obstacles to avoid
            """
            timeS = time.clock()
            bodyParts = self.naoBody.getBodyParts(robotAngles)
            del bodyParts["rightArmParts"]
            bodyPartsList = []
            for shapes in bodyParts.values():
                bodyPartsList.extend(shapes)
            if otherObstacles != None:
                obstacles = bodyPartsList + otherObstacles
            else:
                obstacles = bodyPartsList
            # Choose grasping location
            armAngles = robotAngles[20:24]
            graspGoals = None
            pathGoals = None
            # Grab Left or Right
            if ballObject.points[1] > (self.naoBody.rShoulderLoc[1] - 20):
                dirV = np.array([0,-1,0])
            else:
                dirV = np.array([0,1,0])
            graspXYZ = ballObject.points + dirV*ballObject.radius
            pathXYZ = graspXYZ + dirV*(self.naoBody.lowerArmRad + 5)
            graspGoals = self.getControlGoals(graspXYZ, obstacles)
            if graspGoals != None:
                pathGoals = self.getControlGoals(pathXYZ, obstacles + [ballObject])
            # Grab Up or down
            if graspGoals == None or pathGoals == None:
                if ballObject.points[2] > self.naoBody.rShoulderLoc[2]:
                    dirV = np.array([0,0,-1])
                else:
                    dirV = np.array([0,0,1])
                graspXYZ = ballObject.points + dirV*ballObject.radius
                pathXYZ = graspXYZ + dirV*(self.naoBody.lowerArmRad + 5)
                graspGoals = self.getControlGoals(graspXYZ, obstacles)
                if graspGoals != None:
                    pathGoals = self.getControlGoals(pathXYZ, obstacles + [ballObject])
            if graspGoals == None or pathGoals == None:
                print "Could not find feasable goals"
                return None
            if DEBUG: print "FOUND THE GOALS"
            pathGoals = self.eliminateCloseAngles(pathGoals, 40)
            path = self.runRRT(armAngles, pathGoals, obstacles + [ballObject], \
                               timeout - (time.clock() - timeS))
            if path == None:
                print "Did not find a propper path"
                return None
            path = self.smoothedPath(path, obstacles + [ballObject])
            path = [list(p.angles) for p in path]
            if DEBUG: print "GOT THE PATH"
            finalP = self.findClosestAngle(path[-1], graspGoals)
            path.append(list(finalP))
            # Allign the wrist
            _, _, handP, _ = self.naoBody.getArmPositions(path[-1], "right")
            wristDir = ballObject.points - handP
            goalWrist = self.alignWrist(path[-1], wristDir)
            wristDiff = (goalWrist - robotAngles[-2]) / (len(path) - 1)
            wristAngles = [(robotAngles[-2] + wristDiff*i) for i in range(len(path))]
            fullMotion = [(path[i] + [wristAngles[i]]) for i in range(len(path))]
            return fullMotion
    def pathToPosition(self, robotAngles, position, otherObs=None, timeout=30):
            """ Returns an Nx4 list with the motion moving the hand to the desired
            position
            @Param robotAngles (1x26 list): All of the body angles in degrees in the
            order specified by Alderbaran
            http://www.aldebaran-robotics.com/documentation/naoqi/motion/control-joint.html#body-chain-def
            @Param position (1x3 list): The position to move the hand to in mm
            @Param otherObs (list): A list of other Shape3D obstacles to avoid
            """
            timeS = time.clock()
            bodyParts = self.naoBody.getBodyParts(robotAngles)
            del bodyParts["rightArmParts"]
            bodyPartsList = []
            for shapes in bodyParts.values():
                bodyPartsList.extend(shapes)
            if otherObs != None:
                obstacles = bodyPartsList + otherObs
            else:
                obstacles = bodyPartsList
            # Calculate goals in control space
            armAngles = robotAngles[20:24]
            position = np.array(position)
            goals = self.getControlGoals(position, obstacles)
            if goals == None:
                print "Could not find feasable goals"
                return None
            if DEBUG: print "FOUND THE GOALS"
            goals = self.eliminateCloseAngles(goals, 40)
            path = self.runRRT(armAngles, goals, obstacles, \
                               timeout - (time.clock() - timeS))
            if path == None:
                print "Did not find a propper path"
                return None
            path = self.smoothedPath(path, obstacles)
            path = [list(p.angles) for p in path]
            if DEBUG: print "GOT THE PATH"
            return path
    def runRRT(self, fromAngles, toGoals, obstacles = None, timeout = 60):
        """ Returns a list of nodeWeighted representing the path from the start
        to one of the goals.
        @Param fromAngles (1x4 list): A four dimensional vector with the current arm
            angles in "order of limb actuators"
        @Param toGoals (Mx4 list): A list of possible goals to reach in control space
        @Param obstacles (1xR list): A list of Shape3D obstacles that should be avoided.
        @Param timeout (int): The max amount of time to spend before giving up
        """
        fromAngles = np.array(fromAngles)
        toGoals = [np.array(g) for g in toGoals]
        stepRange = self.rrtParam["stepRange"]
        attrPow = self.rrtParam["attrPow"]
        closeEnough = self.rrtParam["closeEnough"]
        nodeList = []                   # Contains all of the sampled nodes
        weightList = []                 # A list of the weights for the node list
        weightSum = 0
        # Get max distance from goal within arm domain
        maxDist = 0
        for a in range(2):
            for b in range(2):
                for c in range(2):
                    for d in range(2):
                        angsT = np.array([self.shoulPitchRange[a], \
                                             self.shoulRollRange[b], \
                                             self.elbYawRange[c], \
                                             self.elbRollRange[d]])
                        for goal in toGoals:
                            distV = angsT - goal
                            distT = np.sqrt(distV.dot(distV))
                            if distT > maxDist:
                                maxDist = distT
        minDist = float("inf")
        for goal in toGoals:
            distTV = fromAngles - goal
            distT = np.sqrt(distTV.dot(distTV))
            if distT < minDist:
                minDist = distT
        weight = (maxDist - minDist)**attrPow
        newNode = self.nodeWeighted(fromAngles, None, None, weight)
        nodeList.append(newNode)
        weightList.append(weight)
        weightSum += weight
        finished = False
        finalNode = None
        randAngs = [None] * 4
        startT = time.clock()
        while (time.clock() - startT) < timeout and not finished:
            if DEBUG2: print len(nodeList)
            # Randomly select a node
            randI = np.random.choice(range(len(nodeList)), \
                    p = np.divide(weightList, weightSum))
            # Move a random distance from it
            for i in range(4):
                randAngs[i] = random.randrange(stepRange[0], stepRange[1])
                randAngs[i] *= random.randrange(-1,2,2)    # Randomly choose sign
            newAngs = nodeList[randI].angles + np.array(randAngs)
            # Check within range
            if self.anglesOutOfRange(newAngs):
                continue
            # Check if path to new node is free
            if self.pathIsNotFree(newAngs, nodeList[randI], obstacles):
                continue
            minDist = float("inf")
            for goal in toGoals:
                distTV = newAngs - goal
                distT = np.sqrt(distTV.dot(distTV))
                if distT < minDist:
                    minDist = distT
            weight = (maxDist - minDist)**attrPow
            newNode = self.nodeWeighted(newAngs, None, randI, weight)
            nodeList.append(newNode)
            weightList.append(weight)
            weightSum += weight
            if DEBUG and ((len(nodeList) % 100) == 0): print len(nodeList)
            # Check if we can connect directly to any of the goals from here
            for goal in toGoals:
                if self.pathIsNotFree(goal, newNode, obstacles):
                    continue
                finalNode = self.nodeWeighted(goal, None, len(nodeList) - 1, 1)
                finished = True
                break
        # Generate path if found
        if finished:
            print "Tree size: ", len(nodeList)
            revPath = [finalNode]
            prev = finalNode.prev
            while prev != None:
                revPath.append(nodeList[prev])
                prev = nodeList[prev].prev
            return [node for node in reversed(revPath)]
        else:
            return None
    def getControlGoals(self, goalXYZ, obstacles, initAtt=10, minGoals=4, timeout=2):
        """ Returns a list of four dimensional arrays that represent goals in the
        control space
        """
        timeS = time.clock()
        foundGoals = []
        angsT = [None]*4
        # Try a fixed number of attempts
        for i in range(initAtt):
            for i in range(4):
                angsT[i] = random.randrange(self.armRanges[i][0], self.armRanges[i][1])
            g = self.configToControl(angsT, goalXYZ)
            if g != None:
                armParts, _ = self.naoBody.getArmShapesAndHandPosition(g, "right")
                if not self.armCollidesWithObstacles(armParts, obstacles):
                    foundGoals.append(g)
        # If we did not reach the minimum during our fixed attempts continue trying
        while len(foundGoals) < minGoals and (time.clock() - timeS) < timeout:
            for i in range(4):
                angsT[i] = random.randrange(self.armRanges[i][0], self.armRanges[i][1])
            g = self.configToControl(angsT, goalXYZ)
            if g != None:
                armParts, _ = self.naoBody.getArmShapesAndHandPosition(g, "right")
                if not self.armCollidesWithObstacles(armParts, obstacles):
                    foundGoals.append(g)
        if len(foundGoals) > 0:
            return foundGoals
        else:
            return None
    def configToControl(self, fromAngles, toXYZ, timeout = .2):
        """ Returns an array of four coordinates in control space that map onto
        the desired XYZ
        @Param fromAngles (1x4 list): A four dimensional vector with the current arm
            angles in "order of limb actuators"
        @Param toXYZ (1x3 list): A three dimensional vector with the euclidian coordinates
            of the goal in the order [X, Y, Z]
        @Param timeout (int): The max amount of time to spend before giving up
        """
        stepRange = [1, 20]
        closeEnough = self.rrtParam["closeEnough"]
        toXYZ = np.array(toXYZ)
        currAng = np.array(fromAngles)
        _, _, handP, _ = self.naoBody.getArmPositions(currAng, 'right')
        dToGoalV = handP - toXYZ
        currDist = np.sqrt(dToGoalV.dot(dToGoalV))
        randAngs = [None]*4
        timeStart = time.clock()
        while (time.clock() - timeStart) < timeout:
            for i in range(4):
                randAngs[i] = random.randrange(stepRange[0], stepRange[1])
                randAngs[i] *= random.randrange(-1,2,2)    # Randomly choose sign
            newAngs = currAng + np.array(randAngs)
            if self.anglesOutOfRange(newAngs):
                continue
            _, _, handP, _ = self.naoBody.getArmPositions(newAngs, 'right')
            dToGoalV = handP - toXYZ
            newDist = np.sqrt(dToGoalV.dot(dToGoalV))
            if newDist < currDist:
                currAng = newAngs
                currDist = newDist
            if currDist <= closeEnough:
                return currAng
        return None
##    def testRandom(self, toXYZ):
##        numValid = 0
##        ranges = [[-119, 119], [-94, 0], [-119, 119], [2, 88]]
##        toXYZ = np.array(toXYZ)
##
##        while numValid < 50:
##            newAng = [random.randint(ranges[x][0], ranges[x][1]) for x in range(4)]
##            _, _, handP, _ = self.naoBody.getArmPositions(newAng, 'right')
##            dToGoalV = handP - toXYZ
##            newDist = np.sqrt(dToGoalV.dot(dToGoalV))
##
##            if newDist < 20:
##                numValid += 1
##                print numValid
    def weightedRandSel(self, nodes, weightTotal):
        """ Select a node randomly based on its weight and returns its index
        @Param nodes (1xN list): A list of nodes with a weight field
        """
        r = random.uniform(0, weightTotal)
        s = 0
        for i in range(len(nodes)):
            s += nodes[i].weight
            if r < s: return i
    def pathIsNotFree(self, goalAngs, fromNode, obstacles):
        """ Returns true if a straight path to goalAng is is free of collisions
        @Param goalAngs (1x4 array): The angles to reach
        """
        stepInt = self.rrtParam["stepInt"]
        currAng = np.array(fromNode.angles)
        distV = goalAngs - currAng
        dist = np.sqrt(distV.dot(distV))
        dirV = distV / np.linalg.norm(distV)
        numSt = int(dist / stepInt)
        for i in range(1,numSt + 1):
            currAng += dirV * stepInt
            armParts, _ = self.naoBody.getArmShapesAndHandPosition(currAng, "right")
            if self.armCollidesWithObstacles(armParts, obstacles):
                return True
        armParts, _ = self.naoBody.getArmShapesAndHandPosition(goalAngs, "right")
        if self.armCollidesWithObstacles(armParts, obstacles):
            return True
        return False
    def armCollidesWithObstacles(self, armParts, obstacles):
        for part in armParts:
            for obst in obstacles:
                if part.collidesWith(obst):
                    return True
        return False
    def anglesOutOfRange(self, angles):
        if (angles[0] < self.shoulPitchRange[0] or
                    angles[0] > self.shoulPitchRange[1] or
                    angles[1] < self.shoulRollRange[0] or
                    angles[1] > self.shoulRollRange[1] or
                    angles[2] < self.elbYawRange[0] or
                    angles[2] > self.elbYawRange[1] or
                    angles[3] < self.elbRollRange[0] or
                    angles[3] > self.elbRollRange[1]):
            return True
        else:
            return False
    def updateNodes(self, nodeList, newNode):
        """ Adds the new node to the list, goes through all of the nodes on the list,
        updates their fields and returns the total weight
        """
        numNodes = len(nodeList)
        totalWeight = 0
        newNodeTotalDist = 0
        for node in nodeList:
            distV = node.angles - newNode.angles
            dist = np.sqrt(distV.dot(distV))
            newNodeTotalDist += dist
            node.avgDToNodes = (node.avgDToNodes * numNodes + dist)/(numNodes + 1)
            weight = node.avgDToNodes**self.nodeWeighted.spreadPow + \
                     (self.nodeWeighted.maxD - node.dToGoal)**self.nodeWeighted.attrPow
            node.weight = weight
            totalWeight += weight
        newNode.avgDToNodes = newNodeTotalDist / (numNodes + 1)
        weight = newNode.avgDToNodes**self.nodeWeighted.spreadPow + \
                 (self.nodeWeighted.maxD - newNode.dToGoal)**self.nodeWeighted.attrPow
        newNode.weight = weight
        totalWeight += weight
        nodeList.append(newNode)
        return totalWeight
    class nodeWeighted:
        def __init__(self, angles, xyz, prev, weight=1):
            """
            Note: angles and xyz are arrays
            """
            self.angles = angles
            self.xyz = xyz
            self.prev = prev
            self.weight = weight
##    class NodeList:
##        numNodes = 0
##        angles = []
##        xyz = []
##        previous = []
##        dToGoal = []
##        weights = []
##
##        def addNode(self):
##            pass
    def alignWrist(self, currAngles, desDir):
        """ Calculates the angle at which the hand best faces the desired direction
        @Param currAngles (1x4 List): The arm angles
        @Param desDir (1x3 list): The direction in which to try to point the arm
        """
        desDir = np.array(desDir)
        desDir = desDir/np.linalg.norm(desDir)
        # Negative extreme
        negAngles = np.array(currAngles + [0])
        negAngles[4] = self.wristRange[0]
        _, _, handP, handDir = self.naoBody.getArmPositions(negAngles, "right")
        handV = handDir - handP
        mag1 = handV.dot(desDir)
        # Search untill you find the best allignment
        angsT = np.array(negAngles)
        while True:
            angsT[4] += 5
            _, _, handP, handDir = self.naoBody.getArmPositions(angsT, "right")
            handV = handDir - handP
            magT = handV.dot(desDir)
            if magT < mag1:
                negAngles = np.array(angsT)
                negAngles[4] -= 5
                break
            mag1 = magT
        # Positive extreme
        posAngles = np.array(currAngles + [0])
        posAngles[4] = self.wristRange[1]
        _, _, handP, handDir = self.naoBody.getArmPositions(posAngles, "right")
        handV = handDir - handP
        mag2 = handV.dot(desDir)
        # Search untill you find the best allignment
        angsT = np.array(posAngles)
        while True:
            angsT[4] -= 5
            _, _, handP, handDir = self.naoBody.getArmPositions(angsT, "right")
            handV = handDir - handP
            magT = handV.dot(desDir)
            if magT < mag2:
                posAngles = np.array(angsT)
                posAngles[4] += 5
                break
            mag2 = magT
        # Best of two
        if mag1 > mag2:
            return negAngles[4]
        else:
            return posAngles[4]
    def smoothedPath(self, path, obstacles):
        """ Returns a list of nodesWeighted that that make up a smoothed out path by
        taking the nodes from the RRT's path, connecting all possible edges, and finding
        the shortest path.
        """
        graph = {}
        numN = len(path)
        for n1 in range(numN):
            graph[n1] = {}
        # Populate the graph with new edges
        for n1 in range(numN):
            for n2 in range(n1, numN):
                if n1 == n2 or self.pathIsNotFree(path[n2].angles, path[n1], obstacles):
                    continue
                distV = path[n1].angles - path[n2].angles
                dist = np.sqrt(distV.dot(distV))
                graph[n1][n2] = dist
                graph[n2][n1] = dist
        # Run Dijkstras
        pathInd = self.shortestPath(graph, [0], numN - 1, distances = {0:0})
        return [path[x] for x in pathInd]
    def shortestPath(self, graph, toVisit, end, distances = {}, visited = [], prev = {}):
        """ Use Dijkstras to find the shortest path in a graph
        """
        if not toVisit:
            return None
        curNode = min(toVisit, key=distances.get)
        toVisit.remove(curNode)
        # If we've reached the end return the path that got us here
        if curNode == end:
            revPath = []
            while curNode != None:
                revPath.append(curNode)
                curNode = prev.get(curNode, None)
            return [node for node in reversed(revPath)]
        visited.append(curNode)
        # Go through the neighbors
        for neighbor in graph[curNode]:
            if neighbor in visited:
                continue
            distStored = distances.get(neighbor, float("inf"))
            distNew = distances[curNode] + graph[curNode][neighbor]
            if neighbor not in toVisit: toVisit.append(neighbor)
            if distNew < distStored:
                distances[neighbor] = distNew
                prev[neighbor] = curNode
        # Recurse to find shortest path
        return self.shortestPath(graph, toVisit, end, distances, visited, prev)
    def eliminateCloseAngles(self, angleList, minDist = 40):
        """ Returns a list of four dimensional arrays of angles where all pairs of
        angles are at least the minDist from each other.
        @Param angleList (Nx1 list): A list of four dimensional arrays of angles
        """
        minDist = 20
        removeI = []
        numAngles = len(angleList)
        for i1 in range(numAngles):
            if i1 in removeI:
                continue
            for i2 in range(i1,numAngles):
                if (i1 == i2) or (i2 in removeI):
                    continue
                distV = angleList[i1] - angleList[i2]
                dist = np.sqrt(distV.dot(distV))
                if dist < minDist:
                    removeI.append(i2)
        removeI = set(removeI)
        return [angleList[i] for i in range(numAngles) if i not in removeI]
    def findClosestAngle(self, goalAngle, angleList):
        """ Returns a four dimensional array of the angle closest to the goal
        goalAngle (1x4 array): The goal
        @Param angleList (Nx1 list): A list of four dimensional arrays of angles
        """
        minD = 10**6
        closest = None
        for ang in angleList:
            distV = ang - goalAngle
            dist = np.linalg.norm(distV)
            if dist < minD:
                closest = np.array(ang)
                minD = dist
        return closest
    def getBodyParts(self, angles = None):
        """ Returns a list of the body parts that make up the nao in the order
        of head (2), chest, abdomen, right arm (2), left arm (2), right leg (2),
        and left leg (2). If angles is None, then it will give a standard "standing"
        list of body parts
        @Param angles (1x26 list): All of the body angles in degrees in the order
        specified by Alderbaran
        http://www.aldebaran-robotics.com/documentation/naoqi/motion/control-joint.html#body-chain-def
        """
        if angles == None:
            return self.naoBody.getBodyParts()
        else:
            return self.naoBody.getBodyParts(angles)
class Body:
    # NEW NAO #
    # Body locations relative to center of sphere on chest (mm)
    headLoc = np.array([0, 0, 120])
    neckLoc = np.array([0, 0, 80])
    rShoulderLoc = np.array([0, -112, 45])
    lShoulderLoc = np.array([0, 112, 45])
    rHipLoc = np.array([0, -50, -145])
    lHipLoc = np.array([0, 50, -145])
    # Body sizes
    headLength = 130
    headSphereRad = 55
    headPillRad = 40
    chestSphereRad = 70
    chestPillLength = 140
    chestPillRad = 40
    upperArmLength = 108
    upperArmRad = 30
    lowerArmLength = 109
    lowerArmRad = 35
    upperLegLength = 100
    upperLegRad = 40
    lowerLegLength = 100
    lowerLegRad = 45
##    # Body locations relative to center of sphere on chest (mm)
##    headLoc = np.array([0, 0, 120])
##    neckLoc = np.array([0, 0, 80])
##    rShoulderLoc = np.array([0, -95, 45])
##    lShoulderLoc = np.array([0, 95, 45])
##    rHipLoc = np.array([0, -50, -145])
##    lHipLoc = np.array([0, 50, -145])
##
##    # Body sizes
##    headLength = 130
##    headSphereRad = 55
##    headPillRad = 40
##    chestSphereRad = 70
##    chestPillLength = 140
##    chestPillRad = 40
##    upperArmLength = 105
##    upperArmRad = 30
##    lowerArmLength = 105
##    lowerArmRad = 35
##    upperLegLength = 100
##    upperLegRad = 40
##    lowerLegLength = 100
##    lowerLegRad = 45
    # OLD NAO #
##    # Body locations relative to center of sphere on chest (mm)
##    headLoc = np.array([0, 0, 110])
##    neckLoc = np.array([0, 0, 65])
##    rShoulderLoc = np.array([0, -95, 45])
##    lShoulderLoc = np.array([0, 95, 45])
##    rHipLoc = np.array([0, -60, -150])
##    lHipLoc = np.array([0, 60, -150])
##
##    # Body sizes
##    headLength = 130
##    headSphereRad = 55
##    headPillRad = 30          #TODO<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
##    chestSphereRad = 70
##    chestPillLength = 135
##    chestPillRad = 40
##    upperArmLength = 90
##    upperArmRad = 30
##    lowerArmLength = 100
##    lowerArmRad = 35
##    upperLegLength = 115
##    upperLegRad = 40
##    lowerLegLength = 100       #TODO<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
##    lowerLegRad = 100          #TODO<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    standInitAngles = [-0.0, 0.0, 81.0, 18.0, -80.0, -58.0, 0.0, 0.0, 0.0, -0.0, -25.0, \
                       40.0, -20.0, 0.0, 0.0, 0.0, -25.0, 40.0, -20.0, 0.0, 81.0, -18.0, \
                       80.0, 59.0, -0.0, 0.0]
    def getBodyParts(self, angles = standInitAngles):
        """ Returns a list of the body parts that make up the nao in the order
        of head (2), chest, abdomen, right arm (2), left arm (2), right leg (2),
        and left leg (2). If angles is None, then it will give a standard "standing"
        list of body parts
        @Param angles (1x26 list): All of the body angles in degrees in the order
        specified by Alderbaran
        http://www.aldebaran-robotics.com/documentation/naoqi/motion/control-joint.html#body-chain-def
        """
        bodyParts = {}
        angles = np.array(angles)
        headShapes = self.getHeadShapes(angles[0:2])
        bodyParts["headParts"] = headShapes
        chestShape = Shape3D(np.array([0, 0, 0]), self.chestSphereRad, "sphere")
        p1 = np.array([0, 0, 0]).astype(float)
        p2 = np.array([0, 0, -self.chestPillLength]).astype(float)
        abdomenShape = Shape3D([p1, p2], self.chestPillRad, "pill")
        bodyParts["torsoParts"] = [chestShape, abdomenShape]
        rightArmShapes, _ = self.getArmShapesAndHandPosition(angles[20:26], "right")
        bodyParts["rightArmParts"] = rightArmShapes
        leftArmShapes, _ = self.getArmShapesAndHandPosition(angles[2:8], "left")
        bodyParts["leftArmParts"] = leftArmShapes
        rightLegShapes, _ = self.getLegShapesAndAnklePos(angles[14:20], "right")
        bodyParts["rightLegParts"] = rightLegShapes
        leftLegShapes, _ = self.getLegShapesAndAnklePos(angles[8:14], "left")
        bodyParts["leftLegParts"] = leftLegShapes
        return bodyParts
    def getHeadShapes(self,angles):
        """
        @Param angles (array): Head angles array [headYaw, headPitch]
        """
        headYaw = np.radians(angles[0])
        headPitch = np.radians(angles[1])
        rightP = self.headLoc + np.array([0, -self.headLength/2, 0])
        leftP = self.headLoc + np.array([0, self.headLength/2, 0])
        rotM = np.array([[ np.cos(headPitch), 0, np.sin(headPitch)],
                         [                    0, 1,              0],
                         [-np.sin(headPitch), 0, np.cos(headPitch)]])
        rightP = np.dot(rotM, rightP)
        leftP = np.dot(rotM, leftP)
        rotM = np.array([[np.cos(headYaw), -np.sin(headYaw), 0],
                         [np.sin(headYaw),  np.cos(headYaw), 0],
                         [              0,                0, 1]])
        rightP = np.dot(rotM, rightP)
        leftP = np.dot(rotM, leftP)
        headSphere = Shape3D(self.headLoc, self.headSphereRad, "sphere")
        headPill = Shape3D([rightP, leftP], self.headPillRad, "pill")
        return [headSphere, headPill]
    def getArmShapesAndHandPosition(self, angles, LR):
        """
        @Param angles (array): Arm angles in degrees [shoulderPitch, shoulderRoll,
            elbowYaw, elbowRoll, wristYaw(optional)]
        @Param LR: Left or Right arm (ie "left", "right")
        """
        shoulderP, elbowP, handP, _ = self.getArmPositions(angles, LR)
        upperArm = Shape3D([shoulderP, elbowP], self.upperArmRad, "pill")
        lowerArm = Shape3D([elbowP, handP], self.lowerArmRad, "pill")
        return [upperArm, lowerArm], handP
    def getArmPositions(self, angles, LR):
        """ Returns the [X, Y, Z] array of the 3 points corresponding to shoulder,
        elbow, and hand
        @Param angles (array): Arm angles in degrees [shoulderPitch, shoulderRoll,
            elbowYaw, elbowRoll, wristYaw(optional)]
        @Param LR: Left or Right arm (ie "left", "right")
        """
        # Convert to radians
        shPit = np.radians(angles[0])
        shRoll = np.radians(angles[1])
        elYaw = np.radians(angles[2])
        elRoll = np.radians(angles[3])
        if len(angles) < 5:
            wrYaw = np.radians(0)
        else:
            wrYaw = np.radians(angles[4])
        handDir = np.array([0, .7, -.7])
        handP = np.array([1, 0, 0]) * self.lowerArmLength
        elbowP = np.array([1, 0, 0]) * self.upperArmLength
        # Hand with respect to elbow
        rotM = np.array([[1,             0,              0],
                         [0, np.cos(wrYaw), -np.sin(wrYaw)],
                         [0, np.sin(wrYaw),  np.cos(wrYaw)]])
        handDir = np.dot(rotM, handDir)
        rotM = np.array([[np.cos(elRoll), -np.sin(elRoll), 0],
                         [np.sin(elRoll),  np.cos(elRoll), 0],
                         [              0,              0, 1]])
        handDir = np.dot(rotM, handDir)
        handP = np.dot(rotM, handP)
        rotM = np.array([[1,             0,              0],
                         [0, np.cos(elYaw), -np.sin(elYaw)],
                         [0, np.sin(elYaw),  np.cos(elYaw)]])
        handDir = np.dot(rotM, handDir)
        handP = np.dot(rotM, handP)
        # Hand and elbow with respect to shoulder
        handP[0] += self.upperArmLength
        rotM = np.array([[np.cos(shRoll), -np.sin(shRoll), 0],
                         [np.sin(shRoll),  np.cos(shRoll), 0],
                         [             0,               0, 1]])
        handDir = np.dot(rotM, handDir)
        handP = np.dot(rotM, handP)
        elbowP = np.dot(rotM, elbowP)
        rotM = np.array([[ np.cos(shPit), 0, np.sin(shPit)],
                         [             0, 1,             0],
                         [-np.sin(shPit), 0, np.cos(shPit)]])
        handDir = np.dot(rotM, handDir)
        handP = np.dot(rotM, handP)
        elbowP = np.dot(rotM, elbowP)
        if LR == "right":
            handDir += self.rShoulderLoc
            handP += self.rShoulderLoc
            elbowP += self.rShoulderLoc
            shoulderP = self.rShoulderLoc
        else:
            handDir += self.lShoulderLoc
            handP += self.lShoulderLoc
            elbowP += self.lShoulderLoc
            shoulderP = self.lShoulderLoc
        return shoulderP, elbowP, handP, handDir
    def getLegShapesAndAnklePos(self, angles, LR):
        """
        @Param angles (array): Leg angles [hipYawPitch, hipRoll, hipPitch, kneePitch,
            anklePitch, ankleRoll]
        @Param LR: Left or Right leg (ie "left", "right")
        """
        hipP, kneeP, ankleP = self.getLegPositions(angles, LR)
        upperLeg = Shape3D([hipP, kneeP], self.upperLegRad, "pill")
        lowerLeg = Shape3D([kneeP, ankleP], self.lowerLegRad, "pill")
        return [upperLeg, lowerLeg], ankleP
    def getLegPositions(self, angles, LR):
        """ Returns the [X, Y, Z] of the 3 points corresponding to hip, knee, and ankle
        @Param angles: Leg angles [hipYawPitch, hipRoll, hipPitch, kneePitch,
                                   anklePitch, ankleRoll]
        @Param LR: Left or Right leg (ie "left", "right")
        @Return hipP, kneeP, ankleP: The coordinates [X, Y, Z]
        """
        # Convert to radians
        hipYawPit = np.radians(angles[0])
        hipRoll = np.radians(angles[1])
        hipPit = np.radians(angles[2])
        kneePit = np.radians(angles[3])
        kneeP = np.array([0, 0, -1]) * self.upperLegLength
        ankleP = np.array([0, 0, -1]) * self.lowerLegLength
        # Ankle with respect to knee
        rotM = np.array([[ np.cos(kneePit), 0, np.sin(kneePit)],
                         [               0, 1,               0],
                         [-np.sin(kneePit), 0, np.cos(kneePit)]])
        ankleP = np.dot(rotM, ankleP)
        ankleP[2] -= self.upperLegLength
        # Angkle and knee with respect to hip
        rotM = np.array([[ np.cos(hipPit), 0, np.sin(hipPit)],
                         [              0, 1,              0],
                         [-np.sin(hipPit), 0, np.cos(hipPit)]])
        ankleP = np.dot(rotM, ankleP)
        kneeP = np.dot(rotM, kneeP)
        rotM = np.array([[1,               0,                0],
                         [0, np.cos(hipRoll), -np.sin(hipRoll)],
                         [0, np.sin(hipRoll),  np.cos(hipRoll)]])
        ankleP = np.dot(rotM, ankleP)
        kneeP = np.dot(rotM, kneeP)
        # Weird 45 degree hip rotation
        if LR == 'right':
            chocRot = -np.radians(45)
            hipP = self.rHipLoc
        elif LR == 'left':
            chocRot = np.radians(45)
            hipP = self.lHipLoc
        else:
            print "Invalid leg choice (LR)"
        chocM = np.array([[1,               0,                0],
                          [0, np.cos(chocRot), -np.sin(chocRot)],
                          [0, np.sin(chocRot),  np.cos(chocRot)]])
        ankleP = np.dot(chocM, ankleP)
        kneeP = np.dot(chocM, kneeP)
        rotM = np.array([[ np.cos(hipYawPit), 0, np.sin(hipYawPit)],
                         [                 0, 1,                 0],
                         [-np.sin(hipYawPit), 0, np.cos(hipYawPit)]])
        ankleP = np.dot(rotM, ankleP)
        kneeP = np.dot(rotM, kneeP)
        chocM = np.array([[1,                0,                 0],
                          [0, np.cos(-chocRot), -np.sin(-chocRot)],
                          [0, np.sin(-chocRot),  np.cos(-chocRot)]])
        ankleP = np.dot(chocM, ankleP)
        kneeP = np.dot(chocM, kneeP)
        ankleP += hipP
        kneeP += hipP
        return hipP, kneeP, ankleP
class Shape3D:
    safetyRadius = 0;
    def __init__(self, points, radius, shapeType):
        """
        @Param points: List of poitns in 3D space that define this shape. One for a
                sphere and two for a pill. (ie [[1,1,2], [3,3,3]])
        @Param radius (double): The radius of this shape
        @Param shapeType: The type of shape that is to be made. Right now it only
                supports "sphere" and "pill" (points a fixed distance from a line)
        """
        radius = float(radius)
        points = np.array(points)
        self.shape = None
        if shapeType == "sphere":
            points = [float(i) for i in points]
            self.shape = euclid.Point3(points[0], points[1], points[2])
        elif shapeType == "pill":
            points = [[float(i) for i in j] for j in points]
            p1 = euclid.Point3(points[0][0], points[0][1], points[0][2])
            p2 = euclid.Point3(points[1][0], points[1][1], points[1][2])
            self.shape = euclid.LineSegment3(p1, p2)
        else:
            print "Warning: Invalid shape"
        self.points = points
        self.radius = radius
        self.shapeType = shapeType
    def collidesWith(self, other):
        """ Returns true if this obstacle collides with other
        """
        dist = self.shape.connect(other.shape).length
        if dist <= (self.radius + other.radius + self.safetyRadius):
            return True
        else:
            return False
"""
TODO LIST:
    nao random movement
    ltlmop grab ball, dump ball
    create a more natural tracking
    use ode
    suggest adding rectangles and other obstacles
    correct leg angle innacuracy
    use close enough parameter on class for goal in controll
    callibrate the length of limbs
DONE:
    webots comes with new naos (need lab email?)
    created wrist rotation algo
    implemented nao's camera
    nao is grabbing the ball
DISCUSS:
    issue with thumb and actual calculations for grabbing the balll
"""
DEBUG = True
DEBUG2 = False
DEBUGT = False
