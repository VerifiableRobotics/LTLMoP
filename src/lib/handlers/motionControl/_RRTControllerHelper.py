#!/usr/bin/env python
"""
    =============================================================
    _RRTControllerHelper.py - Randomly Exploring Rapid Tree Controller
    =============================================================

"""

from numpy import *
from scipy.linalg import norm
from numpy.matlib import zeros
import __is_inside
import time, sys,os
import scipy as Sci
import scipy.linalg
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
import matplotlib.pyplot as plt
from math import sqrt, fabs , pi
import random
import matplotlib.animation as animation
import thread
import threading

distance_from_next = 0       #set to 1 if wants to print distance from next point, set o otherwise
system_print       = 0       #set to 1 to print to terminal
step4              = 0       #set to 1 to print check after step 4
obstacle_print     = 0       #set to 1 to print obstacle check
check_E_print      = 0       #set to 1 to print check edge appending
step56             = 0       #set to 1 to print check for step 5 and 6
finish_print       = 0       #set to 1 to print finish E and V before trimming
move_goal          = 0       #set to 1 when move goal testing

#def __init__(self, ax, motion, maxt=2, dt=0.02):
#    #operate_system (int): Which operating system is used for execution. Ubuntu and Mac is 1, Windows is 2
#    if sys.platform in ['win32', 'cygwin']:
#        self.operate_system = 2
#    else:
#        self.operate_system = 1

def setVelocity(p, V, E, heading,E_prev,radius, last=False):
    """
    This function calculates the velocity for the robot with RRT.
    The inputs are (given in order):
        p        = the current x-y position of the robot
        E        = edges of the tree  (2 x No. of nodes on the tree)
        V        = points of the tree (2 x No. of vertices)
        heading     = index of the previous heading point on the tree
        E_prev   = current index of E
        last = True, if the current region is the last region
             = False, if the current region is NOT the last region
    """

    pose     = mat(p).T
    #dis_cur = distance between current position and the next point
    dis_cur  = vstack((V[1,E[1,E_prev]],V[2,E[1,E_prev]]))- pose
    if norm(dis_cur) < 1.5*radius:         # go to next point
        if not heading == shape(V)[1]-1:
            E_prev = E_prev + 1
            dis_cur  = vstack((V[1,E[1,E_prev]],V[2,E[1,E_prev]]))- pose
            heading = E[1,E_prev]
        else:
            dis_cur  = vstack((V[1,E[1,E_prev]],V[2,E[1,E_prev]]))- vstack((V[1,E[0,E_prev]],V[2,E[0,E_prev]]))
    Vel = zeros([4,1])


    Vel[0:2,0] = dis_cur/norm(dis_cur)*0.5                    #TUNE THE SPEED LATER
    Vel[2,0]   = heading
    Vel[3,0]   = E_prev

    return Vel

def buildTree(p,theta,vert, R, system, regionPoly,nextRegionPoly,q_gBundle,mappedRegions,allRegions,max_angle_allowed, plotting, last=False):

    """
    This function builds the RRT tree
    pose: x,y position of the robot
    theta: current orientation of the robot
    R: radius of the robot
    system: determine step_size and radius for the example
    regionPoly: current region polygon
    nextRegionPoly: next region polygon
    q_gBundle: coordinates of q_goals that the robot can reach
    mappedRegions: region polygons
    allRegions: polygon that includes all the region
    max_angle_allowed: the difference in angle between the two nodes allowed (between 0 to 2*pi)
    """

    max_angle = max_angle_allowed



    if self.operate_system == 2:
        # start using anmination to plot Pioneer
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.scope = _Scope(self.ax,self)
        thread.start_new_thread(self.jplot,())

    ## 1: Nao ; 2: STAGE ;  3: ODE
    if system == 1:    ## Nao
        step_size  = 0.2          	#set the step_size for points be 1/5 of the norm from   ORIGINAL = 0.4
    elif system == 2:
        step_size  = 0.5
    elif system == 3:
        step_size  = 15

    if system == 1:    ## Nao
        timeStep = 5   #time step for calculation of x, y position
    elif system == 2:
        timeStep = 4   #time step for calculation of x, y position
    elif system == 3:
        timeStep = 10  #time step for calculation of x, y position    #10
    #############tune velocity OMEGA, TIME STEP

    #fix velocity
    ## 1: Nao ; 2: STAGE ;  3: ODE
    if system == 1:    ## Nao
        velocity  = 0.05          	#set the step_size for points be 1/5 of the norm from
    elif system == 2:
        velocity  = 0.06
    elif system == 3:
        velocity = 2    # what is used in RRTControllerHelper.setVelocity    #1.5
    #############tune velocity OMEGA, TIME STEP

    BoundPoly  = regionPoly       # Boundary polygon = current region polygon
    radius     = R
    q_init     = mat(p).T
    V          = vstack((0,q_init))
    V_theta    = array([theta])
    original_figure = 1

    #!!! CONTROL SPACE: generate a list of omega for random sampling
    omegaLowerBound = -math.pi/20      # upper bound for the value of omega
    omegaUpperBound = math.pi/20       # lower bound for the value of omega
    omegaStepSize   = 20
    omega_range = linspace(omegaLowerBound,omegaUpperBound,omegaStepSize)
    omega_range_abso = linspace(omegaLowerBound*4,omegaUpperBound*4,omegaStepSize*4)    # range used when stuck > stuck_thres
    edgeX    = []
    edgeY    = []

    # check faces of the current region for goal points

    E = [[],[]]
    Other = [[],[]]
    path     = 0          # if path formed then = 1
    stuck    = 0          # count for changing the range of sampling omega
    stuck_thres = 300     # threshold for changing the range of sampling omega

    if not plt.isinteractive():
        plt.ion()
    plt.hold(True)
    while path == 0:
        #step -1: try connection to q_goal (generate path to goal)
        goalCheck = 0;
        i = 0

        # pushing possible q_goals into the current region (ensure path is covered by the current region polygon)
        q_pass = [[],[],[]]
        q_pass_dist = []
        q_gBundle = mat(q_gBundle)
        while i < q_gBundle.shape[1]:   ###not sure about shape
            q_g = q_gBundle[:,i]+(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])*radius    ##original 2*radius
            trial = 1
            if not BoundPoly.isInside(q_g[0],q_g[1]):
                trial = 2
                q_g = q_gBundle[:,i]-(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,i]-V[1:,(shape(V)[1]-1)])*radius    ##original 2*radius

            #forming polygon for path checking
            cross_goal     = cross(vstack((q_g-vstack((V[1,shape(V)[1]-1],V[2,shape(V)[1]-1])),0)).T,hstack((0,0,1)))
            cross_goal       = cross_goal.T
            move_vector_goal = radius*cross_goal[0:2]/sqrt((cross_goal[0,0]**2 + cross_goal[1,0]**2))
            upperEdgeG   = hstack((vstack((V[1,shape(V)[1]-1],V[2,shape(V)[1]-1])),q_g)) + hstack((move_vector_goal,move_vector_goal))
            lowerEdgeG   = hstack((vstack((V[1,shape(V)[1]-1],V[2,shape(V)[1]-1])),q_g)) - hstack((move_vector_goal,move_vector_goal))
            EdgePolyGoal    = Polygon.Polygon((tuple(array(lowerEdgeG[:,0].T)[0]),tuple(array(upperEdgeG[:,0].T)[0]),tuple(array(upperEdgeG[:,1].T)[0]),tuple(array(lowerEdgeG[:,1].T)[0])))

            dist = norm(q_g - V[1:,shape(V)[1]-1])
            connect_goal = BoundPoly.covers(EdgePolyGoal)   #check coverage of path from new point to goal
            #check connection to goal

            """
            if connect_goal:
                print "connection is true"
                path = 1
                q_pass = hstack((q_pass,vstack((i,q_g))))
                q_pass_dist = hstack((q_pass_dist,dist))
            """

            # compare orientation difference
            thetaPrev = V_theta[shape(V)[1]-1]
            theta_orientation = abs(arctan((q_g[1,0]- V[2,shape(V)[1]-1])/(q_g[0,0]- V[1,shape(V)[1]-1])))
            if thetaPrev < 0:
                if q_g[1,0] > V[2,shape(V)[1]-1]:
                    if q_g[0,0] < V[1,shape(V)[1]-1]: # second quadrant
                        theta_orientation = -2*pi + theta_orientation
                    elif q_g[0,0] > V[1,shape(V)[1]-1]: # first quadrant
                        theta_orientation = -pi -theta_orientation
                elif q_g[1,0] < V[2,shape(V)[1]-1]:
                    if q_g[0,0] < V[1,shape(V)[1]-1]: #third quadrant
                        theta_orientation = -pi + theta_orientation
                    elif q_g[0,0] > V[1,shape(V)[1]-1]: # foruth quadrant
                        theta_orientation =  - theta_orientation
            else:
                if q_g[1,0] > V[2,shape(V)[1]-1]:
                    if q_g[0,0] < V[1,shape(V)[1]-1]: # second quadrant
                        theta_orientation = pi - theta_orientation
                    elif q_g[0,0] > V[1,shape(V)[1]-1]: # first quadrant
                        theta_orientation = theta_orientation
                elif q_g[1,0] < V[2,shape(V)[1]-1]:
                    if q_g[0,0] < V[1,shape(V)[1]-1]: #third quadrant
                        theta_orientation = pi + theta_orientation
                    elif q_g[0,0] > V[1,shape(V)[1]-1]: # foruth quadrant
                        theta_orientation =  2*pi - theta_orientation

            ################################## PRINT PLT #################

            if connect_goal :
                plt.suptitle('Randomly-exploring rapid tree', fontsize=12)
                BoundPolyPoints = asarray(PolyUtils.pointList(BoundPoly))
                plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
                plt.xlabel('x')
                plt.ylabel('y')
                if shape(V)[1] <= 2:
                    plt.plot(( V[1,shape(V)[1]-1],q_g[0,0]),( V[2,shape(V)[1]-1],q_g[1,0]),'b')
                else:
                    plt.plot(( V[1,E[0,shape(E)[1]-1]], V[1,shape(V)[1]-1],q_g[0,0]),( V[2,E[0,shape(E)[1]-1]], V[2,shape(V)[1]-1],q_g[1,0]),'b')
                plt.figure(original_figure).canvas.draw()

            if connect_goal and abs(theta_orientation - thetaPrev) < max_angle:
                #if connect_goal and abs(theta_orientation - thetaPrev) < pi/3:
                print "connection is true.Path = 1"
                path = 1
                q_pass = hstack((q_pass,vstack((i,q_g))))
                q_pass_dist = hstack((q_pass_dist,dist))


            i = i + 1

        # connection to goal has established
        if path == 1:
            if shape(q_pass_dist)[0] == 1:
                cols = 0
            else:
                (cols,) = nonzero(q_pass_dist == min(q_pass_dist))
                cols = asarray(cols)[0]
            q_g = q_pass[1:,cols]   ###Catherine
            q_g = q_g-(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])*3*radius   #org 3
            if not nextRegionPoly.isInside(q_g[0],q_g[1]):
                q_g = q_g+(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])/norm(q_gBundle[:,q_pass[0,cols]]-V[1:,(shape(V)[1]-1)])*6*radius   #org 3

            plt.plot(q_g[0,0],q_g[1,0],'ko')
            plt.figure(original_figure).canvas.draw()

            numOfPoint = floor(norm(V[1:,shape(V)[1]-1]- q_g)/step_size)
            if numOfPoint < 3:
                numOfPoint = 3
            x = linspace( V[1,shape(V)[1]-1], q_g[0,0], numOfPoint )
            y = linspace( V[2,shape(V)[1]-1], q_g[1,0], numOfPoint )
            for i in range(x.shape[0]):
                if i != 0:
                    V = hstack((V,vstack((shape(V)[1],x[i],y[i]))))
                    E = hstack((E,vstack((shape(V)[1]-2,shape(V)[1]-1))))

        # path is not formed, try to append points onto the tree
        if path == 0:
            success     = 0           # whether adding a new point is successful
            hit_count   = 0           # count for regenerating new edge with the same q_rand
            Icurrent    = []          # to keep track of the index of the closest point to q_n

            while success == 0 and hit_count <= 2:
                if stuck > stuck_thres:
                    omega = random.choice(omega_range_abso)
                else:
                    #!!!! CONTROL SPACE STEP 1 - generate random omega
                    omega = random.choice(omega_range)


                #!!!! CONTROL SPACE STEP 2 - pick a random point on the tree
                tree_index = random.choice(array(V[0])[0])
                xPrev     = V[1,tree_index]
                yPrev     = V[2,tree_index]
                thetaPrev = V_theta[tree_index]

                j = 1
                #!!!! CONTROL SPACE STEP 3 - Check path of the robot
                path_robot = PolyShapes.Circle(radius,(xPrev,yPrev))
                while j <= timeStep:
                    xOrg      = xPrev
                    yOrg      = yPrev
                    xPrev     = xPrev + velocity/omega*(sin(omega* 1 + thetaPrev)-sin(thetaPrev))
                    yPrev     = yPrev - velocity/omega*(cos(omega* 1 + thetaPrev)-cos(thetaPrev))
                    thetaPrev = omega* 1 + thetaPrev
                    path_robot = path_robot + PolyShapes.Circle(radius,(xPrev,yPrev))

                    j = j + 1

                path_all = PolyUtils.convexHull(path_robot)
                in_bound = BoundPoly.covers(path_all)

                # plotting
                plotPoly(path_all,'r',1)
                plotMap(original_figure,BoundPoly,allRegions)

                stuck = stuck + 1
                if in_bound:
                    stuck = stuck -5
                    x = []
                    y = []
                    for k in  PolyUtils.pointList(path_all):
                        x = hstack((x,k[0]))
                        y = hstack((y,k[1]))

                    V = hstack((V,vstack((shape(V)[1],xPrev,yPrev))))
                    V_theta = hstack((V_theta,thetaPrev))
                    E = hstack((E,vstack((tree_index ,shape(V)[1]-1))))
                    Other = hstack((Other,vstack((velocity,omega))))
                    ##################### E should add omega and velocity
                    success = 1

    if finish_print == 1:
        print 'Here is the V matrix:', V, 'Here is the E matrix:',E
        print >>sys.__stdout__, 'Here is the V matrix:\n', V, '\nHere is the E matrix:\n',E

    #B: trim to a single path
    single = 0
    while single == 0:
        trim  = 0
        for j in range(shape(V)[1]-3):
            (row,col) = nonzero(E == j+1)
            if len(col) == 1:
                E = delete(E, col[0], 1)
                trim = 1

        if trim == 0:
            single = 1;

    # generate V to be sent to SimGUI to plot
    V_toPass = V[0:,0]
    E_toPass = [[],[]]
    for i in range(shape(E)[1]):
        V_toPass = hstack((V_toPass,vstack((i,V[1,E[1,i-1]],V[2,E[1,i-1]]))))
        E_toPass = hstack((E_toPass,vstack((i-1,i))))

    ####print with matlib

    BoundPolyPoints = asarray(PolyUtils.pointList(BoundPoly))
    plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],'k')
    plt.plot(V[1,:],V[2,:],'b')
    for i in range(shape(E)[1]-1):
        plt.text(V[1,E[0,i]],V[2,E[0,i]], V[0,E[0,i]], fontsize=12)
        plt.text(V[1,E[1,i]],V[2,E[1,i]], V[0,E[1,i]], fontsize=12)
    plt.figure(original_figure).canvas.draw()

    heading  = E[0,0]
    # parse string for RRT printing in GUI (in format: RRT:E[[1,2,3]]:V[[1,2,3]])
    V = array(V)
    V_toPass = array(V_toPass)
    E_toPass = array(E_toPass)
    return V, E, heading,0,V_toPass,E_toPass

def plotMap(number,currentRegion,allRegions):
    """
    Plotting regions and obstacles with matplotlib.pyplot

    number: figure number (see on top)
    """

    if not plt.isinteractive():
        plt.ion()
    plt.hold(True)

    plotPoly(currentRegion,'k')
    plt.figure(number).canvas.draw()

def plotPoly(c,string,w = 1):
    """
    Plot polygons inside the boundary
    c = polygon to be plotted with matlabplot
    string = string that specify color
    w      = width of the line plotting
    """
    for i in range(len(c)):
        toPlot = Polygon.Polygon(c.contour(i))
        if bool(toPlot):
            BoundPolyPoints = asarray(PolyUtils.pointList(toPlot))
            plt.plot(BoundPolyPoints[:,0],BoundPolyPoints[:,1],string,linewidth=w)
            plt.plot([BoundPolyPoints[-1,0],BoundPolyPoints[0,0]],[BoundPolyPoints[-1,1],BoundPolyPoints[0,1]],string,linewidth=w)

def data_gen(self):
    self.ax.cla()
    self.plotPioneer(1)
    self.plotPoly(self.realRobot, 'r')
    self.plotPoly(self.robot, 'b')
    pose = self.pose_handler.getPose()
    self.ax.plot(pose[0],pose[1],'bo')
    self.ax.plot(self.q_g[0],self.q_g[1],'ro')
    self.plotPoly(self.overlap,'g')
    self.plotPoly(self.m_line,'b')
    yield(pose[0],pose[1])
    self.ax.plot(self.prev_follow[0],self.prev_follow[1],'ko')


def jplot(self):
    ani = animation.FuncAnimation(self.fig, self.scope.update, self.data_gen)
    plt.show()

class _Scope:
    def __init__(self, ax, motion, maxt=2, dt=0.02):
        self.i = 0
        self.ax = ax
        self.line, = self.ax.plot(1)
        self.ax.set_ylim(0, 1)
        self.motion = motion

    def update(self,data):
        (data1) = self.motion.data_gen()
        a = data1.next()
        self.line.set_data(a)
        self.ax.relim()
        self.ax.autoscale()
        return self.line,



