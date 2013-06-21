#!/usr/bin/env python
""" 
    =========================================================
    heatControllerHelper.py - Convex Polygon Point Controller
    =========================================================
    
    A Python translation and trimming of David C. Conner's MATLAB implementation.
"""

from math import atan2, acos
from numpy import *
from scipy.linalg import norm, svd, solve, eig
from scipy.optimize import fmin

eps = 2**(-52) # A very small number, for noise tolerance in calculations

def getController(Vertex, exitface, last):
    """
    This function returns an initialized controller that will calculate
    the potential field based control law that will take a robot from an
    initial position in a region to the exit face that leads to the next
    region or to the map reference point (in the last region)
    The input is:
       Vertex = vertices of the region, specified in clockwise order (2 x No. of vertices)
       exitface - index of exitface. (i.e. if the exit face is the face
       defined by vertices 3 and 4 then exitface=3). if last = 1, any exitface
       will do...
       init - initial point in this region
       last - True = this is the last region, False = it is NOT the last region 

      This function calls functions created by David Conner (dcconner@cmu.edu)
    """

    #******************************************************
    #     When entering a new region :
    #******************************************************

    # Given convex polygon and choice of exit face
    # pre-calculate the following 

    # Convert defined polygons to the data we need
    # P0,N0 = center point and outward normal for exit face
    # Pin,Nin = center point and inward normals
    # Vtx = clockwise list of vertices beginning and ending at exit face

    # input: Vertex clockwise list of vertices (2xM) and choice of exit face

    if last:
        [P0, N0, Pin, Nin, Vtx] = get_face_definitions(Vertex, -1)
    else:
        [P0, N0, Pin, Nin, Vtx] = get_face_definitions(Vertex, exitface)


    # Get the map reference point, always pass 4 to get_map_point,
    #  which allows some other choices as well. 
    qx = get_map_point(P0, N0, Pin, Nin, 4, Vtx)     

    # Fix for long skinny regions... 
    if not last:
        exitFaceLength = norm(Vtx[:,0]-Vtx[:,-1])
        P0_qxDistance = norm(P0-qx)
        aspect = exitFaceLength/P0_qxDistance # ratio of exit face length to distance from map origin
        if aspect < 1: # small exit face \ large distance from qx - move map origin
            qx = (qx*aspect/2 + P0*(1-aspect/2))
            #print qx

    # product of distances to each face
    # this is the max value given map origin chosen above
    Bmax = distance_product(qx, P0, N0, Pin, Nin)

    # Disp find the limiting angles
    # by moving polygon to qx  and aligning exit face center
    # on the negative x-axis
    [ae1, ae2] = get_angle_limits(qx, P0, Vertex, exitface)

    # characteristic radius for smoothing near vertices
    Brad = 1e-3*(Bmax**(1/Vtx.shape[1]))

    hessian = True # calculate the hessian terms

    if last:
        # Return a controller that heads to the "center" of the region
        # NOTE: This will never converge perfectly due to discrete-time simulation;
        # you should implement a distance threshold afterwhich the robot will stop

        # define the goal point within the polygon
        qf = qx # map reference point
        
        # alpha  - scaling factor for convergence   |X| = |q-qf|/(|q-qf| + alpha)
        alpha = 0.25 # alpha > 0
        
        def controller(pos):    
            return Xgoal_penn(pos, P0, N0, Pin, Nin, qx, ae1, ae2, Bmax, hessian, Vtx, Brad, qf, alpha)
        
    else:  # not last region
        # Return a controller that heads out the center of the exitface, perpendicular to the face
        def controller(pos):
            return Xoq_penn(pos, P0, N0, Pin, Nin, qx, ae1, ae2, Bmax, hessian, Vtx, Brad)

    return controller

    #*************************************************************************


def get_face_definitions(Vertex, exitface):
    """
    Extract the face information
    P0, N0 = center point and outward normal
    Pin, Nin = center point and inward normals
    Vtx = clockwise list of vertices beginning an ending at exit face

    Vertex = (2 x M) clock wise list of vertices of polygon
           M = number of vertices in polygon
    vertex = [x                                ]
             [y                                ]

    exitface = desired exit face for this polygon

     copyright 2003-2005 David C. Conner (dcconner@cmu.edu), 
    """
    n = Vertex.shape[1]
    if not all(Vertex[:,0] == Vertex[:,-1]):
        verts = concatenate((Vertex, Vertex[:,0]), 1) # Repeat the first vertex
    else:
        verts = Vertex
        n = n-1 # dcc 11/03/03

    ndx = arange(0,n)
    P = (verts[:,ndx] + verts[:,ndx+1])/2

    # Inward pointing normal
    N = -1 * concatenate((-(verts[1,ndx+1] - verts[1,ndx]),\
        (verts[0,ndx+1] - verts[0,ndx])))
    N2 = multiply(N,N)
    Norm = sqrt(N2.sum(axis=0))

    for i in xrange(n):
        N[:,i] = N[:,i]/Norm[0,i]

    Vtx = Vertex.copy()

    if exitface < 0:
        P0 = mat([])
        N0 = mat([])
        Pin = P.copy()
        Nin = N.copy()
        # Vtx is the same
    else:
        P0 =  P[:, exitface].copy()
        N0 = -N[:, exitface].copy()
        # Shift vertices based on exitface location
        if exitface == 0:
            Pin = P[:,1:n].copy()
            Nin = N[:,1:n].copy()
            Vtx = concatenate((Vtx[:,1:n], Vtx[:,0]), 1)
        elif exitface == n-1:
            Pin = P[:,0:(n-1)]
            Nin = N[:,0:(n-1)]
            #Vtx = same as input
        else:
            Pin = concatenate((P[:,(exitface+1):n], P[:,0:exitface]), 1)   # dcc 11/04/03
            Nin = concatenate((N[:,(exitface+1):n], N[:,0:exitface]), 1)   # dcc 11/04/03 
            Vtx = concatenate((Vtx[:,(exitface+1):n], Vtx[:,0:exitface+1]), 1) # dcc 11/04/03

    return [P0, N0, Pin, Nin, Vtx]

def get_map_point(P0, N0, Pin, Nin, choice, Vtx):
    """
    Find origin for mapping operation
      qx = get_map_point(P0,N0,Pin,Nin,choice,vertices)
     choice :
       2 = vertex average
       4+= max distance (beta function) location
    """

    if choice == 2:
        # vertex average
        if not P0.size == 0:
            qx = (Pin.sum(axis=1)+P0)/(Pin.shape[1]+1) # Centroid by face average
        else:
            qx = Pin.sum(axis=1)/Pin.shape[1] # Centroid by face average
          
    elif choice >= 4:
        # maximum of distance function beta (use vertex average ic)
        qx = get_map_point(P0,N0,Pin,Nin,2,Vtx) # Centroid by vertex
        qx = fmin(beta_minimizer,(qx[0,0],qx[1,0]),(P0,N0,Pin,Nin,Vtx),disp=0)
        qx = mat(qx).T

    return qx
   

def is_inside(q, P0, N0, Pin, Nin, Vtx):
    """
    Allow test in arbitrary bounded region without worrying about
    which is the exit face.
    """

    dprev = (Nin[:,-1].T*(q-Pin[:,-1]))[0,0]
    if not P0.size == 0:
        d = (-N0.T*(q-P0))[0,0]
        if d < -1e-6:  # Boundary of exit is inside the next cell
            return False
        elif d < -1e-12 and dprev < 0:
            # Two votes for out
            return False
        elif d < -1e-12:
            d1 = norm(q-Vtx[:,0])
            dn = norm(q-Vtx[:,-1])
         
            if d1 > 1e-12 and dn > 1e-12:
                return False
            else:
                # Close to a vertex
                if d1 < 1e-12:
                    N = -N0 + Nin[:,0]
                    N = N/norm(N)
                    dv = (N.T*(q-Vtx[:,0]))[0,0]
                    if dv < 0:
                        return False
                else:
                    # Must be 2nd vertex
                    N = -N0 + Nin[:,-1]
                    N = N/norm(N)
                    dv = (N.T*(q-Vtx[:,-1]))[0,0]
                    if (dv < 0):
                        return False
        dprev = d
       
    for i in xrange(Pin.shape[1]):
        d = Nin[:,i].T * (q-Pin[:,i])
      
        # Allow us to fudge the boundary near vertex
        # because of numerical issues
        # If it's close to boundary, require 2 votes for outside
        # in case of vertex

        if d < -1e-8:
            # Far enough out - call it out
            return False
        elif d < -1e-12 and dprev < 0:
            # Two votes for out
            return False
        elif d < -1e-12:
            # check if far enough from boundary
            d1 = norm(q-Vtx[:,i])
            if P0.size == 0:
                dn = norm(q-Vtx[:,0]) 
            else:
                dn = norm(q-Vtx[:,i+1])
         
            if d1 > 1e-12 and dn > 1e-12:
                return False
            else:
                # Close to a vertex
                if d1 < 1e-12:
                    if i == 1:
                        N = -N0
                    else:
                        N = Nin[:,i-1]
                else:
                    if i == Nin.shape[1]:
                        N = -N0
                    else:
                        N = Nin[:,i+1]
            
                N = N + Nin[:,i]
                N = N/norm(N)
                dv = (N.T*(q-Vtx[:,i]))[0,0]
                if dv < -1e-12:
                    return False
        dprev = d

    return True
   

def distance_product(q, P0, N0, Pin, Nin):
    """
    Calculates the product of distances to each face
    given point normal definition of faces and the location of an
    interior point.
    """

    if not P0.size == 0:
        B = ((P0-q).T*N0)[0,0] # Convert to inward pointing normal for exit face
    else:
        # In case polygon defined without an exit face
        B = 1.0

    # Multiply by distance to face i
    for i in xrange(Pin.shape[1]):
        Bi = ((q-Pin[:,i]).T*Nin[:,i])[0,0]
        B = B*Bi

    return B


def get_angle_limits(qx, P0, Vertex, exitface, Brad=0):
    """
    Get angles of the exit vertices assuming the exit face is centered
    on the negative x-axis

     copyright 2003-2005 David C. Conner (dcconner@cmu.edu), 
    """

    # Safety catch - this shouldn't be called for convergent cells
    if P0.size == 0:
        ae1 = -pi
        ae2 =  pi
        return [ae1, ae2]

    if not all(Vertex[:,0] == Vertex[:,-1]):
        V = concatenate((Vertex, Vertex[:,0]), 1) # Repeat the first vertex
    else:
        V = Vertex

    qe = P0-qx
    qe = qe/norm(qe)
    a0 = atan2(qe[1,0], qe[0,0])

    # Vertices numbered clockwise
    # angles measured ccw
    dV = V[:,exitface+1]-V[:,exitface]

    if Brad > norm(dV):
        Brad = norm(dV)

    dV = dV/norm(dV)

    # Perturb the limit toward the middle of exit face
    qv = V[:,exitface]-qx #+(Brad/5)*dV

    ae1 = -pi + acos(dot(qv.T,qe)[0,0]/norm(qv)) # angle relative to the exit normal

    # Perturb the limit toward the middle of exit face
    qv = V[:,exitface+1]-qx #-(Brad/5)*dV

    ae2 = pi - acos(dot(qv.T,qe)[0,0]/norm(qv)) # angle relative to the exit normal

    return [ae1, ae2]


def cond(matrix):
    """
    Returns the condition number of a matrix.
    """
    s = svd(matrix, compute_uv=0)
    return s[0]/s[-1]

def Xgoal_penn(q,P0,N0,Pin,Nin,qx,ae1,ae2,Bmax,hessian,Vtx,Brad,qf,alpha):
    """
    This function maps the solution to gradient in the disk world to
    polygon using the Rizzi style convergence controller after mapping to disk

    Returns:
       X - Desired velocity vector
       DqX - Jacobian of vector field
       F - potential value
       inside - q is inside polygon
       J - mapping jacobian
    Requires:
       q        = 2x1 coordinate vector
      (P0,N0)   = definition of exit face
      (Pin,Nin) = matrix defining the other boundary faces
       qx       = coordinate of map center
       ae1,ae2   = angle limits of for unit circle (calculated once for each polygon)
       hessian     = False=skip hessian calc, True=do hessian calc
       Bmax        = maximum Beta value used by some mappings
       qf       = goal (final) point
       alpha      = convergence factor (alpha > 0)

     copyright 2003-2005 David C. Conner (dcconner@cmu.edu), 
       revised 11/08/05 dcc  - simplified based on dynamic_goal
    """

    # Carry on with the calculation even if outside so as
    # not to screw up Matlab integration. TODO: ???
    #
    # Points outside the boundary map to the boundary with appropriate vector field
    # Count on the event simulation to handle the errors

    inside = is_inside(q,P0,N0,Pin,Nin,Vtx)

    # Initialize
    vtx_check = 0
    discontinuity = 0
    H=mat([])
    cn=NaN
    Hn=NaN

    # Distance from current location to the goal
    qv=q-qf
    D = norm(qv)

    # We'll do control if not at the goal, but event simulation will stop 
    # us within the ball of radius R

    # Map to the disk world return coordinates (w,z) in disk world and
    # the coordinate in the recentered and rotated polygon
    [qc,qs,Rot,J,Hp,B,DxB,DyB,DxxB,DyyB,DxyB,vtx_check,limit] = map2diskScale(q,P0,N0,Pin,Nin,qx,-pi,pi,Bmax,hessian,Vtx,Brad)
    [qcf,qs,Rot,J,Hp,B,DxB,DyB,DxxB,DyyB,DxyB,vtx_check,limit] = map2diskScale(qf,P0,N0,Pin,Nin,qx,-pi,pi,Bmax,hessian,Vtx,Brad)
      
    # Find gradient in the disk world
    [Fc,Gc,Hc,CNc] = disk_goal(qc,qcf,hessian)
    discontinuity = 0

      
    # Pull back
    F   = Fc
    DxW = J[0,0]
    DyW = J[0,1]
    DxZ = J[1,0]
    DyZ = J[1,1]
      
    DwF = Gc[0,0]
    DzF = Gc[0,1]
      
    # Vector norms
    NG2=(DwF*DxW + DxZ*DzF)**2 + (DwF*DyW + DyZ*DzF)**2
    NG = sqrt(NG2)
    NG32 = NG**3

    if NG > 0:
        DxF=(DwF*DxW + DxZ*DzF)/NG
        DyF=(DwF*DyW + DyZ*DzF)/NG
    else:
        DxF = 0
        DyF = 0
        NG  = 1 # Protect against divide by 0
        NG32= 1

    # Negative normalized gradient
    G = -mat([DxF,DyF])

    if inside:
        DxxW = Hp[0,0]
        DyyW = Hp[0,1]
        DxyW = Hp[0,2]
        DxxZ = Hp[1,0]
        DyyZ = Hp[1,1]
        DxyZ = Hp[1,2]
          
        DwwF = Hc[0,0]
        DzzF = Hc[1,1]
        DwzF = Hc[0,1]
          
        DxxF= ((DwF*DyW + DyZ*DzF)*(DwF**2*(-(DxW*DxyW) + DxxW*DyW) + \
            DzF*(DwwF*DxW*(-(DxZ*DyW) + DxW*DyZ) + \
            DwzF*DxZ*(-(DxZ*DyW) + DxW*DyZ) - DxyZ*DxZ*DzF + DxxZ*DyZ*DzF) + \
            DwF*(DwzF*DxW*(DxZ*DyW - DxW*DyZ) - DxyW*DxZ*DzF + DxxZ*DyW*DzF + \
            DxxW*DyZ*DzF + DxZ**2*DyW*DzzF - \
            DxW*(DxyZ*DzF + DxZ*DyZ*DzzF))))/NG32
        DxyF= ((DwF*DyW + DyZ*DzF)*(DwF**2*(DxyW*DyW - DxW*DyyW) + \
            DzF*(DwwF*DyW*(-(DxZ*DyW) + DxW*DyZ) + \
            DwzF*DyZ*(-(DxZ*DyW) + DxW*DyZ) - DxZ*DyyZ*DzF + DxyZ*DyZ*DzF) + \
            DwF*(DwzF*DyW*(DxZ*DyW - DxW*DyZ) + DxyZ*DyW*DzF - DxZ*DyyW*DzF - \
            DxW*DyyZ*DzF + DxyW*DyZ*DzF + DxZ*DyW*DyZ*DzzF - \
            DxW*DyZ**2*DzzF)))/NG32
        DyxF= ((DwF*DxW + DxZ*DzF)*(DwF**2*(DxW*DxyW - DxxW*DyW) + \
            DzF*(DwzF*DxZ**2*DyW - DwzF*DxW*DxZ*DyZ + \
            DwwF*DxW*(DxZ*DyW - DxW*DyZ) + DxyZ*DxZ*DzF - DxxZ*DyZ*DzF) + \
            DwF*(DwzF*DxW*(-(DxZ*DyW) + DxW*DyZ) + DxW*DxyZ*DzF + DxyW*DxZ*DzF - \
            DxxZ*DyW*DzF - DxxW*DyZ*DzF - DxZ**2*DyW*DzzF + \
            DxW*DxZ*DyZ*DzzF)))/NG32
        DyyF= ((DwF*DxW + DxZ*DzF)*(DwF**2*(-(DxyW*DyW) + DxW*DyyW) + \
            DzF*(DwzF*DxZ*DyW*DyZ - DwzF*DxW*DyZ**2 + \
            DwwF*DyW*(DxZ*DyW - DxW*DyZ) + DxZ*DyyZ*DzF - DxyZ*DyZ*DzF) + \
            DwF*(DwzF*DyW*(-(DxZ*DyW) + DxW*DyZ) - DxyZ*DyW*DzF + DxZ*DyyW*DzF + \
            DxW*DyyZ*DzF - DxyW*DyZ*DzF - DxZ*DyW*DyZ*DzzF + \
            DxW*DyZ**2*DzzF)))/NG32

        H = -mat([[DxxF,DxyF],[DyxF,DyyF]]) # change in vector field
                                            # of negative gradient so use
                                            # negative sign with matrix 
        cn = cond(H)
    else:
        # Hessian but not inside polygon C**2 approximation
        H = mat(zeros((2,2)))
        cn = NaN

    # No need to rotate back to original frame for the goal controller

    # Scaling factor a' la Rizzi '98
    D2=(qv.T*qv)[0,0] # given qv=q-qf from above
    S = D2/(D2+alpha)

    # Final version
    X=S*G.T

    if hessian:
        # Recalculate based on convergence scaling
        DqS = 2*alpha*qv.T/((D2+alpha)**2)
        DqX = S*H + (G.T*DqS)[0,0]
    else:
        DqX = H

    return [X,DqX,F,inside,J]

def disk_goal(q,qf,hessian=False):
    """
    This function calculates the solution to Laplace's equation for
    steady state temperature distribution given boundary conditions
    on a unit disk.  The boundary is = 1 for ae1 to ae2, and
    0 elsewhere

    Returns:
       F = Function value
       G = Gradient of F = DqF (cartesian gradient)
       H = Hessian  of F = Dq(DqF)
      cn = condition number of Hessian

    Requires:
       q = 2x1 coordinate vector
       qf = 2x1 target coordinate vector
       hessian= False=skip hessian calc, True=do hessian calc
    """

    r = norm(q)          # Get the radius of the coordinate
      
    inside = is_inside_circ(q) # Is this point in the unit disk (including boundary)
      
    if not inside and (r-1)<100*eps:
        # Assume numerical noise in conversion from boundary
        q=q/(norm(q)+50*eps)            
        r = 1.0
        inside = is_inside_circ(q)

    qv = q-qf
    nv = norm(qv)

    w  = q[0,0] # Extract for cosmetic reasons 
    z  = q[1,0]
    wf = qf[0,0]
    zf = qf[1,0]
     
    F = (w**2 - 2*w*wf + wf**2 + (z - zf)**2)/(2.*(-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2)))
              
    DwF =  -(((-1 + wf**2 + zf**2)*(-(w**2*wf) + wf*(-1 + z**2) + w*(1 + wf**2 - 2*z*zf + zf**2)))/ \
            (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))**2)
              
    DzF =  -(((-1 + wf**2 + zf**2)*((-1 + w**2)*zf - z**2*zf + z*(1 - 2*w*wf + wf**2 + zf**2)))/ \
            (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))**2)

    G = mat([DwF,DzF])

    if not hessian:
        # Skip the hessian calculations if we're only doing kinematic solution
        H = []
        cn = 1
    else:
        # Do the hessian calculations         
        DwwF =  ((-1 + wf**2 + zf**2)*(-((1 - 2*w*wf + wf**2 - 2*z*zf + zf**2)* \
                (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))) + \
                2*(-2*wf + 2*w*(wf**2 + zf**2))*(-(w**2*wf) + wf*(-1 + z**2) + w*(1 + wf**2 - 2*z*zf + zf**2))))/ \
                (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))**3

        DzzF =  ((-1 + wf**2 + zf**2)*(-((1 - 2*w*wf + wf**2 - 2*z*zf + zf**2)* \
                (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))) + \
                2*(2*wf**2*z + 2*zf*(-1 + z*zf))*((-1 + w**2)*zf - z**2*zf + z*(1 - 2*w*wf + wf**2 + zf**2))))/ \
                (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))**3

        DwzF =  ((-1 + wf**2 + zf**2)*(-((2*wf*z - 2*w*zf)* \
                (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))) + \
                2*(2*wf**2*z + 2*zf*(-1 + z*zf))*(-(w**2*wf) + wf*(-1 + z**2) + w*(1 + wf**2 - 2*z*zf + zf**2))))/ \
                (-2*w*wf + wf**2*z**2 + (-1 + z*zf)**2 + w**2*(wf**2 + zf**2))**3

     
        H = mat([[DwwF, DwzF],[DwzF, DzzF]])
        
        cn = cond(H)

    return [F,G,H,cn]

def is_inside_circ(q):
    rad = dot(q.T,q)[0,0]
    return (rad <= 1.0)

def map2diskScale(q,P0,N0,Pin,Nin,qx,ae1,ae2,Bmax,hessian=False,Vtx=None,Brad=None):
    """
    Converted to use an arbitrary location
    Find the x-y location relative to center point oriented
    according to the exit face
    We assume vertices are numbered clockwise,and angles are ccw
    for a given exit face, the prior vertex is the most positive angle
    qc = location in disk qc=[w;z]=\phi([x;y])
    R  = Rotation matrix to align exit face along negative x-axis
    J  = D_q\phi Jacobian of the mapping J=[DxW DyW; DxZ DyZ]
    Hp = "Hessian" terms of mapping [DxxW DyyW DxyW; DxxZ DyyZ DxyZ];
    B,DxB,...,DxyB = distance function and partials

    written by David Conner
     revised 04/22/03
    """

    if Brad is None:
        Brad = mat([])

    qv=q-qx

    # Get rotation to place exit face bisector on negative x-axis
    if not P0.size == 0:
        qe = P0-qx
        qe = qe/norm(qe)
        a0 = atan2(qe[1,0],qe[0,0])
    else:
        a0 = pi
        qe = mat([[1],[0]])

    # rotation of polygon relative to x-axis
    R = mat([[cos(pi-a0),-sin(pi-a0)],[sin(pi-a0),cos(pi-a0)]])
    if P0.size == 0:
        Ns0=mat([])
        Ps0=mat([])
    else:
        Ns0=R*N0
        Ps0=R*(P0-qx)
            
    Ns = R*Nin
    Ps = R*(Pin-qx*mat(ones((1,Pin.shape[1]))))

    if Vtx is not None:
        Vs = R*(Vtx-qx*ones((1,Vtx.shape[1])))
    else:
        Vs = []

    # Convert to standard polygon configuration
    qx = mat([[0],[0]])      
    nq = norm(qv)

    if nq < 100*eps:
       limit = 1
       qs = qx # set to exactly 0
       if nq > 0:
            qs = 100*eps*R*(qv/nq)
    else:
        qs = R*qv
        limit = 0

    # Check for point inside
    inside = is_inside(q,P0,N0,Pin,Nin,Vtx)

    # Check to see if we are near a vertex 
    # if so, use the mapping to fillet curve
    if inside:
        [vtx_check, qs] = check_vertex(qs,Ps0,Ns0,Ps,Ns,Vs,Brad)
    else:
        vtx_check = 0

    nq = norm(qs)

    # Get the beta function used in the mapping   
    N = 1 + Pin.shape[1] 
    Bfact = Bmax**(1/N)
    [B,DxB,DyB,DxxB,DyyB,DxyB] = beta_function(qs,qx,Ps0,Ns0,Ps,Ns,Vs,hessian,Bfact)
    Bfact = Bfact/Bmax

    if not hessian:
        DxxB=0
        DyyB=0
        DxyB=0
        Hp=mat([])

    B = B*Bfact
    DxB=DxB*Bfact
    DyB=DyB*Bfact
    DxxB=DxxB*Bfact
    DyyB=DyyB*Bfact
    DxyB=DxyB*Bfact

    if nq < 100*eps:
        fact = norm(qv)/(100*eps)
        fact = 1 + fact*((nq + B) - 1)
        qc = R*qv/fact
    else:
        qc = qs/(nq + B)


    # Make sure we didn't futz it up with mapping near boundary
    # We started inside, so make sure we stay inside
    if norm(qc) > 1.0 and inside:
        # Numerical noise in conversion from boundary
        qc = qc/norm(qc)

    # Now calculate the Jacobian
    if nq < 2*eps:
        limit = 1
        J = mat(zeros((2,2)))
        J[1,1]=1/(B)
        J[0,0]=J[1,1]
        if hessian:
            qss = R*qv
            x = qss[0,0]
            y = qss[1,0]
            A = atan2(y,x)
            DxxW = (-9*cos(A) + cos(3*A) - 8*DxB)/(4*B**2)
            DyyW = -cos(A)**3/B**2
            DxyW = (-3*sin(A) + sin(3*A) - 4*DyB)/(4*B**2)
            DxxZ = -sin(A)**3/B**2
            DyyZ = -(9*sin(A) + sin(3*A)+8*DyB)/(4*B**2)
            DxyZ = -(3*cos(A) + cos(3*A) + 4*DxB)/(4*B**2)
            Hp=mat([[DxxW,DyyW,DxyW],[DxxZ,DyyZ,DxyZ]])
    else:
        x = qs[0,0]
        y = qs[1,0]
        DxW = (B - DxB*x + y**2/nq)/(B + nq)**2
        DyW = -((x*(DyB + y/nq))/(B + nq)**2)
        DxZ = -(((DxB + x/nq)*y)/(B + nq)**2)
        DyZ = (B + x**2/nq - DyB*y)/(B + nq)**2

        J=mat([[DxW,DyW],[DxZ,DyZ]])
        if hessian:
            DxxW = (-2*(DxB + (x/nq))*(B - DxB*x + (y**2/nq)) + (B + nq)*(-(DxxB*x) - ((x*y**2)/nq**3)))/((B + nq)**3)
            DyyW = x*(2*(DyB + y/nq)**2 - ((B + nq)*(x**2 + DyyB*nq**3))/nq**3)/(B + nq)**3
            DxyW = (-2*(DyB + y/nq)*(B - DxB*x + y**2/nq) + (B + nq)*(DyB - DxyB*x + (2*x**2*y + y**3)/nq**3))/(B + nq)**3
            DxxZ = (y*(2*(DxB + x/nq)**2 - ((B + nq)*(y**2 + DxxB*nq**3))/nq**3))/(B + nq)**3
            DyyZ = (-2*(B + x**2/nq - DyB*y)*(DyB + y/nq) + (B + nq)*(-(DyyB*y) - (x**2*y)/nq**3))/(B + nq)**3
            DxyZ = -(((B + nq)*(DxB + x/nq) - 2*(DxB + x/nq)*y*(DyB + y/nq) + (B + nq)*y*(DxyB - (x*y)/nq**3))/(B + nq)**3)
            Hp=mat([[DxxW,DyyW,DxyW],[DxxZ,DyyZ,DxyZ]])

    return [qc,qs,R,J,Hp,B,DxB,DyB,DxxB,DyyB,DxyB,vtx_check,limit]
   

def check_vertex(qs,Ps0,Ns0,Ps,Ns,Vs,Brad):
    vtx_check = -1
    qsr = qs

    if Vs.size == 0:
        return [-1, qsr]

    for i in xrange(Vs.shape[1]):   
        if norm(qs-Vs[:,i]) < Brad:
            vtx_check = i
            break

    if vtx_check == -1:
        return [-1, qsr]


    if vtx_check == 0:
        n1 = Ns[:,vtx_check]
        if Ns0.size == 0:
            n2 = Ns[:,Vs.shape[1]-1]
        else:      
            n2 = -Ns0 # reverse normal for exit face
    elif vtx_check == Vs.shape[1]-1:
        if Ns0.size == 0:
            n1 =  Ns[:,0]
            n2 =  Ns[:,vtx_check]
        else:
            n1 = -Ns0 # reverse normal for exit face
            n2 =  Ns[:,vtx_check-1]
    else:
        n1 = Ns[:,vtx_check]
        n2 = Ns[:,vtx_check-1]

    V=Vs[:,vtx_check]
    S1=mat([[-n1[1,0]],[n1[0,0]]])
    S2=mat([[n2[1,0]],[-n2[0,0]]])

    # Vertex frame
    Y=n1+n2
    Y=Y/norm(Y)

    X=mat([[Y[1,0]],[-Y[0,0]]])

    T=concatenate((X,Y),1) # transform matrix

    # Get point in vertex frame
    qv=(qs-V)
    x=(X.T*qv)[0,0]
    y=(Y.T*qv)[0,0]


    del2 = Brad*(S2.T*X)[0,0]
    del1 = -del2

    # y = M P = A
    A=mat(  [[Brad*dot(S2.T,Y)[0,0]],               # y(x2)  
             [dot(S2.T,Y)[0,0]/dot(S2.T,X)[0,0]],          # y'(x2)
             [0],                            # y''(x2)
             [Brad*dot(S1.T,Y)[0,0]]])              # y(x1)  to force  symmetry


    # parse range to get better matrix condition number
    if abs(del2) > 0.01:
        # y(x)_= a x**4 + b x**3  + c x**2  + d x + e 
        #
        # y'(0)= d = 0
        # y(x)_= a x**4 + b x**3  + c x**2  + e 
        M=mat([ [ del2**4,    del2**3,    del2**2,    1],    # y(x2)
                [ 4*del2**3,  3*del2**2,  2*del2,     0],    # y'(x2)
                [ 12*del2**2, 6*del2,     2,          0],    # y''(x2)
                [ del1**4,    del1**3,    del1**2,    1]])   # y(x1)  force symmetry
         

        P = solve(M,A)

        # Is there a better way to do this?
        P = concatenate((P[0:3,0].T, mat([0]), mat([P[3,0]])), 1)
    elif abs(del2) > 1e-4:
        # y(x)_= 1000000 a x**4 + 100000 b x**3  + 10000 c x**2  + d x + e 
        #
        # y'(0)= d = 0
        # y(x)_= 1000000 a x**4 + 100000 b x**3  + 10000 c x**2  + e 

        M=mat([ [ 100000*del2**4,   1000*del2**3,   10*del2**2, 1],    # y(x2)
                [ 400000*del2**3,   3000*del2**2,   20*del2,    0],    # y'(x2)
                [ 1200000*del2**2,  6000*del2,      20,         0],    # y''(x2)
                [ 100000*del1**4,   1000*del1**3,   10*del1**2, 1]])   # y(x1)  force symmetry
        P = solve(M,A)

        # Is there a better way to do this?
        P = concatenate((multiply(mat([100000, 1000, 10]),P[0:3,0].T), mat([0]), mat([P[3,0]])), 1)
    else:
        # y(x)_= 1000000 a x**4 + 100000 b x**3  + 10000 c x**2  + d x + e 
        #
        # y'(0)= d = 0
        # y(x)_= 1000000 a x**4 + 100000 b x**3  + 10000 c x**2  + e 
        M=mat([ [ 100000000*del2**4,    1000000*del2**3,    100*del2**2,    1],    # y(x2)
                [ 400000000*del2**3,    3000000*del2**2,    200*del2,       0],    # y'(x2)
                [ 1200000000*del2**2,   6000000*del2,       200,            0],    # y''(x2)
                [ 100000000*del1**4,    1000000*del1**3,    100*del1**2,    1]])   # y(x1)  force symmetry

        P = solve(M,A)

        # Is there a better way to do this?
        P = concatenate((multiply(mat([100000000, 1000000, 100]),P[0:3,0].T), mat([0]), mat([P[3,0]])), 1)

    yp = polyval(P.T,x)[0]
    if y >= yp:
        # We're above the cut
        return [0, qsr]

    # We're below the cut so map to the cut line and
    # continue with the vector evaluation
    qsr = V + T*mat([[x],[yp]])
    return [vtx_check, qsr]


def xor(a, b):
    return (a and not b) or (b and not a)

def beta_function(q,qx,P0,N0,Pin,Nin,Vtx,hessian=False,Bfact=1):
    inside = is_inside(q,P0,N0,Pin,Nin,Vtx)
    qv = q-qx
    nq = norm(qv)

    B = 1 # Initialize the distance function
    Bi = mat(zeros((1,Pin.shape[1])))
    for i in xrange(Pin.shape[1]):
        Bi[0,i] = ((q-Pin[:,i]).T*Nin[:,i])[0,0]  # contribution of each face of polygon
        if Bi[0,i] < -1.e-2*Bfact:
            Bi[0,i] = -1.e-2*Bfact # Limit how large the negative can be
        B = B*Bi[0,i]

    # Calculate the distance product and partials
    if not P0.size == 0: 
        B0 = ((P0-q).T*N0)[0,0] # Convert to inward pointing normal for exit face
        if B0 < -1.e-2*Bfact:
            B0 = -1.e-2*Bfact # Limit how large the negative can be
        # Initialize the partial calculation based on exit face 
        DxB = -N0[0,0]*B # Should be inward pointing normal for distance to face 
        DyB = -N0[1,0]*B 
    else:
        # If no exit face is defined, this is a closed polygon.
        # Initialize the values for the computation
        B0=1
        DxB = 0
        DyB = 0

    # Complete the distance calculation
    B = B*B0

    vertex_region = 0
    if not inside and B > 0:
        # Must be in a vertex region (2 negatives - only 2 given convex polygon)
        B = -B
        vertex_region = 1
        if B0 < 0:
            if -N0[0,0] > 0:
                # Inward point normal component in positive direction
                # and we're on negative side, so positive movement acts to
                # increase the distance function
                DxB = abs(DxB) # Acts to increase B
            else:
                # Inward point normal negative, so positive movement
                # makes us more negative
                DxB = -abs(DxB) # Acts to decrease B
            if -N0[1,0] > 0:
                # Inward point normal component in positive direction
                # and we're on negative side, so positive movement acts to
                # increase the distance function
                DyB = abs(DyB)
            else:
                # Inward point normal negative, so positive movement
                # makes us more negative
                DyB = -abs(DyB)
        else:
            DxB = -DxB # B should have been negative when we did this
            DyB = -DyB # calculation

    # Now calculate the partials of the distance function
    for i in xrange(Pin.shape[1]):
        Bp = B0
        for j in xrange(Pin.shape[1]):
            if i != j:
                Bp = Bp*Bi[0,j]
        if vertex_region:
            # TODO: This is a silly if-else...
            if Bi[0,i] > 0:
                # The distance product in this zone 
                # should be negative, but it won't 
                # since we'll have a double negative above
                Bp = -Bp
            elif Bi[0,i] < 0:
                # The distance product in this zone 
                # should be positive, but it won't 
                # since we'll have a single negative above
                Bp = -Bp
       
        Dx=Bp*Nin[0,i] # Acts consistent with the normal
        Dy=Bp*Nin[1,i]            

        DxB = DxB + Dx
        DyB = DyB + Dy

    if not hessian:
        DxxB=mat([])
        DyyB=mat([])
        DxyB=mat([])
    else:
        # I'm calculating the hessian here just to make the code clearer.
        # It would be more efficient to use earlier loops
        if P0.size == 0:
            # initialize
            DxxB = 0
            DyyB = 0
            DxyB = 0
        else: 
            # Calculate partial due to exit face 
            i=0 #for i=1:size(Pin,2)
            DxxS=0
            DyyS=0
            DxyS=0
            for j in xrange(Pin.shape[1]): # j never = i for 0 edge
                # Each partial will consider one normal on inlet faces
                Bp=1
             
                for k in xrange(Pin.shape[1]):
                    if k != j: # can never = i=0
                        Bp=Bp*Bi[0,k]
                
                if vertex_region:
                    if B0 > 0 and Bi[0,j] > 0:
                        # The distance product in this zone 
                        # should be negative, but it won't 
                        # since we'll have a double negative above
                        Bp = -Bp
                    elif xor((B0 < 0),(Bi[0,j] < 0)):
                        # The distance product in this zone 
                        # should be positive, but it won't 
                        # since we'll have a single negative above
                        Bp = -Bp
                    elif B0 < 0 and Bi[0,j] < 0:
                        # The distance product in this zone 
                        # should be negative to counteract the double
                        # negative effect of B0 and Bi(j) 
                        Bp = -Bp
                    # otherwise it is already positive as needed
             
                # This acts consistent with sign of normal and sign
                # of the distance product calculated above
                
                Dxx = Nin[0,j]*Bp
                Dyy = Nin[1,j]*Bp
                Dxy = Nin[1,j]*Bp

                DxxS = DxxS + Dxx
                DyyS = DyyS + Dyy
                DxyS = DxyS + Dxy
             
            # reverse normal for calculation
            DxxB = -N0[0,0]*DxxS
            DyyB = -N0[1,0]*DyyS
            DxyB = -N0[0,0]*DxyS

        for i in xrange(Pin.shape[1]):
            DxxS = 0
            DyyS = 0
            DxyS = 0
          
            for j in xrange(Pin.shape[1]):
                Bp=1
                if i != j:
                    # consider partials with the other inlet faces
                    for k in xrange(Pin.shape[1]):
                        if k != i and k != j:
                            Bp=Bp*Bi[0,k]
                        elif k != i and k == j:
                            # Include the exit face here
                            Bp=Bp*B0
                    if vertex_region:
                        if Bi[0,i] > 0 and Bi[0,j] > 0:
                            # The distance product in this zone 
                            # should be negative, but it won't 
                            # since we'll have a double negative above
                            Bp = -Bp
                        elif xor((Bi[0,i] < 0),(Bi[0,j] < 0)):
                            # The distance product in this zone 
                            # should be positive, but it won't 
                            # since we'll have a single negative above
                            Bp = -Bp
                        elif Bi[0,i] < 0 and Bi[0,j] < 0:
                            # We need to counteract the double negative
                            # effect of Bi(i) and Bi(j) with normals
                            Bp = -Bp
                        # otherwise positive
                  
                        Dxx = Nin[0,j]*Bp 
                        Dyy = Nin[1,j]*Bp
                        Dxy = Nin[1,j]*Bp
                  
                elif not P0.size == 0:
                    # i==j use this for the 0 edge
                    Bp = 1
                  
                    for k in xrange(Pin.shape[1]):
                        if k != j:
                            Bp=Bp*Bi[0,k]
                    
                    if vertex_region:
                        if Bi[0,i] > 0 and B0 > 0: 
                            # The distance product in this zone 
                            # should be negative, but it won't 
                            # since we'll have a double negative above
                            Bp = -Bp
                        elif xor((Bi[0,i] < 0),(B0 < 0)):
                            # The distance product in this zone 
                            # should be positive, but it won't 
                            # since we'll have a single negative above
                            Bp = -Bp
                        elif Bi[0,i] < 0 and B0 < 0:
                            # counteract double negative
                            Bp = -Bp
                        #otherwise positive

                    Dxx = -N0[0,0]*Bp 
                    Dyy = -N0[1,0]*Bp
                    Dxy = -N0[1,0]*Bp

                else:
                    Dxx = 0
                    Dyy = 0
                    Dxy = 0

                DxxS=DxxS+Dxx
                DyyS=DyyS+Dyy
                DxyS=DxyS+Dxy
            DxxB = DxxB + DxxS*Nin[0,i]
            DyyB = DyyB + DyyS*Nin[1,i]
            DxyB = DxyB + DxyS*Nin[0,i]          

    return [B,DxB,DyB,DxxB,DyyB,DxyB]

def beta_minimizer(q, P0, N0, Pin, Nin, Vtx):
    """
    Return negative of beta function for use in fminsearch
    to find the point of maximum value
    """
    q = mat(q).T
    B = -distance_product(q,P0,N0,Pin,Nin)
    if not is_inside(q,P0,N0,Pin,Nin,Vtx):
        B = abs(B)
    return B


def Xoq_penn(q,P0,N0,Pin,Nin,qx,ae1,ae2,Bmax,hessian,Vtx,Brad):
    """
    This function maps the solution to Laplace's equation for
    steady state temperature distribution given boundary conditions
    on a unit disk to a given polygon.  
    Returns:
       X = X(q) normalized vector function 
     DqX = Vector field derivative
       F = Function value at q (just pull back - not based on scaled gradient)
    inside= is the coordinate inside the polygon?
    trouble

    Requires:
       q  = 2x1 coordinate vector
      (P0,N0) = definition of exit face
      (Pin,Nin) = matrix defining the other boundary faces
       qx = coordinate of map center
      ae1,ae2= angle limits of for unit circle (calculated once for each polygon)
    hessian= 0=skip hessian calc, 1=do hessian calc
    Bmax  = maximum Beta value used by some mappings
    map_choice = choice of polygon to disk mapping

     copyright 2003-2005 David C. Conner (dcconner@cmu.edu), 
       revised 11/04/05 dcc  - simplified based on Xoq
    """

    [F,inside,G,H,Hn,J,vtx_check] = \
       polygon_heat_penn(q,P0,N0,Pin,Nin,qx,ae1,ae2,Bmax,hessian,Vtx,Brad)
       
    # Assign reference vector
    X   = G.T 
    DqX = H

    return [X,DqX,F,inside,J]


def polygon_heat_penn(q,P0,N0,Pin,Nin,qx,ae1,ae2,Bmax,hessian,Vtx,Brad):
    """
    This function maps the solution to Laplace's equation for
    steady state temperature distribution given boundary conditions
    on a unit disk to a given polygon.  
    Returns:
       F = Function value at q (just pull back - not based on scaled gradient)
    inside= is the coordinate inside the polygon?
       G = "Normalized" Gradient of F at q => G = D_{\phi(q)}F D_q \phi /norm(D_{Phi(q)}F)
       H = "Hessian" Change in the vector field G at q =>  H = D_q G
       J = Jacobian of Phi at q   =>   J = D_q \phi (q)
      vtx_check = is point near a vertex (and therefore need smoothing)
       Hn = hessian norm

    Requires:
       q  = 2x1 coordinate vector
      (P0,N0) = definition of exit face (center point and normal)
      (Pin,Nin) = matrix defining the other boundary faces (center point and normal)
       qx = coordinate of map center
      ae1,ae2= angle limits of for unit circle (calculated once for each polygon)
    Bmax  = maximum Beta value used by some mappings
    hessian = 0=skip hessian calc, 1=do hessian calc
    vtx  = matrix of polygon vertices (beginning and ending with exit vertex)

     copyright 2003-2005 David C. Conner (dcconner@cmu.edu), 
       revised 11/04/05 dcc  - simplified based on polygon_heat_scaled
    """

    # Carry on with the calculation even if outside so as not to screw up Matlab 
    # integration.
    #
    # Points outside the boundary map to the boundary with appropriate vector field
    # Count on the event simulation to handle the errors
    inside = is_inside(q,P0,N0,Pin,Nin,Vtx)

    # Map to the disk world return coordinates (w,z) in disk world and
    # the coordinate in the recentered and rotated polygon
    [qc,qs,R,J,Hp,B,DxB,DyB,DxxB,DyyB,DxyB,vtx_check,limit] = map2diskScale(q,P0,N0,Pin,Nin,qx,ae1,ae2,Bmax,hessian,Vtx,Brad)
    [Fc,Gc,Hc,CNc,discontinuity] = disk_heat(qc,ae1,ae2,hessian)

    F = Fc  # Just the pull back - ignore the result of scaling gradient

    # x,y in polygon   w,z in disk
    DxW = J[0,0]
    DyW = J[0,1]
    DxZ = J[1,0]
    DyZ = J[1,1]
      
    DwF = Gc[0,0]
    DzF = Gc[0,1]
      
    # Vector norms
    NG2=(DwF*DxW + DxZ*DzF)**2 + (DwF*DyW + DyZ*DzF)**2
    NG = sqrt(NG2)
    NG32 = NG**3

    if NG > 0:
        DxF=(DwF*DxW + DxZ*DzF)/NG
        DyF=(DwF*DyW + DyZ*DzF)/NG
    else:
        DxF = 0
        DyF = 0
        NG  = 1 # Protect against divide by 0
        NG32= 1
         
    G = -mat([DxF, DyF])
    # Rotate back to original frame
    # Since G is 1x2 row vector, we'd normally multiply by R' to rotate, 
    # but we want to rotate opposite of original rotation R, we need R**{-1),
    # so R**{-1}' = R
    G=G*R
      
    if hessian: # & inside)       dcc 1/28/04
      
        DxxW = Hp[0,0]
        DyyW = Hp[0,1]
        DxyW = Hp[0,2]
        DxxZ = Hp[1,0]
        DyyZ = Hp[1,1]
        DxyZ = Hp[1,2]
      
        DwwF = Hc[0,0]
        DzzF = Hc[1,1]
        DwzF = Hc[0,1]
      
        DxxF= ((DwF*DyW + DyZ*DzF)*(DwF**2*(-(DxW*DxyW) + DxxW*DyW) + \
          DzF*(DwwF*DxW*(-(DxZ*DyW) + DxW*DyZ) + \
          DwzF*DxZ*(-(DxZ*DyW) + DxW*DyZ) - DxyZ*DxZ*DzF + DxxZ*DyZ*DzF) + \
          DwF*(DwzF*DxW*(DxZ*DyW - DxW*DyZ) - DxyW*DxZ*DzF + DxxZ*DyW*DzF + \
          DxxW*DyZ*DzF + DxZ**2*DyW*DzzF - \
          DxW*(DxyZ*DzF + DxZ*DyZ*DzzF))))/NG32
        DxyF= ((DwF*DyW + DyZ*DzF)*(DwF**2*(DxyW*DyW - DxW*DyyW) + \
          DzF*(DwwF*DyW*(-(DxZ*DyW) + DxW*DyZ) + \
          DwzF*DyZ*(-(DxZ*DyW) + DxW*DyZ) - DxZ*DyyZ*DzF + DxyZ*DyZ*DzF) + \
          DwF*(DwzF*DyW*(DxZ*DyW - DxW*DyZ) + DxyZ*DyW*DzF - DxZ*DyyW*DzF -\
          DxW*DyyZ*DzF + DxyW*DyZ*DzF + DxZ*DyW*DyZ*DzzF - \
          DxW*DyZ**2*DzzF)))/NG32
        DyxF= ((DwF*DxW + DxZ*DzF)*(DwF**2*(DxW*DxyW - DxxW*DyW) + \
          DzF*(DwzF*DxZ**2*DyW - DwzF*DxW*DxZ*DyZ + \
          DwwF*DxW*(DxZ*DyW - DxW*DyZ) + DxyZ*DxZ*DzF - DxxZ*DyZ*DzF) + \
          DwF*(DwzF*DxW*(-(DxZ*DyW) + DxW*DyZ) + DxW*DxyZ*DzF + DxyW*DxZ*DzF - \
          DxxZ*DyW*DzF - DxxW*DyZ*DzF - DxZ**2*DyW*DzzF + \
          DxW*DxZ*DyZ*DzzF)))/NG32
        DyyF= ((DwF*DxW + DxZ*DzF)*(DwF**2*(-(DxyW*DyW) + DxW*DyyW) + \
          DzF*(DwzF*DxZ*DyW*DyZ - DwzF*DxW*DyZ**2 + \
          DwwF*DyW*(DxZ*DyW - DxW*DyZ) + DxZ*DyyZ*DzF - DxyZ*DyZ*DzF) + \
          DwF*(DwzF*DyW*(-(DxZ*DyW) + DxW*DyZ) - DxyZ*DyW*DzF + DxZ*DyyW*DzF + \
          DxW*DyyZ*DzF - DxyW*DyZ*DzF - DxZ*DyW*DyZ*DzzF + \
          DxW*DyZ**2*DzzF)))/NG32

        H = -mat([[DxxF,DxyF],[DyxF,DyyF]])  # change in vector field
                                             # of negative gradient so use
                                             # negative sign with matrix 
                                   
        # Rotate back to original frame
        H = R.T*H*R  # transform into the original polygon coordinate frame
      
      
        # Spectral Norm                             
        nH2 = eig(H.T*H)[0]       
        Hn = sqrt(nH2.max(0))
                          
    # A final check   
    if rank(J) < 2 or cond(J) > 1e8:
        print "Jphi near singular - must be at vertex"
        Nj=mat([[0],[0]])
        Sj=0
        for i in xrange(Pin.shape[1]):
            d=((q-Pin[:,i]).T*Nin[:,i])[0,0]
            if d < 1e-6:
                Nj=Nj+Nin[:,i]
                Sj=Sj + (1-d)

        if Sj > 0:
            print "Modify gradient at vertex"
            G = (Nj/Sj).T  
            H = mat(zeros((2,2)))
            Hn= 0

        print "End of J fix"

    return [F,inside,G,H,Hn,J,vtx_check] 


def disk_heat(q,ae1,ae2,hessian=False):
    """
    This function calculates the solution to Laplace's equation for
    steady state temperature distribution given boundary conditions
    on a unit disk.  The boundary is = 1 for ae1 to ae2, and
    0 else where

    Returns:
       F = Function value
       G = Gradient of F = DqF (cartesian gradient)
       H = Hessian  of F = Dq(DqF)
      cn = condition number of Hessian
    discontinuity= 0=N/A, 1=at boundary 

    Requires:
       q = 2x1 coordinate vector
      ae1= limit of the inlet region (least bound in -pi to pi)
      ae2= limit of the inlet region (upper bound in -pi to pi)
      hessian= 0=skip hessian calc, 1=do hessian calc

     copyright 2003-2004 David C. Conner, 
    """

    H=mat([])

    dae = 2*pi - ae2 + ae1
    ae1 = ae1 - 0.005*dae
    ae2 = ae2 + 0.005*dae

    r = norm(q)               # Get the radius of the coordinate
    theta = atan2(q[1,0],q[0,0]) # Get the angle wrt x-axis
      
    inside = is_inside_circ(q) # Is this point in the unit disk (including boundary)
      
    if not inside and (r-1)<100*eps:
        # Assume numerical noise in conversion from boundary
        q=q/(norm(q)+50*eps) 
        r = 1.0
        inside = is_inside_circ(q)

    if not inside:
        # Really outside
        if theta > ae1 or theta < ae2:
            F = 0
        else:
            F = 1
    else:
        # Inside so calculate the heat correctly
        # Equation reduced using definition of the log of complex number 
        F =  (ae2-ae1)/(2*pi) - atan2((r*sin(ae1-theta)), 1 - r*cos(ae1-theta))/pi + \
            atan2( r*sin(ae2-theta),  1 - r*cos(ae2-theta))/pi

    if r < 2*eps:
        # Do special calculations based on limits as r->0 found with Mathematica
        lDwF=(-sin(ae1) + sin(ae2))/pi
        lDzF=(cos(ae1) - cos(ae2))/pi
        G=mat([lDwF,lDzF])
        cn = 0
        discontinuity = 0
        limit =1
        if hessian:
            lDwwF=(-sin(2*ae1) + sin(2*ae2))/pi
            lDzzF=(sin(2*ae1) - sin(2*ae2))/pi
            lDwzF=(cos(2*ae1) - cos(2*ae2))/pi
            H = mat([[lDwwF, lDwzF],
                     [lDwzF, lDzzF]])
            cn=cond(H)
        return [F,G,H,cn,discontinuity,limit]
    else:
        limit=0
        # It gets a little crazy at these isolated points, so let's push off
        # from these points in the calculations
        if (abs(r-1) < 1e-8) and ((abs(ae1-theta) < 1e-6) or (abs(ae2-theta) < 1e-6)):
            # Disk heat singularity
            discontinuity = 1
            if theta < ae1:
                # Get us off of the singularity a < ae1
                theta =  ae1 - 1e-7
            elif theta > ae2:
                # Get us off of the singularity a > ae2
                theta = ae2 + 1e-7
            else:
                # Get us off of the singularity ae1 <= a <= ae2
                if theta > ae1:
                    # must be near ae1   
                    theta = ae1+1e-7
                else:
                    # must be near ae2
                    theta = ae2 - 1e-7
               
            # Update the mapping to reflect change
            # Hopefully this will help partial derivative continuity
            if r > 1.:
                r = r + 1.e-8
            else:
                r = r - 1.e-8
            
            q =r*mat([[cos(theta)],[sin(theta)]])

        elif (abs(r-1) < 1e-6) and ((abs(ae1-theta) < 1e-6) or (abs(ae2-theta) < 1e-6)):
            discontinuity = -1 # Warning
        else:
            discontinuity = 0
         
        w = q[0,0] # Extract for cosmetic reasons 
        z = q[1,0]
         
        DwF= ((-2*r*z + 2*z*cos(ae1 - theta) + 2*w*sin(ae1 - theta))/(-(r*(1 + r**2)) + \
              2*r**2*cos(ae1 - theta)) + (-2*r*z + 2*z*cos(ae2 - theta) + 2*w*sin(ae2 - theta))/(r*(1 + r**2) - \
              2*r**2*cos(ae2 - theta)))/(2.*pi)

        DzF= ((2*(r*w - w*cos(ae1 - theta) + z*sin(ae1 - theta)))/(-(r*(1 + r**2)) + \
               2*r**2*cos(ae1 - theta)) + (2*(r*w - w*cos(ae2 - theta) + z*sin(ae2 - theta)))/(r*(1 + r**2) - \
               2*r**2*cos(ae2 - theta)))/(2.*pi)

        G = mat([DwF,DzF])

        if not hessian:
            # Skip the hessian calaculations if we're only doing kinematic solution
            H=mat([])
            cn=1
        else:
            # Do the hessian calculations         
            DwwF= -((-((w*(w**2*z + z**3 - r*z*cos(ae1 - theta) - r*w*sin(ae1 - theta)))/(1 + r**2 - 2*r*cos(ae1 - theta))) + \
                  (r*(r*z - z*cos(ae1 - theta) - w*sin(ae1 - theta))*(w + 3*r**2*w - 4*r*w*cos(ae1 - theta) + \
                  2*r*z*sin(ae1 - theta)))/(1 + r**2 - 2*r*cos(ae1 - theta))**2 + \
                  (w*(w**2*z + z**3 - r*z*cos(ae2 - theta) - r*w*sin(ae2 - theta)))/(1 + r**2 - 2*r*cos(ae2 - theta)) + \
                  (r*(-(r*z) + z*cos(ae2 - theta) + w*sin(ae2 - theta))*\
                  (w + 3*r**2*w - 4*r*w*cos(ae2 - theta) + 2*r*z*sin(ae2 - theta)))/(1 + r**2 - 2*r*cos(ae2 - theta))**2)/(pi*r**4))

            DzzF=((-(r*w*z) + w*z*cos(ae1 - theta) + (-r**2 + w**2)*sin(ae1 - theta))/(1 + r**2 - 2*r*cos(ae1 - theta)) + \
                  ((z + 3*r**2*z - 4*r*z*cos(ae1 - theta) - 2*r*w*sin(ae1 - theta))*(r*w - w*cos(ae1 - theta) + z*sin(ae1 - theta)))/\
                  (1 + r**2 - 2*r*cos(ae1 - theta))**2 + (r*w*z - w*z*cos(ae2 - theta) + (r**2 - w**2)*sin(ae2 - theta))/\
                  (1 + r**2 - 2*r*cos(ae2 - theta)) - ((z + 3*r**2*z - 4*r*z*cos(ae2 - theta) - 2*r*w*sin(ae2 - theta))*\
                  (r*w - w*cos(ae2 - theta) + z*sin(ae2 - theta)))/\
                  (1 + r**2 - 2*r*cos(ae2 - theta))**2)/(pi*r**3)
      
            DwzF= -(((r*(r*z - z*cos(ae1 - theta) - w*sin(ae1 - theta))*(z + 3*r**2*z - 4*r*z*cos(ae1 - theta) - \
                  2*r*w*sin(ae1 - theta)))/(1 + r**2 - 2*r*cos(ae1 - theta))**2 + \
                  (-w**4 - 3*w**2*z**2 - 2*z**4 + r*z**2*cos(ae1 - theta) + r*w*z*sin(ae1 - theta))/\
                  (1 + r**2 - 2*r*cos(ae1 - theta)) - (r*(r*z - z*cos(ae2 - theta) - w*sin(ae2 - theta))*\
                  (z + 3*r**2*z - 4*r*z*cos(ae2 - theta) - 2*r*w*sin(ae2 - theta)))/\
                  (1 + r**2 - 2*r*cos(ae2 - theta))**2 + \
                  (w**4 + 3*w**2*z**2 + 2*z**4 - r*z**2*cos(ae2 - theta) - \
                  r*w*z*sin(ae2 - theta))/(1 + r**2 - 2*r*cos(ae2 - theta)))/(pi*r**4))
      
            H = mat([[DwwF, DwzF],
                     [DwzF, DzzF]])
            
            cn = cond(H)
      
    return [F,G,H,cn,discontinuity]
