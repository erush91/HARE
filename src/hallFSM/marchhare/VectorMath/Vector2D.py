#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

"""
Vector.py , Built on Spyder for Python 2.7
Erik Lindstrom , Adam Sperry , James Watson, 2016 October
Vectors, Turns, Frames, and common operations in 2D geometry

== LOG ==


== TODO ==

"""

# ~ Standard Libraries ~
from math import sqrt , cos , sin , acos , pi , atan2 , e
# ~ Special Libraries ~
import Tkinter
import numpy as np
from scipy.spatial import ConvexHull
# ~ Local Libraries ~
if __name__ == "__main__":
    import sys , os
    sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath( __file__ ) ) ) )
from marchhare.marchhare import PriorityQueue , elemw , format_dec_list , print_list, tandem_sorted
from marchhare.MathKit import wrap_bounds_fraction , roundint , eq , round_small 
from marchhare.Vector import vec_mag , vec_unit , vec_proj , vec_round_small , vec_copy , vec_from_seg , vec_avg , vec_dif_mag , is_vector , bbox_from_points , vec_copy_deep

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026

# == Coordinate Transform 2D ==

def polr_2_cart( polarCoords ): # usual conversion 
    """ Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is X+ """
    return [ polarCoords[0] * cos(polarCoords[1]) , polarCoords[0] * sin(polarCoords[1]) ]
    # TODO : Look into imaginary number transformation and perform a benchmark
    
def cart_2_polr(cartCoords): # usual conversion # <<< resenv
    """ Convert cartesian coordinates [x , y] to polar coordinates [radius , angle (radians)]. Theta = 0 is X+ """
    return [ vec_mag(cartCoords) , atan2( cartCoords[1] , cartCoords[0] ) ]

def polr_2_cart_0Y(polarCoords): # 0 angle is +Y North 
    """ Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ """
    return [ polarCoords[0] * sin(polarCoords[1]) , polarCoords[0] * cos(polarCoords[1])  ]
    # TODO : Look into imaginary number transformation and perform a benchmark
    
def cart_2_polr_0Y(cartCoords): # 0 angle is +Y North 
    """ Convert cartesian coordinates [x , y] to polar coordinates [radius , angle (radians)]. Theta = 0 is UP = Y+ """
    return [ vec_mag(cartCoords) , atan2( -cartCoords[0] , cartCoords[1] ) ]
    
# == End Transform 2D ==

# == General 2D Functions ==

def ray_angle( ray ):
    """ Return the direction of the ray in radians """ # [ [ x0 , y0 ] , [ x1 , y1 ] ] : +p0 ------- +p1 ----->
    return cart_2_polr( vec_from_seg( ray ) )[1] # Return only the angle portion of polar equivalent of the direction vector

def circ_spacing( dia , numPts , center = None ): 
    """ Return a list of 'numPts' points equally spaced around a 2D circle with a center at (0,0), or at 'center' if specified """
    div = 2 * pi / numPts
    circPts = []
    for pntDex in range(numPts):
        offset = polr_2_cart_0Y( [ dia/2 , pntDex * div ] ) 
        if center:
            circPts.append( np.add( center , offset ) )
        else:
            circPts.append( offset )
    return circPts

def winding_num( point , polygon , excludeCollinear = True):
    """ Find the winding number of a point with respect to a polygon , works for both CW and CCWpoints """
    # NOTE: Function assumes that the points of 'polygon' are ordered. Algorithm does not work if they are not 
    # This algorithm is translation invariant, and can handle convex, nonconvex, and polygons with crossing sides.
    # This algorithm does NOT handle the case when the point lies ON a polygon side. For this problem it is assumed that
    #there are enough trial points to ignore this case
    # This works by shifting the point to the origin (preserving the relative position of the polygon points), and 
    #tracking how many times the positive x-axis is crossed
    w = 0
    v_i = []
    x = lambda index: elemw( v_i , index )[0] # We need wrapping indices here because the polygon is a cycle
    y = lambda index: elemw( v_i , index )[1]
    for vertex in polygon: # Shift the point to the origin, preserving its relative position to polygon points
        v_i.append( np.subtract( vertex , point ) )
    for i in range(len(v_i)): # for each of the transformed polygon points, consider segment v_i[i]-->v_i[i+1]
        
        if excludeCollinear and eq( 0 , d_point_to_segment_2D( [ 0 , 0 ] , [ [ x(i) , y(i) ] , [ x(i+1) , y(i+1) ] ] ) ):
            return 0
    
        if y(i) * y(i + 1) < 0: # if the segment crosses the x-axis
            r = x(i) + ( y(i) * ( x(i + 1) - x(i) ) ) / ( y(i) - y(i + 1) ) # location of x-axis crossing
            if r > 0: # positive x-crossing
                if y(i) < 0: 
                    w += 1 # CCW encirclement
                else:
                    w -= 1 #  CW encirclement
        # If one of the polygon points lies on the x-axis, we must look at the segments before and after to determine encirclement
        elif ( eq( y(i) , 0 ) ) and ( x(i) > 0 ): # v_i[i] is on the positive x-axis, leaving possible crossing
            if y(i + 1) > 0:
                w += 0.5 # possible CCW encirclement or switchback from a failed CW crossing
            else:
                w -= 0.5 # possible CW encirclement or switchback from a failed CCW crossing
        elif ( eq( y(i + 1) , 0 ) ) and ( x(i + 1) > 0 ): # v_i[i+1] is on the positive x-axis, approaching possible crossing
            if y(i) < 0:
                w += 0.5 # possible CCW encirclement pending
            else:
                w -= 0.5 # possible  CW encirclement pending
    return w

def point_in_poly_w( point , polygon ): 
    """ Return True if the 'polygon' contains the 'point', otherwise return False, based on the winding number """
    # print "Winding Number:" , winding_num(point , polygon)
    return not eq( winding_num( point , polygon ) , 0 ) # The winding number gives the number of times a polygon encircles a point 
    
def point_in_triangle( p , v0 , v1 , v2 ):
    """ Return True if 'p' lies within a triangle defined by 'v0' , 'v1' , 'v2' """
    # URL , Point in triangle: https://stackoverflow.com/a/34093754/893511
    dX   = p[0] - v2[0];
    dY   = p[1] - v2[1];
    dX21 = v2[0] - v1[0];
    dY12 = v1[1] - v2[1];
    D    = dY12 * ( v0[0] - v2[0] ) + dX21 * ( v0[1] - v2[1] );
    s    = dY12 * dX + dX21 * dY;
    t    = ( v2[1] - v0[1] ) * dX + ( v0[0] - v2[0] ) * dY;
    if D < 0:
        return s <= 0 and t <= 0 and s + t >= D;
    return s >= 0 and t >= 0 and s + t <= D;

def d_point_to_segment_2D( point , segment ): 
    """ Return the shortest (perpendicular) distance between 'point' and a line 'segment' """
    # URL: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    segPt1 = segment[0] ; segPt2 = segment[1]
    return abs( ( segPt2[0] - segPt1[0] ) * ( segPt1[1] - point[1] ) - \
                ( segPt1[0] - point[0] ) * ( segPt2[1] - segPt1[1] ) ) / sqrt( ( segPt2[0] - segPt1[0] )**2 + ( segPt2[1] - segPt1[1] )**2 )
    
def d_point_to_segment_2D_signed( point , segment ): 
    """ Return the signed ( + RHS , - LHS ) perpendicular distance between 'point' and a line 'segment' defined from tail to head """
    # URL: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    segPt1 = segment[0] ; segPt2 = segment[1]
    num = ( segPt2[0] - segPt1[0] ) * ( segPt1[1] - point[1] ) - ( segPt1[0] - point[0] ) * ( segPt2[1] - segPt1[1] )
    return abs( num ) / sqrt( ( segPt2[0] - segPt1[0] )**2 + ( segPt2[1] - segPt1[1] )**2 ) * np.sign( num )

def intersect_seg_2D( seg1 , seg2 , slideCoincident = False , includeEndpoints = True ): 
    """ Return true if line segments 'seg1' and 'seg2' intersect, otherwise false """
    # URL: http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
    # NOTE: 'uA' and 'uB' could be used to calc intersection point if desired, see above URL
    den =   (seg2[1][1] - seg2[0][1]) * (seg1[1][0] - seg1[0][0]) - (seg2[1][0] - seg2[0][0]) * (seg1[1][1] - seg1[0][1])
    uAnum = (seg2[1][0] - seg2[0][0]) * (seg1[0][1] - seg2[0][1]) - (seg2[1][1] - seg2[0][1]) * (seg1[0][0] - seg2[0][0])
    uBnum = (seg1[1][0] - seg1[0][0]) * (seg1[0][1] - seg2[0][1]) - (seg1[1][1] - seg1[0][1]) * (seg1[0][0] - seg2[0][0])
    if den == 0:
        if eq(uAnum , 0.0) or eq(uBnum , 0.0):
            return True if not slideCoincident else False
        else:
            return False
    else:
        uA = uAnum * 1.0 / den
        uB = uBnum * 1.0 / den
        if ( includeEndpoints and (uA >= 0 and uA <= 1) and (uB >= 0 and uB <= 1) ) or ( (not includeEndpoints) and (uA > 0 and uA < 1) and (uB > 0 and uB < 1) ):
            return True
        else:
            return False

def intersect_pnt_2D( seg1 , seg2 ): # <<< resenv
    """ if line segments 'seg1' and 'seg2' intersect, then return intersection point , otherwise return false """
    #               { seg1: [ [x1,y1] , [x2,y2] ] , seg2: [ [x3,y3] , [x4,y4] ]  }
    # URL: http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
    den =   (seg2[1][1] - seg2[0][1]) * (seg1[1][0] - seg1[0][0]) - (seg2[1][0] - seg2[0][0]) * (seg1[1][1] - seg1[0][1])
    uAnum = (seg2[1][0] - seg2[0][0]) * (seg1[0][1] - seg2[0][1]) - (seg2[1][1] - seg2[0][1]) * (seg1[0][0] - seg2[0][0])
    uBnum = (seg1[1][0] - seg1[0][0]) * (seg1[0][1] - seg2[0][1]) - (seg1[1][1] - seg1[0][1]) * (seg1[0][0] - seg2[0][0])
    if den == 0:
        if eq(uAnum , 0.0) or eq(uBnum , 0.0):
            return vec_avg( seg1[0] , seg1[1] , seg2[0] , seg2[1] ) # Lines are coincident, return the average of segment centers, this is not overlap center
        else:
            return False # Lines are parallel
    else:
        uA = uAnum * 1.0 / den
        uB = uBnum * 1.0 / den
        if (uA >= 0 and uA <= 1) and (uB >= 0 and uB <= 1):
        # { seg1:[ [x1,y1] , [x2,y2] ] , seg2: [ [x3,y3] , [x4,y4] ]  }
        #   return [ x1 + uA * ( x2 - x1 ) , y1 + uA * ( y2 - y1 ) ]
            return [ seg1[0][0] + uA * (seg1[1][0] - seg1[0][0]) , seg1[0][1] + uA * (seg1[1][1] - seg1[0][1]) ]
        else:
            return False # Lines do not intersect

def intersect_ray_2D( ray1 , ray2 ): # <<< resenv
    """ if rays 'ray1' and 'ray2' intersect, then return intersection point , otherwise return False """
    #               { ray1: [ [x1,y1] , [x2,y2] ] , ray2: [ [x3,y3] , [x4,y4] ]  }
    # URL: http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
    den =   (ray2[1][1] - ray2[0][1]) * (ray1[1][0] - ray1[0][0]) - (ray2[1][0] - ray2[0][0]) * (ray1[1][1] - ray1[0][1])
    uAnum = (ray2[1][0] - ray2[0][0]) * (ray1[0][1] - ray2[0][1]) - (ray2[1][1] - ray2[0][1]) * (ray1[0][0] - ray2[0][0])
    uBnum = (ray1[1][0] - ray1[0][0]) * (ray1[0][1] - ray2[0][1]) - (ray1[1][1] - ray1[0][1]) * (ray1[0][0] - ray2[0][0])
    if den == 0:
        if eq(uAnum , 0.0) or eq(uBnum , 0.0):
            return vec_avg( ray1[0] , ray1[1] , ray2[0] , ray2[1] ) # Lines are coincident, return the average of segment centers, this is not overlap center
        else:
            return False # Lines are parallel
    else:
        uA = uAnum * 1.0 / den
        uB = uBnum * 1.0 / den
        if uA >= 0 and uB >= 0:
        # { ray1: [ [x1,y1] , [x2,y2] ] , ray2: [ [x3,y3] , [x4,y4] ]  }
        #   return [ x1 + uA * ( x2 - x1 ) , y1 + uA * ( y2 - y1 ) ]
            return [ ray1[0][0] + uA * (ray1[1][0] - ray1[0][0]) , ray1[0][1] + uA * (ray1[1][1] - ray1[0][1]) ]
        else:
            return False # Lines do not intersect
            
def intersect_ray_seg_pnt_2D( ray , seg , endPoints = True , slideCoincident = False ): # <<< resenv
    """ if 'ray' and line 'seg' intersect, then return intersection point , otherwise return false 
    endPoints = False: Ignore intersections that happen at the segment endpoints
    slideCoincident = True: Count the case where the segment and the ray are coincident as non-intersecting """
    #               { ray: [ [x1,y1] , [x2,y2] ] , seg: [ [x3,y3] , [x4,y4] ]  }
    #                        + ------- + --->             + ------- +
    # URL: http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
    den =   (seg[1][1] - seg[0][1]) * (ray[1][0] - ray[0][0]) - (seg[1][0] - seg[0][0]) * (ray[1][1] - ray[0][1])
    uAnum = (seg[1][0] - seg[0][0]) * (ray[0][1] - seg[0][1]) - (seg[1][1] - seg[0][1]) * (ray[0][0] - seg[0][0])
    uBnum = (ray[1][0] - ray[0][0]) * (ray[0][1] - seg[0][1]) - (ray[1][1] - ray[0][1]) * (ray[0][0] - seg[0][0])
    if den == 0:
        if eq(uAnum , 0.0) or eq(uBnum , 0.0):
            if not slideCoincident:
                return vec_avg( ray[0] , ray[1] , seg[0] , seg[1] ) # Lines are coincident, return the average of segment centers, this is not overlap center
            else:
                return False # Do not consider coincident lines and rays to be colliding, since we are allowing ends to "slide past" each other
        else:
            return False # Lines are parallel
    else:
        uA = uAnum * 1.0 / den
        uB = uBnum * 1.0 / den
        if (uA >= 0) and ( (uB >= 0 and uB <= 1) if endPoints else (uB > 0 and uB < 1) ):
        # { seg1:[ [x1,y1] , [x2,y2] ] , seg2: [ [x3,y3] , [x4,y4] ]  }
        #   return [ x1 + uA * ( x2 - x1 ) , y1 + uA * ( y2 - y1 ) ]
            return [ ray[0][0] + uA * (ray[1][0] - ray[0][0]) , ray[0][1] + uA * (ray[1][1] - ray[0][1]) ]
        else:
            return False # Lines do not intersect

# TODO: Rewrite all the intersection functions this way so that we don't have to have a bunch of versions based on the needs of client code

def intersect_ray_seg_pnt_frac_2D( ray , seg ): 
    """ if 'ray' and line 'seg' intersect, then return intersection point , otherwise return false 
    endPoints = False: Ignore intersections that happen at the segment endpoints
    slideCoincident = True: Count the case where the segment and the ray are coincident as non-intersecting """
    #               { ray: [ [x1,y1] , [x2,y2] ] , seg: [ [x3,y3] , [x4,y4] ]  }
    #                        + ------- + --->             + ------- +
    # URL: http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
    den =   (seg[1][1] - seg[0][1]) * (ray[1][0] - ray[0][0]) - (seg[1][0] - seg[0][0]) * (ray[1][1] - ray[0][1])
    uAnum = (seg[1][0] - seg[0][0]) * (ray[0][1] - seg[0][1]) - (seg[1][1] - seg[0][1]) * (ray[0][0] - seg[0][0])
    uBnum = (ray[1][0] - ray[0][0]) * (ray[0][1] - seg[0][1]) - (ray[1][1] - ray[0][1]) * (ray[0][0] - seg[0][0])
    if den == 0:
        if eq(uAnum , 0.0) or eq(uBnum , 0.0):
            return vec_avg( ray[0] , ray[1] , seg[0] , seg[1] ) , None , None # Lines are coincident, return the average of segment centers, this is not overlap center
        else:
            return False , None , None # Lines are parallel
    else:
        uA = uAnum * 1.0 / den # Intersection distance along ray
        uB = uBnum * 1.0 / den # Intersection distance along segment
        if (uA >= 0) and ( uB >= 0 and uB <= 1 ):
        # { seg1:[ [x1,y1] , [x2,y2] ] , seg2: [ [x3,y3] , [x4,y4] ]  }
        #   return [ x1 + uA * ( x2 - x1 ) , y1 + uA * ( y2 - y1 ) ]
            return [ ray[0][0] + uA * (ray[1][0] - ray[0][0]) , ray[0][1] + uA * (ray[1][1] - ray[0][1]) ] , uA , uB
        else:
            return False , None , None # Lines do not intersect

def proj_pnt_onto_seg( pnt , seg ): 
    """ Project point 'pnt' onto segment 'seg' """
    segVec = np.subtract( seg[1] , seg[0] )
    offPnt = np.subtract( pnt , seg[0] )
    return np.add( seg[0] , np.multiply( vec_unit( segVec ) , vec_proj( offPnt , segVec ) ) )
    
def proj_pnt_within_seg( pnt , seg ):
    """ Return the least distance between 'pnt' projected onto 'seg' , positive for projected inside segment , negative to projected outside """
    segVec = np.subtract( seg[1] , seg[0] ) # Represent the segment as a vector from point 0 to point 1
    offPnt = np.subtract( pnt , seg[0] ) # Express the point relative to point 0
    dist = vec_proj( offPnt , segVec ) # Project the point onto the representation
    segVec = np.subtract( seg[0] , seg[1] ) # Represent the segment as a vector from point 1 to point 0
    offPnt = np.subtract( pnt , seg[1] ) # Express the point relative to point 0
    return min( dist , vec_proj( offPnt , segVec ) ) # # Project the point onto the representation , and determine the shortest distance to the bounds
            
def centroid_discrete_masses( massCenters , totalMass ): # <<< resenv
    """ Return the COM for a collection of point masses 'massCenters' with a known 'totalMass' """
    centroid = [ 0 for i in range( len( massCenters[0][1] ) ) ] # build a zero vector the same dimensionality as the data coords
    for massPoint in massCenters: # for every mass-point pair in the data
        for i, coord in enumerate( massPoint[1] ): # for every coordinate in the point
            centroid[i] += massPoint[0] / totalMass * coord # Add the coordinate scaled by it's mass distribution
    return centroid 

def poly2D_convex_area_centroid( polyPoints ): 
    """ Return the area and centroid of a convex polygon composed of at least three 'polyPoints' """
    # http://www.mathopenref.com/coordtrianglearea.html
    # Assume that we are dealing with convex polygons (hulls), otherwise use shoelace algorithm
    # NOTE: This algorithm will produce incorrect results for a nonconvex polygon!
    A = 0 # triangle area
    triCentroids = list() # Triangle centers
    totArea = 0 # total area of the polygon
    for pntDex in range( 2 , len( polyPoints ) ): # for each point in the polygon, starting with the third
        Ax = polyPoints[ pntDex   ][ 0 ] # The point at index 0 will always be one vertex of each triangle
        Bx = polyPoints[ pntDex-1 ][ 0 ]
        Cx = polyPoints[ 0        ][ 0 ]
        Ay = polyPoints[ pntDex   ][ 1 ]
        By = polyPoints[ pntDex-1 ][ 1 ]
        Cy = polyPoints[ 0        ][ 1 ]
        A = 0.5 * abs( Ax*(By-Cy) + Bx*(Cy-Ay) + Cx*(Ay-By) )
        totArea += A
        triCentroids.append( ( A , vec_avg( polyPoints[pntDex] , polyPoints[pntDex-1] , polyPoints[0] ) ) )
    return totArea , centroid_discrete_masses( triCentroids , totArea )

def get_area_and_centroid( vertices ):
    """ Return the area and the area centroid of a simple, closed polygon defined as a list of ordered 'vertices' """ 
    # NOTE: Vertices must be ordered
    # NOTE: Polygon need not be convex
    # Bourke, Paul. "Calculating the area and centroid of a polygon." Swinburne Univ. of Technology (1988).
    x = []
    y = []
    for vertex in vertices:           # Organize verticies in x and y lists.
        x.append( vertex[0] )
        y.append( vertex[1] )
    area = 0.5 * np.abs( np.dot( x , np.roll( y , 1 ) ) - np.dot( y , np.roll( x , 1 ) ) ) 
    cx = 0
    cy = 0
    for i in range( len( x ) ):
        cx += ( x[i] + elemw( x , i+1 ) ) * ( x[i] * elemw( y , i+1 ) - elemw( x , i+1 ) * y[i] )
        cy += ( y[i] + elemw( y , i+1 ) ) * ( x[i] * elemw( y , i+1 ) - elemw( x , i+1 ) * y[i] )
    return area , np.array( [ cx / ( 6 * area ) , cy / ( 6 * area ) ] )

def point_in_bbox( point , bbox ): # <<< resenv
    """ Return true if 'point' is inside a 'bbox' defined by [ [ xMin , yMin ] , [ xMax , yMax ] ] """
    return bbox[0][0] <= point[0] and point[0] <= bbox[1][0] and bbox[0][1] <= point[1] and point[1] <= bbox[1][1]

def vec2d_rot_90_CCW( vec2d ):
    """ Rotate 'vec2d' +90deg (+pi/2 rad) """
    return [ -vec2d[1] , vec2d[0] ] # This rotation is extremely easy in 2D
    
def pnt_rght_of_ray( pnt , ray ):
    """ Return True if 'pnt' is to the right of 'ray' , otherwise return False """ # [ [ x1 , y1 ] , [ x2 , y2 ] ]
    normalPosRot = vec2d_rot_90_CCW( np.subtract( ray[1] , ray[0] ) ) #                + ----------- + ----->
    return True if np.dot( np.subtract( pnt ,ray[0] ) , normalPosRot ) < 0 else False # Returns False if 'pnt' falls on 'ray'
    
def pnt_left_of_ray( pnt , ray ):
    """ Return True if 'pnt' is to the left of 'ray' , otherwise return False """ # [ [ x1 , y1 ] , [ x2 , y2 ] ]
    normalPosRot = vec2d_rot_90_CCW( np.subtract( ray[1] , ray[0] ) ) #               + ----------- + ----->
    return True if np.dot( np.subtract( pnt ,ray[0] ) , normalPosRot ) > 0 else False # Returns False if 'pnt' falls on 'ray'
    
def point_on_ray( point , ray ):
    """ Return True if 'point' is on the 'ray' , otherwise return False """ # [ [ x1 , y1 ] , [ x2 , y2 ] ]
    normalPosRot = vec2d_rot_90_CCW( np.subtract( ray[1] , ray[0] ) ) #         + ----------- + ----->
    return True if eq( np.dot( np.subtract( point ,ray[0] ) , normalPosRot ) , 0 ) else False # If projection of 'point' onto perpendicular 0, is on 'ray'
    
def angle_between( vec1 , vec2 ):
    """ Return the smallest angle between 'vec1' and 'vec2' """
    return acos( np.dot( vec_unit( vec1 ) , vec_unit( vec2 ) ) )
    
def seg_len( segment ):
    """ Return the length of 'segment' """
    return vec_dif_mag( segment[0] , segment[1] ) # Return the distance between the segment endpoints

# == End General 2D ==

# == Build Order Geometry ==

def are_parallel(p1, p2, p3, p4):
    """
    Takes two sets of points (x, y), tests if unit vectors are simillar and returns True
    p1 and p2 make first line
    p3 and p4 make second line 
    """
    v1 = p2 - p1
    v2 = p4 - p3
    norm = 0
    for i in v1:
        norm += i**2    #calculate magnitude of vector
    unit_v1 = v1/np.sqrt(norm)   #normalize vector ~ Unit Vector

    norm = 0
    for i in v2:
        norm += i**2
    unit_v2 = v2/np.sqrt(norm)

    if abs((unit_v2 - unit_v1).all()) <= 0:
        ##If u_vectors are identical, lines should be parallel, return True
        #print 'lines are parallel'
        return True
        
    #if (unit_v2 == unit_v1).all() or (unit_v2 == -unit_v1).all():
    #    ##If u_vectors are similar, lines should be parallel, return True
    #    print 'lines are parallel'
    #    return True
    else:
        ##Return False if u_vectors are not similar
        #print 'lines are NOT parallel'
        return False

def line_norm_distance(p1, p2, p3, p4, e=0):
    """
    Takes two sets of points (x, y) and a distance factor, returns True if the line norms are a length less than the value e
    p1 and p2 make first line
    p3 and p4 make second line 
    e is the minimum distance between the lines

    Lines are assumed parallel from the function above.
    """

    if (p2[0] - p1[0]) == 0:    # tests the x values from line 1. The line is vertical if there is no difference

        # The normal distance between the lines is the diference in x positions
        epsilon = p1[0] - p3[0]
        #print 'vertical line'
        #print 'epsilon', epsilon\

    elif (p2[1] - p1[1]) == 0:  # tests the y values from line 1. The line is horizontal if there is no difference

        # The normal distance between the lines is the diference in y positions
        epsilon = p1[1] - p3[1]
        #print 'horizpntal line'
        #print 'epsilon', epsilon

    else:                       # The lines are reformated in y = mx + b slope form.
        m1 = (p2[1] - p1[1])/(p2[0] - p1[0])    #slope of the parallel lines

        m_prime = -1/m1                         #slope, normal to the parallel lines
        b_prime = p1[1] - m_prime * p1[0]       #Y intercept of the normal line, found from intersecting point 1 from line 1

        m2 = m1                                 #slope of parallel lines
        b2 = p3[1] - m2 * p3[0]                 #Y intercept of line 2

        x = float((b_prime - b2) / (m2 - m_prime))      #Intersecting point of line 2 and the normal line.
        y = float(m_prime * x + b_prime)
        epsilon = np.sqrt((x - p1[0])**2 + (y - p1[1])**2)      #Distance along normal line from line 1 and 2
        #print 'diagonal line'
        #print 'epsilon', epsilon

    if abs(epsilon) <= abs(e):  # Quick test if calculated distance is with in the 'e' bounds set
        #print 'Distance is close enough'
        return True
    else:
        #print 'Distance is NOT close enough', epsilon - e
        return False

def point_in_line_point(p1, p2, p3, p4):
    """
    Takes two sets of points (x, y) and a distance factor, returns True if the lines intersect with each other
    p1 and p2 make first line
    p3 and p4 make second line 

    Lines are assumed to be on identical axis from the functions above.
    """

    if p1[0]-p2[0] == 0:        #Tests if lines are verticle, if so, 'y' values are used
        #print 'verticle lines'
        l1max = max(p1[1], p2[1])
        l1min = min(p1[1], p2[1])
        l2max = max(p3[1], p4[1])
        l2min = min(p3[1], p4[1])
    else:                       #If lines are horizontal or diagonal 'x' values are used
        #print 'otherwise'
        l1max = max(p1[0], p2[0])
        l1min = min(p1[0], p2[0])
        l2max = max(p3[0], p4[0])
        l2min = min(p3[0], p4[0])

    if l1min <= l2min < l1max:
        # Tests if lower point is within bounds
        #print 'l2min inbetween'
        return True
    elif l1min < l2max <= l1max:
        # Tests if upper point is within bounds
        #print 'l2max inbetween'
        return True

    elif l1min > l2min and l1max < l2max:
        # Tests if lower and upper points are surrounding the bounds
        #print 'l1min > l2min and l1max < l2max'
        return True
    else:
        # Returns False if points are not with in correct boundries
        #print 'close but no'
        return False

# == End Build Order ==
       

# === Geometric Classes & Functions ===
  
# == 2D Geometry ==

class Turn(object): # <<< resenv
    """ Represents a 2D rotation """
    
    def __init__(self, pTheta):
        """ Store the scalar and complex representations of the rotation """
        self._theta = pTheta
        self.complex = e ** ( 1j * self._theta ) # This has a magnitude of 1
    
    @property
    def theta(self):
        """ 'theta' is a property so that the complex and scalar representations are set at the same time """
        return self._theta
    
    @theta.setter
    def theta( self , pTheta ):
        """ 'theta' is a property so that the complex and scalar representations are set at the same time """
        self._theta = pTheta
        self.complex = e ** ( 1j * self._theta ) # This has a magnitude of 1
    
    def apply_to( self , vec ):
        """ Apply the rotation to the vector """
        temp = self.complex * ( vec[0] + vec[1] * 1j )
        return [ round_small( temp.real ) , round_small( temp.imag ) ]
        
    def __str__( self ):
        """ Return the string of the angle that the turn represents """
        return str( self._theta )

# = 2D Frames and Figures =

class Pose2D(object): # <<< resenv
    """ x , y , theta """
    
    def __init__( self , pos = [ 0.0 , 0.0 ] , theta = 0 ):
        """ Create a Pose from a 2D vector and an angle """
        self.position = vec_copy( pos )
        self.orientation = Turn( theta )
        
    def __str__( self ):
        """ String representation of a 2D pose """
        return "[ " + format_dec_list( self.position , places=4 ) + " , " + str( self.orientation.theta ) + " ]"
    
    # TODO: I have sum and difference here, why not define operators?
    
    @staticmethod
    def diff_from_to( aPose , bPose ):
        """ Return the difference from 'aPose' to 'bPose', assuming both are expressed in the same frame """
        return Pose2D(
            np.subtract( bPose.position , aPose.position ) ,  # Vector moves A onto B
            bPose.orientation.theta - aPose.orientation.theta # Theta turns A to B 
        )
        
    @staticmethod
    def diff_mag( aPose , bPose ):
        """ Return the linear and angular difference between Poses """
        #      Vector moves A onto B                                     , Theta turns A to B 
        return vec_mag( np.subtract( bPose.position , aPose.position ) ) , bPose.orientation.theta - aPose.orientation.theta  
        
    @staticmethod
    def sum_of( *poses ):
        """ Return a Pose2D that is the sum of the 'poses' , This is NOT the same as nested frames! """
        totPos = [ 0.0 , 0.0 ]
        totAng = 0
        for pose in poses:
            totPos = np.add( totPos , pose.position )
            totAng += pose.orientation.theta
        return Pose2D( totPos , totAng )
        
    @staticmethod
    def add( op1 , op2 ):
        """ Add all the associated pose components of 'op1' and 'op2' without respect to relative frames , return the result as a Pose2D """
        return Pose2D(  
            [ op1.position[0] + op2.position[0] , op1.position[1] + op2.position[1] ] , 
            op1.orientation._theta + op2.orientation._theta
        )
        
    @staticmethod
    def subtract( op1 , op2 ):
        """ Subtract all the associated pose components of 'op2' from 'op1' without respect to relative frames , return the result as a Pose2D """
        return Pose2D(  
            [ op1.position[0] - op2.position[0] , op1.position[1] - op2.position[1] ] , 
            op1.orientation._theta - op2.orientation._theta
        )
    
    # TODO: See if I can use the following three functions to construct 'Frame2d.lab_to_local_Pose' since they are basically copies
    
    def to_bases( self ):
        """ Treat this Pose2D as the origin of a reference frame and get the basis vectors """
        return { 'origin': self.position , 
                 'xBasis': polr_2_cart( [ 1 , self.orientation._theta ] ) , 
                 'yBasis': polr_2_cart( [ 1 , pi/2 + self.orientation._theta ] ) }
    
    def relative_pos( self , vec ):
        """ Express 'vec' as a relative position , as though it were in a frame with this Pose as the origin """        
        bases = self.to_bases()
        localOffset = np.subtract( vec , self.position )
        return [ vec_proj( localOffset , bases['xBasis'] ) , vec_proj( localOffset , bases['yBasis'] ) ]
        
    def relative_Pose( self , otherPose ):
        """ Assuming that this pose is the origin, express 'otherPose' as a relative pose , as though it were a child frame """
        return Pose2D(
            self.relative_pos( otherPose.position ) ,
            otherPose.orientation._theta - self.orientation._theta
        )
        
    def local_to_parent( self , localPose ):
        """ Assuming 'localPose' is expressed with respect to this pose as the origin, express it in the same frame as this pose """
        relative = self.orientation.apply_to( localPose.position )
        return Pose2D(
            np.add( self.position , relative )  ,
            localPose.orientation._theta + self.orientation._theta
        )
        
    def get_copy( self ):
        """ Return a copy of this Pose2D """
        return Pose2D( self.position , self.orientation._theta )
        
    def serialize( self ):
        """ Flatten the Pose2D to a list q = [ x , y , theta ] and return """
        return [ self.position[0] , self.position[1] , self.orientation.theta ]
        
    @staticmethod
    def deserialize( qList ):
        """ Return a Pose2D from a list q = [ x , y , theta ] """
        return Pose2D( qList[:2] , qList[2] )
        
    def copy_round_small( self ):
        """ Return a version of the Pose with all the near-zero components rounded to 0 """
        return Pose2D( vec_round_small( self.position ) , round_small( self.orientation._theta ) )
        
    @staticmethod
    def angle_cmp( op1 , op2 ):
        """ Compare the orientations of 'op1' and 'op2' """
        if op1.orientation._theta < op2.orientation._theta:
            return -1
        elif op1.orientation._theta > op2.orientation._theta:
            return 1
        else:
            return 0

# - Class Frame2D - 
       
class Frame2D( Pose2D ): 
    """ A Frame is a container for geometric objects, it is defined by a Pose that is relative to the parent Frame 
    
    self.position: ---------- position in the parent frame , a position added to upstream frames
    self.orientation: ------- orientation in the parent frame , an orientation composed with upstream frames
    self.labPose.position: -- position in the lab frame , the summation of this and all upstream frames
    self.labPose.orientation: orientation in the lab frame , the composition of this and all upstream frames
    """
    
    def __init__( self , pPos , pTheta , *containedObjs ):
        super( Frame2D , self ).__init__( pPos , pTheta ) # This object inherits Pose2D, and uses position and orientation to define its state
        self.labPose = Pose2D() # The lab pose: position and orientation in the lab frame
        self.ntnPose = Pose2D() # The notional pose: Used to make "what-if" calculations without creating a new object or disturbing present coords
        self.objs = list(containedObjs) if containedObjs else [] # List of contained drawable objects
        self.subFrames = [] # List of sub-frames, frames contained by this frame, moving this frame will move all sub-frames
        self.parent = None  # Reference to the parent frame
        
    def attach_sub( self , subFrm ):
        """ Attach a subframe to this frame with appropriate connections """
        subFrm.parent = self # A frame can have only one parent
        self.subFrames.append( subFrm ) # Add 'subFrm' to the list of subframes
        
    def remove_sub( self , subFrm ):
        """ Remove the subframe with the 'id' matching 'subFrm' if it exists , return success """
        try: # If the subframe exists , remove it , return success
            self.subFrames.remove( subFrm )
            return True
        except Exception: # else does not exist in the list , do nothing , return failure
            return False
        
    def recursive_add_sub( self , *subFrames ):
        """ Add subframes or list of subframes to the simulation frame, recursively """
        for elem in subFrames:
            if isinstance( elem , list ):
                self.recursive_add_sub( *elem )
            else:
                self.attach_sub( elem )
        
    def transform_objects( self ):
        """ Dummy transform """
        pass
    
    # TODO: CREATE A FUNCTION THAT DOES ITS OWN TRANSFORMATION IN ADDITION TO THE ONE DONE BY THE FRAME , NO ARGS, WORKS ON ITSELF
    
    def transform_contents(self, upPosition = None, upOrientation = None):
        """ Transform the coordinates of contained geometric objects and request sub-Frames to transform """
        
        # Set the orientation that will rotate the contents to the lab frame
        if upOrientation != None: # If an orientation was passed from upstream, then compose self orientation with it
            self.labPose.orientation = Turn( self.orientation._theta + upOrientation.theta )
        else: # else there is no parent frame, self orientation is the only one that influences coords
            self.labPose.orientation = self.orientation        
         
        # Using the above orientation, rotate the Frame origin in the lab frame
        if isinstance( upPosition , ( list , np.ndarray ) ): # If coordinates passed from upstream are not 'None', add them to all containing objects
            self.labPose.position = np.add( vec_copy( upPosition ) , upOrientation.apply_to(self.position) ) # Calc the position in the labe frame, given the above
        else: # else there is no parent frame, objects are expressed in this frame only
            self.labPose.position = vec_copy( self.position ) # Calc the position in the labe frame, given the above
            
        for obj in self.objs: # For each of the contained objects
            for pntDex , point in enumerate(obj.coords): # For each point in the contained object
                # 1. Transform relative coordinates to lab coordinates
                obj.labCoords[pntDex] = np.add( self.labPose.position , self.labPose.orientation.apply_to( point ) ) 
                
        # 2. Apply an special transformations for representing the obj
                
        if self.objs:
            self.transform_objects() # TODO: See if this targets the right function
                    
        for frame in self.subFrames: # for each of the sub-frames, recursively transform
            frame.transform_contents( self.labPose.position , self.labPose.orientation )
            
    def transform_notional( self , upPosition = None , upOrientation = None , depth = 0 ):
        """ Transform the coordinates of contained geometric objects and request sub-Frames to transform """
        
        if depth == 0 and upPosition != None and upOrientation != None:
            self.ntnPose = Pose2D( upPosition , upOrientation._theta )
        else:
        
            # Set the orientation that will rotate the contents to the lab frame
            if upOrientation != None: # If an orientation was passed from upstream, then compose self orientation with it
                self.ntnPose.orientation = Turn( self.orientation._theta + upOrientation.theta )
            else: # else there is no parent frame, self orientation is the only one that influences coords
                self.ntnPose.orientation = Turn( self.orientation._theta )        
             
            # Using the above orientation, rotate the Frame origin in the lab frame
            if isinstance( upPosition , ( list , np.ndarray ) ): # If coordinates passed from upstream are not 'None', add them to all containing objects
                self.ntnPose.position = np.add( vec_copy( upPosition ) , upOrientation.apply_to( self.position ) ) # Calc the position in the labe frame, given the above
            else: # else there is no parent frame, objects are expressed in this frame only
                self.ntnPose.position = vec_copy( self.position ) # Calc the position in the labe frame, given the above
    
        for frame in self.subFrames: # for each of the sub-frames, recursively transform
            frame.transform_notional( self.ntnPose.position , self.ntnPose.orientation , depth + 1 )

    def get_Pose( self , frame = 'rel' ):
        """ Return a Pose2D representing the pose in the specified frame , copy of Pose2d """
        if frame == 'rel':
            return Pose2D( self.position , self.orientation._theta )
        elif frame == 'lab':
            return self.labPose.get_copy()
        elif frame == 'ntn':
            return self.ntnPose.get_copy()
        else:
            raise ValueError( "Frame2D.get_Pose: " + str(frame) + " is not a recognized frame!" )
            
    def get_lab_Pose( self ): # TODO: Left this in so that other code does not break , REMOVE LATER!
        """ Return a copy of the lab pose of this frame """
        return self.labPose.get_copy()
    
    def local_to_lab( self , vec ):
        """ Express a 'vec' specified in the local frame in the lab frame """
        labRelative = self.labPose.orientation.apply_to( vec ) # This function assumes that at least one update has occurred
        return np.add( self.labPose.position , labRelative )
        
    def pts_to_lab( self , ptsLst ):
        rtnPts = []
        for pnt in ptsLst:
            rtnPts.append( self.local_to_lab( pnt ) )
        return rtnPts
        
    def get_lab_bases( self ):
        """ Return the origin and the basis vectors if this frame """
        return { 'origin': self.labPose.position , 
                 'xBasis': np.subtract( self.local_to_lab( [ 1 , 0 ] ) , self.labPose.position ) , 
                 'yBasis': np.subtract( self.local_to_lab( [ 0 , 1 ] ) , self.labPose.position ) }
        # return rtnDict
        
    def lab_to_local( self , vec ):
        """ Express a 'vec' specified in the lab frame in the local frame """
        bases = self.get_lab_bases()
        localOffset = np.subtract( vec , self.labPose.position )
        return [ vec_proj( localOffset , bases['xBasis'] ) , vec_proj( localOffset , bases['yBasis'] ) ]
        
    def lab_to_local_Pose( self , pLabPose ):
        """ Express a lab pose in the local frame """
        return Pose2D(
            self.lab_to_local( pLabPose.position ) ,
            pLabPose.orientation.theta - self.labPose.orientation.theta
        )
    
    def local_to_lab_Pose( self , localPose ):
        """ Return a Pose2D in the lab frame that is offset from this Frame2D by offsetPose in the local frame """
        labRelative = self.labPose.orientation.apply_to( localPose.position ) # This function assumes that at least one update has occurred
        return Pose2D( np.add( self.labPose.position , labRelative ) , 
                       localPose.orientation.theta + self.labPose.orientation.theta )
    
    # ~ Set Pose ~
       
    def set_pos( self , pos , frame = "rel" ):
        """ Set the center of the polygon, relative to the containing frame """
        if frame == "rel":
            self.position = vec_copy( pos )
        elif frame == "ntn":
            self.ntnPose.position = vec_copy( pos )
        elif frame == "lab": # Probably should not use this one
            self.labPose.position = vec_copy( pos )
        else:
            raise ValueError( "Frame2D.set_pos: " + str(frame) + " does not designate a known frame!")
	
    def set_theta( self , pTheta , frame = "rel" ):
        """ Set the orientation of the polygon, relative to the containing frame """
        if frame == "rel":
            self.orientation.theta = pTheta # This causes the Turn to recalc
        elif frame == "ntn":
            self.ntnPose.orientation.theta = pTheta # This causes the Turn to recalc
        elif frame == "lab": # Probably should not use this one, it will just be overwritten for an update
            self.labPose.orientation.theta = pTheta # This causes the Turn to recalc
        else:
            raise ValueError( "Frame2D.set_theta: " + str(frame) + " does not designate a known frame!")
    
    def update_config( self , qList ):
        """ Update the frame like it was a robot , 'qList' takes the form [ X , Y , THETA ] , this is for RRT and the like """ 
        # NOTE: This function assumes that 'qList' is of the correct length
        # print "DEBUG , q:" , qList
        self.position = qList[:2]
        self.orientation.theta = qList[2] # This causes the Turn to recalc
        self.transform_contents() # Make sure all the constituents are in the right place
    
    def set_Pose( self , pPose , frame = "rel" ):
        """ Make the pose of this Frame2D equal to the input Pose2D """
        self.set_pos( pPose.position , frame )
        self.set_theta( pPose.orientation._theta , frame )
        
    def set_Pose_same( self , otherFrame ):
        """ Set the pose of this frame to one identical to the 'otherFrame' """
        self.set_pos( otherFrame.position )
        self.set_theta( otherFrame.orientation._theta )
        
    # ~ Offset Pose ~
        
    def move( self , vec ):
        """ Move the center of the polygon, relative to the containing frame """
        self.position = np.add( self.position , vec_copy( vec ) )
	
    def rotate( self , pTheta ):
        """ Set the orientation of the polygon, relative to the containing frame """
        self.orientation.theta = self.orientation.theta + pTheta  # This causes the Turn to recalc

    # ~ Drawing ~

    def colorize(self, pColor):
        """ Color all Segments 'pColor' """ # NOTE: This function assumes that a canvas has already been assigned
        for vecDex , vec in enumerate( self.objs ):
            self.objs[vecDex].set_color( pColor )
            
    # ~~ SubFrame Info ~~
            
    # ~ Subframe Lab ~            
            
    def vertex_set_lab_old( self ):
        """ Return a set of all vertices of all contained geometric objects """ # Using a set because some members may share vertices
        allVerts = set( [] ) # The set of all vertices of all contained objs
        
        for obj in self.objs: # For each of the contained objects
            for vert in obj.labCoords: # For each of the lab coords of the object
                allVerts.add( tuple( vert ) ) # Collect the coordinate , tuplize it , and discard repeats
        
        for frame in self.subFrames: # For each of the contained frames , collect all the successor frame vertices
            allVerts = allVerts.union( frame.vertex_set_lab() )
            
        return allVerts
        
    def vertex_set_lab( self , depth = 0 , activeOnly = False ):
        """ Return a set of all vertices of all contained geometric objects """
        allVerts = set( [] ) # The set of all vertices of all contained objs
        
        if depth == 0:
            # self.set_Pose( Pose2D( [0,0] , 0 ) , frame="ntn" )
            self.transform_contents() # Assign notional coords to this and all below
        
        # TODO: Reaching into the subclass feels weird and this is probably a sign to move 'points' to Frame2D
        if hasattr( self , 'points' ): 
            if activeOnly:
                if self.collActive:
                    for vert in self.labPts: # For each of the lab coords of the object
                        allVerts.add( tuple( vert ) ) # Collect the coordinate , tuplize it , and discard repeats
                # else this poly is not active , do not collect its points
            else:
                for vert in self.ntnPts: # For each of the lab coords of the object
                    allVerts.add( tuple( vert ) ) # Collect the coordinate , tuplize it , and discard repeats
        
        for frame in self.subFrames: # For each of the contained frames , collect all the successor frame vertices
            allVerts = allVerts.union( frame.vertex_set_lab( depth + 1 , activeOnly ) )
            
        return allVerts
            
    def hull_of_contents_lab( self ):
        """ Return the convex hull of all the contained geometric objects based on their vertices """
        
        containedPts = list( self.vertex_set_lab() ) # Get all the unique vertices from this frame
        hullPts = []
                
        try: # Scipy ConvexHull sometimes throws random, unexplained exceptions, catch them with a 'try'
            frameHull = ConvexHull( containedPts ) 
            for pDex in frameHull.vertices:
               hullPts.append( frameHull.points[ pDex ] ) 
        except Exception as ex: # URL, generic excpetions: http://stackoverflow.com/a/9824050
            print "Encountered" , type( ex ).__name__  ,  "with args:" , ex.args , "with full text:" , str( ex )
            
        return hullPts # Will return an empty list if there was an error creating the hull
    
    def contained_centroid_lab( self , centroids = [] , areas = [] ):
        """ Get the centoid of all the objects contained by this frame , expressed in the lab frame """
        
        try: # If this frame is a Poly2D, get its centroid in the lab frame
            cntrd , area = self.get_lab_centroid( wArea = True )
            centroids.append( cntrd ) ; areas.append( area )
        except Exception: # else this is not a Poly2D, ex: Design, do nothing and recur
            pass
        
        for frame in self.subFrames:
            frame.contained_centroid_lab( centroids , areas ) 
           
        totalMass = sum( areas ) # Calc the total area of all the contained components
        massCenters = zip( areas , centroids ) # Collect the centroids in ( MASS[i] , CENTROID[i] ) pairs
        
        return centroid_discrete_masses( massCenters , totalMass )
        
    # ~ Subframe Relative ~
    
    def vertex_set_rel( self , depth = 0 , activeOnly = False ):
        """ Return a set of all vertices of all contained geometric objects """
        allVerts = set( [] ) # The set of all vertices of all contained objs
        
        if depth == 0:
            # self.set_Pose( Pose2D( [0,0] , 0 ) , frame="ntn" )
            self.transform_notional( upPosition = [0,0] , upOrientation = Turn( 0.0 ) ) # Assign notional coords to this and all below
        
        # TODO: Reaching into the subclass feels weird and this is probably a sign to move 'points' to Frame2D
        if hasattr( self , 'points' ): 
            if activeOnly:
                if self.collActive:
                    for vert in self.ntnPts: # For each of the lab coords of the object
                        allVerts.add( tuple( vert ) ) # Collect the coordinate , tuplize it , and discard repeats
                # else this poly is not active , do not collect its points
            else:
                for vert in self.ntnPts: # For each of the lab coords of the object
                    allVerts.add( tuple( vert ) ) # Collect the coordinate , tuplize it , and discard repeats
        
        for frame in self.subFrames: # For each of the contained frames , collect all the successor frame vertices
            allVerts = allVerts.union( frame.vertex_set_rel( depth + 1 , activeOnly ) )
            
        return allVerts
        
    def hull_of_contents_rel( self , activeOnly = False ):
        """ Return the convex hull of all the contained geometric objects based on their vertices """
        
        containedPts = list( self.vertex_set_rel( activeOnly = activeOnly ) ) # Get all the unique vertices from this frame
        hullPts = []
                
        try: # Scipy ConvexHull sometimes throws random, unexplained exceptions, catch them with a 'try'
            frameHull = ConvexHull( containedPts ) 
            for pDex in frameHull.vertices:
               hullPts.append( frameHull.points[ pDex ] ) 
        except Exception as ex: # URL, generic excpetions: http://stackoverflow.com/a/9824050
            print "Encountered" , type( ex ).__name__  ,  "with args:" , ex.args , "with full text:" , str( ex )
            
        return hullPts # Will return an empty list if there was an error creating the hull
        
    def contained_centroid_rel( self , centroids = [] , areas = [] , depth = 0 , activeOnly = False ):
        """ Get the centoid of all the objects contained by this frame , expressed in the lab frame """
        
        if depth == 0:
            self.transform_notional( upPosition = [0,0] , upOrientation = Turn( 0.0 ) ) # Assign notional coords to this and all below
            
        try: # If this frame is a Poly2D, get its centroid in the lab frame
            if activeOnly:
                if self.collActive:
                    area , cntrd = self.get_area_centroid_ntn()
                    centroids.append( cntrd ) ; areas.append( area )
                # else we do not want to count an inactive shape
            else:
                area , cntrd = self.get_area_centroid_ntn()
                centroids.append( cntrd ) ; areas.append( area )
        except Exception: # else this is not a Poly2D, ex: Design, do nothing and recur
            # print "Could not obtain centroid for this" , self.__class__.__name__
            # print "Encountered" , type( ex ).__name__  ,  "with args:" , ex.args , "and with full text:" , endl , str( ex )
            pass # If this is not a Poly2D , ignore and continue
        
        for frame in self.subFrames:
            frame.contained_centroid_rel( centroids , areas , depth + 1 , activeOnly ) # This will automatically accumulate 'centroids' and 'areas'
           
        totalMass = sum( areas ) # Calc the total area of all the contained components
        massCenters = zip( areas , centroids ) # Collect the centroids in ( MASS[i] , CENTROID[i] ) pairs
        # print "DEBUG" , totalMass
        # print "DEBUG" , massCenters
        # print "DEBUG" , centroid_discrete_masses( massCenters , totalMass )
        
        if len( massCenters ) > 0:
            return vec_round_small( centroid_discrete_masses( massCenters , totalMass ) )
        else:
            print "Frame2D.contained_centroid_rel: This frame contains no polygons for centroid calculation"
            return None
        
    def set_supports_for_hull( self , activeOnly = True ):
        # self.transform_contents() # Assume this has already been done
        """ Load this frame with relative support information """
        self.relCentroid = self.contained_centroid_rel( activeOnly = activeOnly ) # Get the relative centroid for this collection of shapes
        print "DEBUG , found the centroid" , self.relCentroid
        self.allVerts = self.vertex_set_rel( activeOnly = activeOnly )
        print "DEBUG , found the vertices" , self.allVerts
        self.hullVerts = self.hull_of_contents_rel( activeOnly ) # Calc the convex hull of all the contained parts
        print "DEBUG , Hull has the vertices" , self.hullVerts
        self.supports = sides_from_vertices( self.hullVerts )
        print "DEBUG , found the suports" , self.supports 
        self.stabilities = planar_poly_stability( self.hullVerts , self.relCentroid )
        print "DEBUG , determined stabilties" , self.stabilities
        # self.hullPutdowns = [ support_to_Pose( supportSeg ).copy_round_small() for supportSeg in self.supports ] # This will create a pose whether it is stable or not
        self.hullPutdowns = [ support_to_Pose( supportSeg ) for supportSeg in self.supports ] # This will create a pose whether it is stable or not
        print "DEBUG , found the poses" , ; print_list( self.hullPutdowns )
        self.sort_putdowns() # This will sort all the appropriate lists in tandem!
    
    def sort_putdowns( self ):
        """ Sort the putdown poses by angle , in place , supports and stability measures are sorted accordingly , return the angles I guess """
        putdownAngles = [ pd.orientation._theta for pd in self.hullPutdowns ] # Create a list of putdown angles to sort by
        [ sortAngles , self.supports , self.stabilities , self.hullPutdowns ] = tandem_sorted( putdownAngles , self.supports , 
                                                                                               self.stabilities , self.hullPutdowns )
        return sortAngles # In case you really just wanted to see it
    
    def copy_hull_putdowns( self ):
        """ Return a list of copies of all the putdown poses """
        rtnList = []
        for hpd in self.hullPutdowns:
            rtnList.append( hpd.get_copy() )
        return rtnList
    
    def get_Pol2D_from_hull_rel( self ):
        """ Return a Poly2D object that represents the convex hull of the frame and all of its contents , relative """
        return Poly2D( self.position , self.orientation._theta , self.hull_of_contents_rel() )
        
    def get_Pol2D_from_hull_lab( self ):
        """ Return a Poly2D object that represents the convex hull of the frame and all of its contents , lab frame """
        # NOTE: This function assumes that the 
        return Poly2D( self.labPose.position , self.labPose.orientation._theta , self.hull_of_contents_lab() )        
        
    def ray_collide_lab( self , ray ):
        """ Return True if this ray collides with the frame or any of its contents """
        for frame in self.subFrames: # Assuming that this frame itself does not represent an object, check all the subframes
            if frame.ray_collide_lab( ray ):
                return True # If any subframe collides, then return True
        return False # else all subframes were searched without collision, return False
        
    def ray_nearest_exterior_clsn_dist_lab( self , ray ):
        """ Return the nearest distance between the 'ray' origin and any collision with the exterior of the polygon collection , return None if there is no collision """
        clsnDist = [] 
        for frame in self.subFrames:
            subDist = frame.ray_nearest_exterior_clsn_dist_lab( ray )
            if subDist != None:
                clsnDist.append( subDist ) 
        if len( clsnDist ) > 0:
            return min( clsnDist )
        else:
            return None
        
# - End Frame2D -        
        
class CollideList( list ):
    """ Container for Poly2Ds , helper class for SimFrame , Toggle collision detection """
    # NOTE: This is not strictly required for collision detection, but could possibly help if there are groups of objects to collide
    
    def __init__( self , *args ):
        """ Normal 'list' init """
        list.__init__( self , *args )
        self.active = True
        
    def attach_members_to_sim( self , simFrame ):
        """ Add all of the members to a SimFrame2D """ # NOTE: 'simFrame' must be of type SimFrame2D
        for elem in self:
            simFrame.collideList.append( elem )
        
    def set_active( self , active ):
        """ Activate (True) or deactivate (False) collision detection"""
        self.active = active
        for elem in self:
            elem.collActive = self.active        
        
    def toggle_active( self ):
        """ Toggle whether or not this list will be checked for collisions """
        self.set_active( not self.active )

# Breaking collision checking out on its own so that the list of collisions is easier to manage

def check_poly_collisions( collideList ):
    """ Check for collisions in a hierarchical search, first a plane sweep across bounding boxes, then contained points, then crossed segments """
    # NOTE: This function assumes that all Frame2Ds with a 'bbox' member are meant to be checked for collisions
    bb_sorter = PriorityQueue()
    for poly in collideList: # Push polys onto priority queue so that we can sort on the left-bound
        if poly.collActive: # Only consider active parts
            bb_sorter.push( poly , poly.bbox[0][0] )
    polys , leftCoords = bb_sorter.unspool() # Fetch two lists: polys sorted increasing by left bound and associated left bounds
    activeX = set([]) # active bounding boxes for the sweep
    bbClldPairs = set([]) # colliding pairs of bounding boxes , always add (min,max) so that no pair is added more than once
    realCollisions = [] # List holding pairs of colliding objects
    for lftDex , lftCrd in enumerate( leftCoords ): # For each left bounding box coordinate, in increasing order
        remv = [] # init indices to remove for this step
        activeX.add( lftDex ) # Add the current index
        for actv in activeX: # For each actrive bbox
            if lftDex != actv: # If the bbox is other than current
                if polys[actv].bbox[1][0] < leftCoords[lftDex]: # if the current leading edge is greater than the active trailing edge
                    remv.append( actv ) # Deactivate bbox
                else: # else the bbox is correctly active and is in x-collision with current, check for y-collision
                    # There are four checks to make, if top or bottom of one bbox falls within the bbox of the other, only one need be true
                    # 1. actv top falls within lftDex
                    if polys[lftDex].bbox[1][1] >= polys[actv].bbox[1][1] and polys[actv].bbox[1][1] >= polys[lftDex].bbox[0][1]:
                        bbClldPairs.add( ( min( lftDex , actv ) , max( lftDex , actv ) ) )
                    # 2. actv bottom falls within lftDex
                    elif polys[lftDex].bbox[1][1] >= polys[actv].bbox[0][1] and polys[actv].bbox[0][1] >= polys[lftDex].bbox[0][1]:
                        bbClldPairs.add( ( min( lftDex , actv ) , max( lftDex , actv ) ) )
                    # 3. lftDex top falls within actv
                    elif polys[actv].bbox[1][1] >= polys[lftDex].bbox[1][1] and polys[lftDex].bbox[1][1] >= polys[actv].bbox[0][1]:
                        bbClldPairs.add( ( min( lftDex , actv ) , max( lftDex , actv ) ) )
                    # 4. lftDex bottom falls within actv
                    elif polys[actv].bbox[1][1] >= polys[lftDex].bbox[0][1] and polys[lftDex].bbox[0][1] >= polys[actv].bbox[0][1]:
                        bbClldPairs.add( ( min( lftDex , actv ) , max( lftDex , actv ) ) )
                    # else there is no bbox collision, no action
        for rem in remv: # Now remove all the bboxs that have become inactive so we don't worry about them next iteration
            activeX.remove( rem )
    
    for pair in bbClldPairs: # Iterate over bbox collisions and find out if the polys actually collide
        if polys[ pair[0] ].check_collision_with( polys[ pair[1] ] ):
            realCollisions.append( [ polys[ pair[0] ] , polys[ pair[1] ] ] ) # package pairs of poly references for the sim to operate on
    return realCollisions        

def check_target_in_collision( target , collideList ):
    """ Return True if 'target' is in collision with any object in 'collideList' , otherwise return False """
    # NOTE: This function assumes that 'target' is an element of 'collideList'
    collisions = check_poly_collisions( collideList ) # run the collision check on all the elements of the list
    # print collisions
    for collision in collisions: # for each of the collisions returned
        if target in collision: # if the target is a participant in a collision, then return true
            return True
    return False # else there were no collisions or the target was not a participant, return False
            
# - Class SimFrame2D -

class SimFrame2D(Frame2D): 
    """ 'Frame2D' to manage simulations of 2D subframes, including collision detection """
    
    def __init__( self , bboxBounds , *subFrames ):
        """ Init sim with subframes as args or lists """
        super(SimFrame2D, self).__init__( [ 0.0 , 0.0 ] , 0.0 ) # Simulation is not relative to anything else, this is the lab frame
        self.bb_sorter = PriorityQueue() # Used to automatically sort bounding boxes
        self.recursive_add_sub( *subFrames )
        self.bbox = bboxBounds # The area of the frame we care about # Has the format [ [ x_min , y_min ] , [ x_max , y_max ] ] 
        self.boundRays = [ [ self.bbox[0] , [ self.bbox[0][0] , self.bbox[1][1] ] ] , # btm-lft to top-lft
                           [ self.bbox[0] , [ self.bbox[1][0] , self.bbox[0][1] ] ] , # btm-lft to btm-rgt
                           [ self.bbox[1] , [ self.bbox[0][0] , self.bbox[1][1] ] ] , # top-rgt to top-lft
                           [ self.bbox[1] , [ self.bbox[1][0] , self.bbox[0][1] ] ] ] # top-rgt to btm-rgt
        self.collideList = []

# - End SimFrame2D -

# - Class Poly2D -

# * Polygon Helpers *

def sides_from_vertices( vertList ):
    """ Return a list of line segments from the ordered 'vertList' , consisting of each consecutive pair in a cycle with CW/CCW preserved """
    segments = [] # List of segments to be returned 
    for pDex in xrange( len( vertList ) ):
        segments.append( [ elemw( vertList , pDex ) , elemw( vertList , pDex + 1 ) ] )
    return segments

# * End Helpers *
    
# == class Segment ==

class Segment( object ): 
    """ A line segment to be displayed on a Tkinter canvas """ # TODO: * Generalize for any display?
#                                                                      * Consider extending this class for particular displays?

    def __init__(self , pCoords = None , TKcanvas=None , color=None): # candidate super signature
        """ Assign vars and conditionally create the canvas object 'self.drawHandle' """
        self.transform = self.dummy_transform # Optionally change this for a different rendering engine
        self.display_xform = self.scaled_coords_to_list
        self.displayScale = 1 # /4.0
        #                    v--- Need to make a copy here, otherwise relative coords will be transformed
        self.coords = vec_copy_deep( pCoords ) if pCoords else [ [0.0 , 0.0] , [0.0 , 0.0] ] # Coordinates expressed in the parent reference Frame
        # Coordinates expressed in the lab frame, used for drawing and lab frame calcs
        self.labCoords = vec_copy_deep( pCoords ) if pCoords else [ [0.0 , 0.0] , [0.0 , 0.0] ] 
        self.canvas = None
        if TKcanvas: # If canvas is available at instantiation, go ahead and create the widget
            self.canvas = TKcanvas
            self.drawHandle = TKcanvas.create_line( self.coords[0][0] , self.coords[0][1] , self.coords[1][0] , self.coords[1][1])
            if color:
                self.canvas.itemconfig(self.drawHandle,fill=color)
        self.hidden = False

    def __str__(self):
        """ Return the endpoints of the Segment as a String """
        return str( self.coords )

    # = Drawing Methods =
    
    def old_dummy_transform(self, pntList, scale): # candidate super fuction
        """ Dummy coordinate transformation, to be replaced with whatever the application calls for """
        print "OLD DUMMY CALLED , FIND OUT WHY!"
        rtnList = []
        for pnt in pntList:
            rtnList.extend( pnt )
        return rtnList # no actual transformation done to coords
        
    def scaled_coords_to_list( self , coords , scale = 1 ):
        """ Unravel lab coord pairs as [ x_1 , y_1 , ... , x_n , y_n ] """
        rtnList = []
        for pair in coords:
            rtnList.extend( np.multiply( pair , scale ) ) # FIXME: Scaling this twice seems redundant
        return rtnList 
        
    def dummy_transform(self, pntList, scale): # candidate super fuction
        """ Dummy coordinate transformation, to be replaced with whatever the application calls for """
        return pntList 
    
    def set_color(self, color): # candidate super functions
        """ Set the 'color' of the line """
        self.canvas.itemconfig(self.drawHandle,fill=color)
        self.canvas.itemconfig(self.drawHandle, width=3)
        
    def attach_to_canvas( self , TKcanvas ): # candidate super function
        """ Given a 'TKcanvas', create the graphics widget and attach it to the that canvas """
        self.drawHandle = TKcanvas.create_line( -10 , -10 , -5 , -5 ) # Init to dummy coords x1, y1 , x2 , y2
        self.canvas = TKcanvas
            
    def update(self): # candidate super function
        """ Update the position of the segment on the canvas """
        temp1 = self.transform( self.labCoords, self.displayScale ) # Project 3D to 2D
        temp2 = self.display_xform( temp1 , 1 )
        self.canvas.coords( self.drawHandle , *temp2 ) 
        
    def set_hidden( self , hidden ):
        """ Set this line Segment hidden (True) or visible (False) """
        self.hidden = hidden
        if hidden:
            self.canvas.itemconfig( self.drawHandle , state=Tkinter.HIDDEN )
        else:
            self.canvas.itemconfig( self.drawHandle , state=Tkinter.NORMAL )

    # = End Drawing =

# == End Segment ==
                    
class Poly2D( Frame2D ): 
    """ Represents a closed polygon in 2 dimensions, not necessarily convex or simple """
    
    def __init__( self , pPos , pTheta , *pointsCCW ):
        super(Poly2D, self).__init__( pPos , pTheta )
        # print "Poly2D got points" , pointsCCW
        self.points = [ vec_copy( pnt ) for pnt in pointsCCW ] # Enforce: Points must be on the local X-Y plane
        self.labPts = [ vec_copy( pnt ) for pnt in pointsCCW ]
        self.ntnPts = [ vec_copy( pnt ) for pnt in pointsCCW ]
        self.bbox = [ [ infty , infty ] , [ -infty , -infty ] ]
        for pntDex in range( len( self.points ) ): # For each point, create a segment from this point to the next, closed figure
            self.objs.append( Segment( [ elemw( self.points , pntDex ) , elemw( self.points , pntDex + 1 ) ] ) )
            # TODO: Look at the graphics book to see how polygons and polyhedra are represented
        self.collActive = True # Flag for whether this polygon should be considered for collisions
        self.hidden = False
        self.name = None
        self.relCentroid = None # Init centroid to the part origin, overwrite if we care about centroids
        self.area = 0 # Init to massless poly, no area
        
    def get_sides( self , frame = "lab" ):
        """ Return all the line segments for this polygon , assuming CCW order , in the lab , relative , or notional frame """
        if frame == "lab": # Assign the points according the the reference frame chosen by the client code
            points = self.labPts
        elif frame == "rel":
            points = self.points
        elif frame == "ntn":
            points = self.ntnPts
        else:
            raise ValueError( "Poly2D.get_sides: '" + str( frame ) + "' is not a recognized rerfernce frame!" )
        return sides_from_vertices( points )
    
    def get_points( self , frame = "lab" ):
        """ Return all the line points for this polygon , in the lab , relative , or notional frame """
        if frame == "lab": # Assign the points according the the reference frame chosen by the client code
            return self.labPts
        elif frame == "rel":
            return self.points
        elif frame == "ntn":
            return self.ntnPts
        else:
            raise ValueError( "Poly2D.get_points: '" + str( frame ) + "' is not a recognized rerfernce frame!" )
        
    def calc_bbox( self ):
        """ Set the bounding box of the polygon in the lab frame """
        self.bbox = [ [ infty , infty ] , [ -infty , -infty ] ] # NOTE: This function assumes that lab points have been calc'd
        for pnt in self.labPts:
            # print self.bbox[0][0] , pnt[0]
            self.bbox[0][0] = min( self.bbox[0][0] , pnt[0] )
            self.bbox[0][1] = min( self.bbox[0][1] , pnt[1] )
            self.bbox[1][0] = max( self.bbox[1][0] , pnt[0] )
            self.bbox[1][1] = max( self.bbox[1][1] , pnt[1] )
    
    def transform_objects( self ):
        """ Fetch the transformed Segment coords and calc bbox """
        for segDex , sgmnt in enumerate(self.objs):
            self.labPts[segDex] = vec_copy( sgmnt.labCoords[0] ) # Take advantage of the transformation already performed , copy for convenience
        self.calc_bbox() # recalc the bounding box for collision checking
    
    # TODO: The frame -> segment -> frame flow of information for drawable transformation now seems a bit cicuitous given the flow of information
    #       in 'transform_notional'. In the future this should be normalized so that frame points are managed by the frame and not the segments
    #       that the points define. In this case the transform_contents of Poly2D should assume the duties of 'transform_objects'
    
    def transform_notional( self , upPosition = None , upOrientation = None , depth = 0 ):
        """ Apply the notional transformation to this frame and to the collection of points """
        Frame2D.transform_notional( self , upPosition , upOrientation , depth ) # Transform the frame
        for pDex , point in enumerate( self.points ): # Transform the points
            self.ntnPts[ pDex ] = np.add( self.ntnPose.position , self.ntnPose.orientation.apply_to( point ) )
     
    @staticmethod
    def regular( nSides , radius , pPos , pTheta ):
        """ Create an 'nSides'-gon with distance 'radius' from center to each vertex, at specified position and angle """
        return Poly2D( pPos , pTheta , *circ_spacing( radius * 2 , nSides ) )
        
    @staticmethod
    def rectangle( axis1 , axis2 , pPos , pTheta , offset = [0,0] ):
        """ Create a rectangle with side lenghts 'axis1' and 'axis2', move origin off-center by 'offset' """ 
        # NOTE: Rectangle not axis-aligned unless user specifies
        return Poly2D( pPos , pTheta , [  axis1/2 + offset[0] ,  axis2/2 + offset[1] ] , 
                                       [ -axis1/2 + offset[0] ,  axis2/2 + offset[1] ] , 
                                       [ -axis1/2 + offset[0] , -axis2/2 + offset[1] ] , 
                                       [  axis1/2 + offset[0] , -axis2/2 + offset[1] ] )
        
    def check_collision_with( self , other ):
        """ Return true if 'other' collides with 'self', otherwise return false """
        # The polys collide if any vertex of one polygon is interior to the other polygon, or if any of the segments of one cross any of the other
        # NOTE: This function assumes lab poses are calc'd
        for lPnt in self.labPts: # for each vertex of 'self' in the lab frame frame, determine if vertex lies within 'other'
            if point_in_poly_w( lPnt , other.labPts ):
                # print "This shape has a vertex in the other shape" , lPnt
                return True # Collision detected, stop searching and return true
        for lPnt in other.labPts:  # for each vertex of 'other' in the lab frame frame, determine if vertex lies within 'self'
            if point_in_poly_w( lPnt , self.labPts ):
                # print "The other shape has a vertex in this shape" , lPnt
                return True # Collision detected, stop searching and return true
        for s_sgmnt in self.objs: # For each segment in 'self'
            for o_sgmnt in other.objs: # For each segment in 'other'
                if intersect_seg_2D( s_sgmnt.labCoords , o_sgmnt.labCoords , slideCoincident = True , includeEndpoints = False ): # Determine if the segments cross
                    # print "Segments Cross:" , s_sgmnt.labCoords , o_sgmnt.labCoords
                    return True # Collision detected, stop searching and return true
        return False # If execution reached this point, all tests failed and there was no collision
        
    def get_midpoints( self ):
        """ Get the midpoints of all the segments in the lab frame """
        midPoints = []
        for pntDex in range( len( self.labPts ) ): # For each point, create a segment from this point to the next, closed figure
            midPoints.append( vec_avg( elemw( self.labPts , pntDex ) , elemw( self.labPts , pntDex + 1 ) ) )
        return midPoints
        
    def set_hidden( self , hidden ):
        """ Set all of the poly Canvas segments hidden (True) or visible (False) """
        self.hidden = hidden
        for obj in self.objs:
            obj.set_hidden( hidden )
            
    def toggle_hidden( self ):
        """ Hide if visible , show if hidden """
        self.set_hidden( not self.hidden )
        
    def set_colliding( self , colliding ):
        """ Set whether or not this polygon is considered for collisions """
        self.collActive = colliding
        
    def toggle_colliding( self ):
        """ Disable if enabled , Enable if disabled """
        self.collActive = not self.collActive
        
    def sim_disable( self ):
        """ Disable collisions and hide the drawables """
        self.set_colliding( False )
        self.set_hidden( True )
        
    def sim_enable( self ):
        """ Enable collisions and show the drawables """
        self.set_colliding( True )
        self.set_hidden( False )
    
    def calc_centroid_rel( self ):
        """ Calculate the polygon area and centroid and assign """
        self.area , self.relCentroid = get_area_and_centroid( self.points ) 
        
    def get_area_centroid_ntn( self ):
        """ Calculate the polygon area and centroid using notional points and poses """
        return get_area_and_centroid( self.ntnPts )
    
    def get_rel_centroid( self , wArea = False ):
        """ Get the controid of the poly relative to the poly center, in the center frame , optionally return area """
        # area , centroid = poly2D_convex_area_centroid( self.points )
        if not self.area or not self.relCentroid:
            self.calc_centroid_rel()
        if wArea:
            return self.relCentroid , self.area
        else:
            return self.relCentroid
        
    def get_lab_centroid( self , wArea = False ):
        """ Get the controid relative of the poly in the lab frame """
        relCentroid , area = self.get_rel_centroid( True )
        if wArea:
            return self.local_to_lab( relCentroid ) , area
        else:
            return self.local_to_lab( relCentroid )
        
    def torque_about( self , graVec , labFulcrum ):
        """ Return the torque about 'labFulcrum' given the direction of gracity 'graVec', in the lab frame """
        # Calculate the lever arm that the gravity vector acts through, gravity acts at the centroid of the poly
        labCentroid = self.local_to_lab( self.relCentroid )
        leverArm = d_point_to_segment_2D( labFulcrum , [ labCentroid , np.add( labCentroid , graVec ) ] )
        return leverArm * self.area * vec_mag( graVec ) # Poly area is a substitute for mass , assume that mag of gravity vector represents g
        # Assume uniform area density = 1
    
    def get_copy( self ):
        """ Returns a copy of this poly """
        temp = Poly2D( self.position , self.orientation.theta , *[ vec_copy( pnt ) for pnt in self.points ] )
        temp.name = self.name
        temp.relCentroid = self.relCentroid
        temp.area = self.area
        return temp
        
    def ray_collide_lab( self , ray ):
        """ Return True if this ray collides with the frame or any of its contents """
        for segment in self.get_sides( "lab" ):
            if is_vector( intersect_ray_seg_pnt_2D( ray , segment , endPoints = False  , slideCoincident = True ) ): # If endpoints are included, touching parts are always trapped
                return True
        for frame in self.subFrames:
            if frame.ray_collide( ray ):
                return True
        return False # If the ray did not intersect the polygon or any of its successors, there was no collision
            
    def ray_collisions_ext_pnt_lab( self , ray ): # TODO: 2017-01-20 - HAVE TO LEAVE THIS HERE TO MAKE PREVIOUS CODE WORK
        """ Return a list of exterior collisions between this 'ray' and Poly2D , List is empty if there are no exterior collisions """
        rtnList = [] 
        for CCWsegment in self.get_sides():
            temp = intersect_ray_seg_pnt_2D( ray , CCWsegment , endPoints = False )
            # 1. Determine if there is a collision
            if is_vector( temp ):
                # 2. If there is a collision, determine if the ray origin is on the exterior of the polygon
                # NOTE: If the ray origin is within a concave poly, and the ray exits, then re-enters the poly, the re-entry will be counted
                if pnt_rght_of_ray( ray[0] , CCWsegment ):
                    rtnList.append( temp ) # If the origin is to the right of the segment (defined CCW), consider the ray origin outside of the polygon
        return rtnList
    
    # intersect_ray_seg_pnt_dist_2D( ray , seg )
        
    def ray_collisions_ext_pnt( self , ray , frame = 'lab' , slideCoincident = False ): 
        """ Return a list of exterior collisions between this 'ray' and Poly2D in the 'frame' , List is empty if there are no exterior collisions , 'endPoints' optional """
        # NOTE: This may be performed for the relative , lab , or notional frame
        rtnList = [] 
        CCWsidesList = self.get_sides( frame )
        # print "Sides:" , CCWsidesList
        for sDex , CCWsegment in enumerate( CCWsidesList ): # It is probably a good idea to update the poly in the frame before asking for coords in that frame
            # print "How does this ray collide?:" , intersect_ray_seg_pnt_frac_2D( ray , CCWsegment )
            temp , rayFrac , segFrac = intersect_ray_seg_pnt_frac_2D( ray , CCWsegment )
            # 1. Determine if there is a collision
            if is_vector( temp ):
                # 2. If there is a collision, determine if the ray origin is on the exterior of the polygon
                # NOTE: If the ray origin is within a concave poly, and the ray exits, then re-enters the poly, the re-entry will be counted
                if segFrac == None: # If the floor is exactly aigned with the segment, do not consider it to collide
                    if not slideCoincident:
                        rtnList.append( temp )
                    else:
                        pass
                elif eq( segFrac , 0 ): # if the ray passes through the 0-end of the segment , we have to evaluate whether it points into the shape
                    # 1. Get the CCW and CW rays , we already have the CCW ray from this point as a segment, see 'Poly2D.get_sides'
                    CWray = CW_ray_from_vrtx_index( self.get_points( frame ) , sDex )
                    # print "Clockwise Ray:" , CWray
                    # print "CCW Ray:      " , CCWsegment
                    # print "Incident Ray: " , ray
                    # 2. Determine the directions of the side-rays and the incident rays
                    CCWdir = ray_angle( CCWsegment )
                    CWdir  = ray_angle( CWray )
                    rayDir = ray_angle( ray )
                    # 3. Determine if the direction of the incident ray points into the interior arc of the adjacent sides
                    interior = wrap_bounds_fraction( [ 0.0 , 2*pi ] , [ CCWdir , CWdir ] , rayDir )
                    # print "How much interior?:" , interior
                    if interior > 0 and interior < 1: # If the ray points into the interior arc, it collides
                        rtnList.append( temp )
                elif eq( segFrac , 1 ): # If we are at the far end of a segment , do nothing , adjcent segent will catch it assuming closed polygon
                    pass
                else: # else the ray collides with the middle of the segment, and the answer is easy
                    if pnt_rght_of_ray( ray[0] , CCWsegment ):
                        rtnList.append( temp ) # If the origin is to the right of the segment (defined CCW), consider the ray origin outside of the polygon
        return rtnList
        
    def ray_nearest_exterior_clsn_dist_lab( self , ray ):
        """ Return the nearest distance between the 'ray' origin and any collision with the exterior of the polygon , return None if there is no collision """
        clsnDist = [ vec_dif_mag( clsnPnt , ray[0] ) for clsnPnt in self.ray_collisions_ext_pnt_lab( ray ) ] # Get this distances between the ray origins and collisions
        for frame in self.subFrames:
            subDist = frame.ray_nearest_exterior_clsn_dist_lab( ray )
            if subDist != None:
                clsnDist.append( subDist ) 
        if len( clsnDist ) > 0:
            return min( clsnDist )
        else:
            return None
        
    def is_trapped_simple_lab( self , frameCollection , sampleRays = 16 , activeOnly = False ):
        """ Return True if there is no unobstructed straight translation path from the current position to the outside of 'frameCollection' , lab frame """
        rayDirs = set( [ self.labPose.orientation._theta + angle for angle in np.linspace( 0 , 2 * pi , sampleRays + 1 )[:-1] ] ) # All of the relative directions to project the polygon in
        for segmentCCW in self.get_sides():
            sideAng = cart_2_polr( np.subtract( segmentCCW[1] , segmentCCW[0] ) )[1] # Calc the direction that the segment points in in the lab frame
            rayDirs.add( sideAng ) # Test rays in the side direction
            rayDirs.add( sideAng + pi ) # Test parallel and opposite to the direction of the side
            rayDirs.add( sideAng + pi/2 ) # Test in a direction that is perpendicular and interior to the CCW side
        hullPts = self.hull_of_contents_lab()
        for rDex , rDir in enumerate( rayDirs ): # For each of the sample directions
            # print "DEBUG , Ray Angle" , rDir , "," , rDex + 1 , "of" , len( rayDirs )
            obstructed = False
            for hDex , hPt in enumerate( hullPts ): # For each of the vertices of the complex hull
                # print "\tDEBUG , Hull Point" , hPt , "," , hDex + 1 , "of" , len( hullPts )
                # Construct a ray with the origin at the vertex and pointing in the sample direction
                trialRay = [ hPt , np.add( hPt , polr_2_cart( [ 1 , rDir ] ) ) ]
                for frame in frameCollection: 
                    # print "\t\tDEBUG , Working on" , frame.name
                    if frame != self:
                        if activeOnly and not frame.collActive: # 'activeOnly' means that we only consider collisions with frames that are active
                            # print "\t\tDEBUG , not evalating collisions with inactive part" , frame.name
                            continue # Note that inactive parts can call this function and still achieve the desired result
                        if frame.ray_collide_lab( trialRay ):
                            obstructed = True
                            # print "\t\tDEBUG , Obstruction!"
                            break
                if obstructed:
                    break
            if not obstructed: # If we casted rays from all the vertices in one direction and none hit an object in the collection, shape is not trapped
                return False
        return True # If all rays from all vertices were cast in all directions without finding an unobstructed direction, then the shape is trapped
    
    def enclosure_number( self , frameCollection , sampleRays = 16 , activeOnly = False ):
        """ Return a number that represents the least number of part boundary crossings encountered be a collection of sample rays cast from the part """
        # Create a set of directions , start with an evenly distributed number 'sampleRays' of directions , relative directions in the poly frame
        rayDirs = set( [ self.labPose.orientation._theta + angle for angle in np.linspace( 0 , 2 * pi , sampleRays + 1 )[:-1] ] ) 
        # Add directions that are parallel to the sides of the poly
        for segmentCCW in self.get_sides():
            sideAng = cart_2_polr( np.subtract( segmentCCW[1] , segmentCCW[0] ) )[1] # Calc the direction that the segment points in in the lab frame
            rayDirs.add( sideAng ) # Test rays in the side direction
            rayDirs.add( sideAng + pi ) # Test parallel and opposite to the direction of the side
            rayDirs.add( sideAng + pi/2 ) # Test in a direction that is perpendicular and interior to the CCW side
        hullPts = self.hull_of_contents_lab() # Create a convex hull of this poly
        lenHullPts = len( hullPts ) # the number of points in the convex hull
        leastAvgCrossings = infty
        for rDex , rDir in enumerate( rayDirs ): # For each of the sample directions
            # print "DEBUG , Ray Angle" , rDir , "," , rDex + 1 , "of" , len( rayDirs )
            collideCount = 0
            for hDex , hPt in enumerate( hullPts ): # For each of the vertices of the complex hull
                # print "\tDEBUG , Hull Point" , hPt , "," , hDex + 1 , "of" , len( hullPts )
                # Construct a ray with the origin at the vertex and pointing in the sample direction
                trialRay = [ hPt , np.add( hPt , polr_2_cart( [ 1 , rDir ] ) ) ]
                for frame in frameCollection: 
                    # print "\t\tDEBUG , Working on" , frame.name
                    if frame != self:
                        if activeOnly and not frame.collActive: # 'activeOnly' means that we only consider collisions with frames that are active
                            # print "\t\tDEBUG , not evalating collisions with inactive part" , frame.name
                            continue # Note that inactive parts can call this function and still achieve the desired result
                        if frame.ray_collide_lab( trialRay ):
                            collideCount += 1
                            # print "\t\tDEBUG , Obstruction!"
            leastAvgCrossings = min( leastAvgCrossings , roundint( float(collideCount) / (lenHullPts*2) ) ) # Enclosure should be a whole number
        return leastAvgCrossings # If all rays from all vertices were cast in all directions without finding an unobstructed direction, then the shape is trapped
    
    def freeness_simple_lab( self , frameCollection , sampleRays = 16 ):
        """ Return the maximum extend of any sampled ray cast from this Poly2D into the 'frameCollection' , return infinity of there is a free direction """
        # NOTE: This is intended as a very rough measure of freedom of movement, and does not represent an actual collision-free distance
        if self.is_trapped_simple_lab( frameCollection , sampleRays ): # If the part is trapped, then calc some maximum free path
            rayDirs = set( [ self.labPose.orientation._theta + angle for angle in np.linspace( 0 , 2 * pi , sampleRays + 1 )[:-1] ] ) # All of the relative directions to project the polygon in
            for segmentCCW in self.get_sides():
                sideAng = cart_2_polr( np.subtract( segmentCCW[1] , segmentCCW[0] ) )[1] # Calc the direction that the segment points in in the lab frame
                rayDirs.add( sideAng ) # Test rays in the side direction
                rayDirs.add( sideAng + pi ) # Test parallel and opposite to the direction of the side
                rayDirs.add( sideAng + pi/2 ) # Test in a direction that is perpendicular and interior to the CCW side
            hullPts = self.hull_of_contents_lab() # Get the hull of this Poly2D and any shapes it might contain
            hullCtr = get_area_and_centroid( hullPts )[1]
            
            # freeness = infty # init freeness to zero and increase for any length of free ray            
            freeness = 0 # init freeness to zero and increase for any length of free ray            
            
            for rDir in rayDirs: # For each of the sample directions
                pointFree = infty
                for hPt in hullPts: # For each of the vertices of the complex hull
                    # Construct a with the origin at the vertex and pointing in the sample direction
                    rayOffset = polr_2_cart( [ 1 , rDir ] )
                    trialRay = [ hPt , np.add( hPt , rayOffset ) ] # Construct a ray in the direction with the hull point as origin
                    if angle_between( np.subtract( hPt , hullCtr ) , rayOffset ) <= pi*0.99/2: # This is a somewhat arbitrary cutoff 
                        freeDists = []
                        for frame in frameCollection:
                            if frame != self:
                                temp = frame.ray_nearest_exterior_clsn_dist_lab( trialRay )
                                if temp > 0 and temp < infty and temp != None: # Reject projected distance to an adjacent object
                                    freeDists.append( temp )
                        if len( freeDists ) > 0:
                            pointFree = min( pointFree , min( freeDists ) )
                pointFree = pointFree if pointFree < infty else 0
                freeness = max( freeness , pointFree )
            return freeness
        else:
            return infty # else the part is open, and there is an infinite free distance
        
    # In retrospect, it may have been easier just to do some collision detection! Original idea was to "shake" the part around in space and see what it hits

def CCW_ray_from_vrtx_index( CCWlist , vertexIndex ):
    """ Construct a ray with origin at the vertex at 'vertexIndex' along the side CCW from the origin """
    return [ elemw( CCWlist , vertexIndex ) , elemw( CCWlist , vertexIndex + 1 ) ]
    
def CW_ray_from_vrtx_index( CCWlist , vertexIndex ):
    """ Construct a ray with origin at the vertex at 'vertexIndex' along the side CW from the origin """
    # print "Points:     " , CCWlist
    # print "Ray point 0:" , elemw( vertexIndex , CCWlist )
    # print "Ray point 1:" , elemw( vertexIndex - 1 , CCWlist )
    return [ elemw( CCWlist , vertexIndex ) , elemw( CCWlist , vertexIndex - 1 ) ]

def CCW_CW_rays_from_vrtx_index( CCWlist , vertexIndex ):
    """ Construct rays with origin at the vertex at 'vertexIndex' along the sides CCW from and CW from the origin , in that order """
    return [ elemw( CCWlist , vertexIndex ) , elemw( CCWlist , vertexIndex + 1 ) ] , [ elemw( CCWlist , vertexIndex ) , elemw( CCWlist , vertexIndex - 1 ) ]
        
def floor_ref_posed_for_segment( ntnSupportSeg , margin = None ):
    """ Return a reference to a Poly2D object representing the floor supporting a sub on a 'ntnSupportSeg' """
    midpoint = vec_avg( *ntnSupportSeg ) # Get the center of the supporting segment
    angle = ray_angle( ntnSupportSeg ) # Get the angle of the supporting segment
    if margin == None:
        floor_ref_posed_for_segment.floorBox.set_Pose( Pose2D( midpoint , angle ) )
    else:
        rayDir = angle - pi/2
        floor_ref_posed_for_segment.floorBox.set_Pose( 
            Pose2D( np.add( midpoint , polr_2_cart( [ margin , rayDir ] ) ) , angle ) 
        )
    floor_ref_posed_for_segment.floorBox.transform_contents()
    return floor_ref_posed_for_segment.floorBox

depth = 1000
halfWidth = 2000

floor_ref_posed_for_segment.floorBox = Poly2D( [ 0 , 0 ] , 0 ,
                                            # [ -1000 , -10 ] , [  1000 , -10 ]  , [  1000 ,   0 ] ,  [ -1000 ,   0 ] ) # Extremelty wide , very thin , floor plane
                                              [ -halfWidth , -depth ] , [  halfWidth , -depth ]  , [  halfWidth ,   0 ] ,  [ -halfWidth ,   0 ] ) # Gigantic square below the floor
floor_ref_posed_for_segment.floorBox.set_colliding( True ) # This should have been True by default , but just in case
    
def next_part_collide_floor( ntnSupportSeg , nextPart , nextNtnPose , floor = None ):
    """ Return True if 'nextPart' collides with the floor assuming its notional pose 'nextNtnPose' and this part is supported by 'ntnSupportSeg' , otherwise False """
    # 1. copy the next part and make sure it is active
    nuPart = nextPart.get_copy() ; nuPart.set_colliding( True )
    # 2. Send it to the appropriate pose
    nuPart.set_Pose( nextNtnPose , frame = 'rel' ) # Assign the specified notional pose to the next part
    # 3. Send the floor to the middle of the supporting segment
    if floor == None: # If we have not already precomputed the pose of the floor , compute
        floor = floor_ref_posed_for_segment( ntnSupportSeg )
    # else there was a floor computed
    # 4. Assign lab coordinates and bounding boxes
    nuPart.transform_contents() # ; print nuPart
    # 5. Evaluate collision and return result
    return floor.check_collision_with( nuPart )
        
# - End Poly2D -
        
# = End 2D Frames =

# == End 2D ==


# == Geo Functions that Use Geo Objects ==

def perp_bisector_2D( segment ):
    """ Return a bisector the same legnth as 'segment' , perpendiculat to 'segment' , and with the same midpoint as 'segment' , Return midpoint """
    midPoint = vec_avg( *segment )
    pt1 = np.add( perp_bisector_2D.turn.apply_to( np.subtract( segment[0] , midPoint ) ) , midPoint )
    pt2 = np.add( perp_bisector_2D.turn.apply_to( np.subtract( segment[1] , midPoint ) ) , midPoint )
    return [ pt1 , pt2 ] , midPoint
perp_bisector_2D.turn = Turn( pi / 2 )

# == End Geo Geo ==


## === SPARE PARTS =========================================================================================================================

#~ # === Assembly Planning Frames ===

#~ # == class Design ==

#~ # ~~ USAGE ~~
#~ # 1. All parts MUST have a unique name so that a one-to-one relationship can be established between the Design and the simulated part representation
        
#~ class Design(Frame2D):
    #~ """ Represents a 'CAD model' of a 2D assemblage of polygons """
    
    #~ def __init__( self , pPos , pTheta , *subPolys ):
        #~ """ Set this object up as a 2D frame """
        #~ super( Design , self ).__init__( pPos , pTheta )
        #~ self.graph = None # Holds the assembly order graph, has the same Pose as the associated Design # IS THIS EVEN USED? # Comment out and see what breaks
        #~ for poly in subPolys:
            #~ self.attach_sub( poly ) 
        #~ self.name = None
        
    #~ def __str__( self ):
        #~ """ Return a string summary of the Design """
        #~ return "Design , Name: " + str( self.name ) + " , Number of parts: " + str( len( self.subFrames ) ) + \
               #~ " , Part names: " + str( [part.name for part in self.subFrames] )
           
    #~ def get_part_index_w_name( self , partName ):
        #~ """ Search for a member 'partName' and return the index if found, otherwise return None """
        #~ for pDex , poly in enumerate( self.subFrames ):
            #~ if poly.name == partName:
                #~ return pDex
        #~ return None
        
    #~ def get_part_by_name( self , partName ):
        #~ """ Return a reference to a part associated with the name given """
        #~ return self.subFrames[ self.get_part_index_w_name( partName ) ]
        
    #~ def get_all_part_names( self ):
        #~ """ Return all the part names for this Design """
        #~ rtnNames = []
        #~ for poly in self.subFrames:
            #~ rtnNames.append( poly.name )
        #~ return rtnNames 

    #~ def active_names_list( self , active = True ):
        #~ """ Return a list of part names with their 'collActive' flags set to 'active'  """
        #~ rtnNames = []
        #~ for frame in self.subFrames:
            #~ if frame.collActive == active:
                #~ rtnNames.append( frame.name )
        #~ return rtnNames    
          
    #~ def get_target_Pose_by_name( self , partName , frame = 'lab' ): # Use this to get the target of RRT build operations
        #~ """ Return a Pose2D representing where 'partName' belongs in the lab frame , if 'partName' does not exist, return None """
        #~ index = self.get_part_index_w_name( partName ) # Get the part index or None if name DNE
        #~ if index != None:
            #~ return self.subFrames[ index ].get_Pose( frame ) # Get the part pose in the specified frame
        #~ else:
            #~ return None # The part name was not found , return None
            
    #~ def copy_parts( self ):
        #~ """ Return copies of all the parts of the design in order to use them for assembly """
        #~ partList = []
        #~ for part in self.subFrames:
            #~ temp = part.get_copy()
            #~ temp.collActive = True # serving part to the simulation, collide
            #~ partList.append( temp )
        #~ return partList
        
    #~ def part_refs_list( self ):
        #~ """ Return a list of references to the Design non-colliding parts """
        #~ return self.subFrames[:] # Return a shallow copy of the subframes list
             
    #~ def get_copy( self , doCollide = False , preserveActive = False ):
        #~ """ Get a copy of this Design with optionally colliding parts """
        #~ partList = []
        #~ for part in self.subFrames:
            #~ temp = part.get_copy()
            #~ if preserveActive: # If we are preserving active flag across copies , ignore 'doCollide'
                #~ temp.collActive = part.collActive
            #~ else: # If we are not preserving active flag across copies , assign all parts 'doCollide'
                #~ temp.collActive = doCollide 
            #~ partList.append( temp )
        #~ rtnDesign = Design( self.position , self.orientation._theta , *partList )
        #~ rtnDesign.name = self.name
        #~ return rtnDesign
    
    #~ def sub_Design_from_parts_list( self , partNames , doCollide = False ):
        #~ """ Return a Design object composes of only 'partNames' in their speicified relative positions """
        #~ subList = []
        #~ for name in partNames:
            #~ temp = self.get_part_by_name( name ).get_copy() # This will throw an error if the part name does not exist in the design!
            #~ temp.collActive = doCollide
            #~ subList.append( temp )
        #~ return Design( self.position , self.orientation._theta , *subList )
            
    #~ def label_parts_enclosed( self ):
        #~ """ Iterate through all the parts and label them enclosed or not """
        #~ # allParts = self.part_refs_list()
        #~ for part in self.subFrames:
            #~ # part.enclosed = part.is_trapped_simple_lab( allParts )
            #~ part.enclosed = part.is_trapped_simple_lab( self.subFrames ) 
            
    #~ def has_trapped_inactive( self ):
        #~ """ Return true if one of the inactive parts is trapped , use this to cache trappedness of unbuilt parts """
        #~ for part in self.subFrames:
            #~ if ( not part.collActive ) and part.enclosed:
                #~ return True
        #~ return False
            
    #~ def label_parts_free( self ):
        #~ """ Iterate through all the parts and assess the freeness of each """
        #~ # allParts = self.part_refs_list()
        #~ for part in self.subFrames:
            #~ # part.freeness = part.freeness_simple_lab( allParts )
            #~ part.freeness = part.freeness_simple_lab( self.subFrames )
            
    #~ def set_parts_colliding( self , active = True ):
        #~ """ Set all the parts to be collidable, the entire design is part of the simulation """
        #~ for frame in self.subFrames:
            #~ frame.set_colliding( active )
            
    #~ def set_active_by_name( self , pName , active = True ):
        #~ """ Search for 'pName' and set the associated part 'active' """
        #~ self.get_part_by_name( pName ).set_colliding( active )
        
    #~ def set_active_by_list( self , namesList , active = True ):
        #~ """ Set the 'active' state of each name in namesList """
        #~ for pName in namesList:
            #~ self.get_part_by_name( pName ).set_colliding( active )
            
    #~ def set_active_by_list_exclusive( self , namesList , active = True ):
        #~ """ Set the 'active' state of each name in namesList , and set all other names to the opposite state """
        #~ for pName in self.get_all_part_names():
            #~ if pName in namesList: # Name is on the list , set to the desired state
                #~ self.get_part_by_name( pName ).set_colliding( active )
            #~ else: # else name is not on the list , set to opposite of the desired state
                #~ self.get_part_by_name( pName ).set_colliding( not active )
                
    #~ def bbox_active_only( self , frame = "lab" , active = True ):
        #~ """ Return the bounding box of all the Design members that are 'active' """
        #~ appPts = []
        #~ for frame in self.subFrames:
            #~ if frame.collActive == active:
                #~ appPts.extend( frame.get_points( frame ) )
            
    #~ def putdowns_from_part_list( self , partsList ):
        #~ """ Generate the putdown poses of an imaginal version of the design in which only 'partsList' is active """
        #~ temp = self.get_copy() # Gen a scratch copy
        #~ temp.set_active_by_list_exclusive( partsList ) # Activate the parts we are interested in and only those parts
        #~ temp.transform_contents() # Make sure everything is in the proper spot
        #~ temp.set_supports_for_hull( activeOnly = True ) # Calc the supports
        #~ return temp.hullPutdowns[:] # Return a copy of the hull putdowns
        
    #~ def extent_scale( self , activeOnly = True ):
        #~ """ Return the diagononal of the bounding box of the relative points in order to offer a sense of scale """
        #~ return vec_dif_mag( *bbox_from_points( list( self.vertex_set_rel( activeOnly = activeOnly ) ) ) )
        
    #~ def count_active_parts( self , active = True ):
        #~ """ Return the number of subFrames that have their flag set to 'active' """
        #~ count = 0
        #~ for frame in self.subFrames:
            #~ if frame.collActive == active:
                #~ count += 1
        #~ return count
        
#~ # == End Design ==


#~ # == Stability and Placement ==

#~ def planar_poly_stability( vertices , centroid ):
    #~ """ Return a stability measure for each side of a closed polygon by projecting the 'centroid' onto each in turn """
    #~ # NOTE: This function does not check for the feasibility of resting on a side and assumes that the 'vertices' define a convex polygon
    #~ # NOTE: Stability listed according to the order of 'vertices' , a side is '[ vertices[i] , vertices[i+1] ]'
    #~ sideStability = []
    #~ for vDex in xrange( len( vertices ) ):
        #~ sideStability.append( proj_pnt_within_seg( centroid , [ elemw( vertices , vDex ) , elemw( vertices , vDex + 1 ) ] ) )
    #~ return sideStability
    
#~ class Table(Poly2D):
    #~ """ Object representing a broad, flat surface on which parts and assemblies will be set """
    
    #~ def __init__( self ,  ):
        #~ """ Create a thin rectangle with the center of the table at the origin , parts will not occupy negative y coords """
        #~ Poly2D.__init__( self , [ 0 , 0 ] , 0 , 
                                #~ [ -1000 , -1000 ] , [  1000 , -1000 ]  , [  1000 ,   0 ] ,  [ -1000 ,   0 ] )
        #~ self.name = "Table"

#~ def support_to_Pose( supportSeg ):
    #~ """ Given a supporting segment in the frame of the object to put down, return a Pose2D that placed the center of the support at [0,0] , within +y """
    #~ midpoint = vec_avg( *supportSeg ) #; print "midpoint" , midpoint
    #~ angle = ray_angle( supportSeg ) #; print "angle" , angle
    #~ return Pose2D( np.multiply( Turn( -angle ).apply_to( midpoint ) , -1 ) , -angle )
    #~ #      Pose2D( np.multiply( Turn( -angle ).apply_to( midpoint ) , -1 ) , -angle )
    
#~ def parts_from_structures( structList , polyList = [] ):
    #~ """ Return a list of Poly2D references given a list of frames that may or may not be nested """
    #~ for struct in structList: # For each of the structures in the list
        #~ if hasattr( struct , "points" ):
            #~ polyList.append( struct )
        #~ parts_from_structures( struct.subFrames , polyList )
    #~ return polyList

#~ # == End Stability ==

# ___ END SPARE ____________________________________________________________________________________________________________________________
