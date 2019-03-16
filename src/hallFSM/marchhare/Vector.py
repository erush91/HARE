#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

"""
Vector.py , Built on Spyder for Python 2.7
Erik Lindstrom , Adam Sperry , James Watson , 2016 October
Vectors , Turns , Frames , and common operations in 2D geometry

== LOG ==


== TODO ==

"""

# ~ Standard Libraries ~
import os , operator
from math import pi
from copy import deepcopy
from random import random
# ~ Special Libraries ~
import numpy as np
# ~ Local Libraries ~
from MathKit import eq_margin , round_small , eq

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl = os.linesep # Line separator
pyEq = operator.eq # Default python equality
piHalf = pi/2

# === Vector Mathematics ===

def vec_mag( vec ): 
    """ Return the magnitude of a vector """
    return np.linalg.norm( vec )

def vec_sqr( vec ): 
    """ Return the squared magnitude of the vector (avoid sqrt for quick comparison) """
    return np.dot( vec , vec ) # The squared magnitude is just the vector dotted with itself

def vec_dif_sqr( vec1 , vec2 ): 
    """ Return the squared magnitude of the vector difference between 'vec1' and 'vec2' """
    vecDiff = np.subtract( vec1 , vec2 )
    return np.dot( vecDiff , vecDiff ) # The squared magnitude is just the vector dotted with itself

def vec_dif_mag( vec1 , vec2 ):
    """ Return the magnitude of the vector difference between 'vec1' and 'vec2' """
    return vec_mag( np.subtract( vec1 , vec2 ) )

def vec_dif_unt( vec1 , vec2 ):
    """ Return the unit vector difference between 'vec1' and 'vec2' """
    return vec_unit( np.subtract( vec1 , vec2 ) )

def vec_zeros( dim ):
    """ Return a vector of 'dim' zeros """
    return [ 0 for d in xrange( dim ) ]

def matx_zeros( *dims ):
    """ For those times when you do not want the Zeros from numpy """
    if len( dims ) == 1:
        return vec_zeros( dims[0] )
    elif len( dims ) > 1:
        rtnMatx = []
        for i in xrange( dims[0] ):
            rtnMatx.append( matx_zeros( *dims[1:] ) )
        return rtnMatx
    else:
        raise IndexError( "matx_zeros: Something went wrong with args " + str( dims ) )        

def vec_round_small( vec ): 
    """ Round components that are have an absolute value less than 'EPSILON' to 0 """
    rtnVec = []
    for i in range( len( vec ) ):
        rtnVec.append( round_small( vec[i] ) )
    return rtnVec

def vec_unit( vec ): 
    """ Return a unit vector in the direction of 'vec', using numpy """
    mag = np.linalg.norm( vec )
    return np.divide( vec , 1 if eq( mag , 0 ) else mag )

def vec_proj( a , b ): 
    """ a projected onto b, a scalar length, using numpy """
    return np.dot(a,b) / np.linalg.norm(b) # Note that the result will be negative if the angle between a and b is > pi/2

def vec_proj_onto( a , b ): 
    """ a projected onto b,  vector in the direction of b """
    return np.multiply( vec_unit( b ) , vec_proj( a , b ) )

def vec_angle_between( v1 , v2 ): 
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
        # URL, angle between two vectors: http://stackoverflow.com/a/13849249/893511
    v1_u = vec_unit( v1 )
    v2_u = vec_unit( v2 )
    if eq( vec_mag( v1_u ) , 0 ) or eq( vec_mag( v2_u ) , 0 ):
        return float('nan')
    angle = np.arccos( np.dot( v1_u , v2_u) )
    if np.isnan( angle ):
        if ( v1_u == v2_u ).all():
            return 0.0
        else:
            return np.pi
    return angle

def vec_parallel( v1 , v2 ):
    """ Return true if 'v1' and 'v2' are parallel within EPSILON, false otherwise """
    angle = vec_angle_between( v1 , v2 )
    return eq( angle , 0.0 ) or eq( angle , pi )

def vec_NaN( dim ):
    """ Return a row vector of dimension 'dim' composes of NaN """
    return [ float('NaN') for i in xrange( dim ) ]

def np_add( *args ):
    """ Perform 'np.add' on more than two args """
    if len( args ) > 2: # If there are more than 2 args, add the first arg to recur on remainder of args
        return np.add( args[0] , np_add( *args[1:] ) ) # Note the star operator is needed for recursive call, unpack to positional args
    else: # base case, there are 2 args*, use vanilla 'np.add'
        return np.add( args[0] , args[1] ) # *NOTE: This function assumes there are at least two args, if only 1 an error will occur

def np_dot( *args ): # <<< toybox
    """ Perform 'np.dot' on more than two args """
    if len( args ) > 2: # If there are more than 2 args, add the first arg to recur on remainder of args
        return np.dot( args[0] , np_dot( *args[1:] ) ) # Note the star operator is needed for recursive call, unpack to positional args
    else: # base case, there are 2 args*, use vanilla 'np.add'
        return np.dot( args[0] , args[1] ) # *NOTE: This function assumes there are at least two args, if only 1 an error will occur

def np_subtract(*args): # <<< resenv
    """ Perform 'np.subtract' on more than two args """
    if len(args) > 2: # If there are more than 2 args, subtract the last arg from the preceeding remainder of args
        return np.subtract( np_subtract(*args[:-1]) , args[-1] ) # Note the star operator is needed for recursive call, unpack to positional args
    else: # base case, there are 2 args*, use vanilla 'np.subtract'
        return np.subtract( args[0] , args[1] ) # *NOTE: This function assumes there are at least two args, if only 1 an error will occur

def vec_sum_chain( vecList ): 
    """ Create a list of vectors that are the resultants of summing correspong vector in 'vecList' and all preceding vectors """
    ptsList = [ vecList[0][:] ] # first vector is the same as first in 'vecList'
    for i in range(1,len(vecList)): # for every successive vector
        ptsList.append( np.add( ptsList[-1] , vecList[i] ) )
    return ptsList

def vec_eq( vec1 , vec2 , margin = EPSILON ):
    """ Return true if two vectors are equal enough, otherwise false """
    if len(vec1) == len(vec2):
        for i in range(len(vec1)):
            if not eq_margin( vec1[i] , vec2[i] , margin):
                return False
    else:
        return False
    return True

def vec_eq_test_w_margin( margin = EPSILON ):
    """ Return a function that performs an 'vec_eq' comparison with the specified margin """
    def eq_test( op1 , op2 ):
        return vec_eq( op1 , op2 , margin )
    return eq_test

def vec_linspace( vec1 , vec2 , numPts ):
    """ Return a list of 'numPts' points (vectors) evenly spaced from 'vec1' to 'vec2', inclusive """
    diff = np.subtract( vec2 , vec1 ) # Vector from point 1 to point 2
    direction = vec_unit( diff ) # Direction of the vector between the two
    span = vec_mag( diff ) # the Euclidian distance between the two points
    ptsList = [] 
    for value in np.linspace(0, span, num=numPts): # For each value in the range
        ptsList.append( np.add( vec1 , # append the sum of the first point and
                                np.multiply( direction , # a vector that is an additional space along the difference
                                             value ) ) )
    return ptsList

def linspace_endpoints( bgnPnt , endPnt , numPnts ):
    """ Create a list of 'numPnts' points between 'bgnPnt' and 'endPnt' , inclusive """
    # NOTE: This function assumes that 'bgnPnt' and 'endPnt' have the same dimensionality
    # NOTE: This is a re-implementation of 'Vector.vec_linspace' , but without np.ndarray
    coordsList = []
    rtnList = []
    for i in xrange( len( bgnPnt ) ):
        coordsList.append( np.linspace( bgnPnt[i] , endPnt[i] , numPnts ) )
    for i in xrange( numPnts ):
        temp = []
        for j in xrange( len( bgnPnt ) ):
            temp.append( coordsList[j][i] )
        rtnList.append( temp )
    return rtnList

def vec_avg( *vectors ): 
    """ Return a vector that is the average of all the 'vectors', equal weighting """
    vecSum = np_add( *vectors ) # NOTE: This function assumes that all vectors are the same dimensionality
    return np.divide( vecSum , len( vectors ) * 1.0 )

def is_vector( vec ): 
    """ Return true if 'vec' is any of { list , numpy array } and thus may particpate in vector operations """
    return isinstance( vec , ( list , np.ndarray ) )

def vec_copy( vec ): 
    """ Return a copy of 'vec', using the appropriate copy mechanism for the underlying datatype """
    if isinstance( vec , list ): 
        return vec[:] 
    elif isinstance( vec , np.ndarray ):
        return vec.copy()
    else:
        raise TypeError("vec_copy: " + str(vec) + " was neither a 'list' nor a 'np.ndarray'!")

def vec_copy_deep( vec ): 
    """ Return a deep copy of 'vec', using the appropriate copy mechanism for the underlying datatype """
    if isinstance( vec , list ): 
        return deepcopy( vec )
    elif isinstance( vec , np.ndarray ):
        return vec.copy()
    else:
        raise TypeError("vec_copy_deep: " + str(vec) + " was neither a 'list' nor a 'np.ndarray'!")

def vec_random( dim ): 
    """ Return a random vector in R-'dim' space with coordinates in [0,1) """
    rtnVec = []
    for i in range(dim):
        rtnVec.append( random() )
    return rtnVec

def vec_random_range( dim , limLo , limHi ): 
    """ Return a vector in which each element takes on a random value between 'limLo' and 'limHi' with a uniform distribution """
    rtnVec = []
    randVec = vec_random( dim )
    span = abs( limHi - limLo )
    for elem in randVec:
        rtnVec.append( elem * span + limLo )
    return rtnVec

def vec_rand_range_lst( dim , limLo , limHi , N ): 
    """ Return a list of 'N' vectors generated by 'vec_random_range' """
    rtnList = []
    for i in xrange( N ):
        rtnList.append( vec_random_range( dim , limLo , limHi ) )
    return rtnList

def vec_random_limits( dim , limits ): 
    """ Return a vector in which each element takes on a random value between 'limits[i][0]' and 'limits[i][1]' with a uniform distribution """
    rtnVec = []
    randVec = vec_random( dim )
    for i , elem in enumerate( randVec ):
        span = abs( limits[i][1] - limits[i][0] )
        rtnVec.append( elem * span + limits[i][0] )
    return rtnVec

def vec_random_perturb( center , radius ): 
    """ Return a vector perturbed by length 'radius' away from 'center' in a random direction """
    randDir = vec_unit( vec_random_range( len( center ) , -1 ,  1 ) ) # Construct a random unit vector with the same dimensionality as 'center'
    offset = np.multiply( randDir , radius )
    return np.add( center , offset )

def vec_rand_prtrb_lst( center , radius , N ): 
    """ Return a list of 'N' vectors generated by 'vec_random_perturb' """
    rtnList = []
    for i in xrange( N ):
        rtnList.append( vec_random_perturb( center , radius ) )
    return rtnList

def vec_round( vec , places=0 ): # <<< resenv
    """ Round each element of 'vec' to specified 'places' """
    rtnVec = []
    for elem in vec:
        rtnVec.append( round( elem , places ) )
    return rtnVec

def vec_clamp( vec , loBnd , upBnd ): # <<< resenv
    """ Return a version of 'vec' with all elements clamped b/n ['loBnd' , 'upBnd'], inclusive """
    rtnVec = []
    for coord in vec:
        if coord < loBnd:
            rtnVec.append( loBnd )
        elif coord > upBnd:
            rtnVec.append( upBnd )
        else:
            rtnVec.append( coord )
    return rtnVec

def vec_clamp_fraction( vec , loBnd , upBnd ): # <<< resenv
    mid = (upBnd + loBnd) / 2
    scales = []
    for i , elem in enumerate( vec ):
        if elem <= mid: # If the angle is less than 'mid', then scale
            scales.append( abs( ( mid - elem ) / ( mid - loBnd ) ) )
        else: # else angle greater than mid, scale
            scales.append( abs( ( elem - mid ) / ( upBnd - mid ) ) )
    return scales

def vec_scaled_clamp( vec , loBnd , upBnd ): # <<< resenv
    """ If any element of 'vec' is out of bounds, return a scaled copy of 'vec' such that the most-offending element is within bounds, or return 'vec' """
    if not vec_check_bounds( vec , loBnd , upBnd ):
        return np.divide( vec , max( vec_limit_fraction( vec , loBnd , upBnd ) ) )
    else:
        return vec

def vec_check_bounds( vec , loBnd , upBnd ): # <<< resenv
    """ Return true if all elements  of 'vec' are b/n ['loBnd' , 'upBnd'], inclusive """
    for coord in vec:
        if coord < loBnd:
            return False
        elif coord > upBnd:
            return False
    return True

def vec_check_limits( vec , limits ): # <<< resenv
    """ Return True of 'vec' [x_0 ... x_N] falls within 'limits' ( ( x0min , x0max ) ... ( xNmin , xNmax ) ) , otherwise false """
    for i , lim in enumerate(limits):
        if vec[i] < lim[0]:
            return False
        elif vec[i] > lim[1]:
            return False
    return True

def vec_limit_fraction( vec , limits ): # <<< resenv
    scales = []
    for i , elem in enumerate( vec ):
        mid = ( limits[i][0] + limits[i][1] ) / 2
        if elem <= mid: # If the angle is less than 'mid', then scale
            scales.append( abs( ( mid - elem ) / ( mid - limits[i][0] ) ) )
        else: # else angle greater than mid, scale
            scales.append( abs( ( elem - mid ) / ( limits[i][1] - mid ) ) )
    return scales

def vec_scaled_limits( vec , limits ): # <<< resenv
    """ If any element of 'vec' is out of bounds, return a scaled copy of 'vec' such that the most-offending element is within bounds, or return 'vec' """
    if not vec_check_limits( vec , limits ):
        return np.divide( vec , np.absolute( vec_limit_fraction( vec , limits ) ) )
    else:
        return vec

def vec_from_seg( segment ):
    """ Return the vector that points in the direction from point 'segment[0]' to 'segment[1]' """
    return np.subtract( segment[1] , segment[0] )

def bbox_from_points( ptsList ):
    """ Return the axis-aligned bounding box that contains all of 'ptsList' , Takes the form [ [ VEC_OF_MINS ... ] , [ VEC_OF_MAXS ... ] ] """
    bbox = [ [ infty for i in ptsList[0] ] , [ -infty for i in ptsList[0] ] ] # NOTE: This function assumes that all points have the same dimensionality
    for point in ptsList: # For every point in the list
        for i in xrange( len( point ) ): # For every dimension of the point
            bbox[0][i] = min( bbox[0][i] , point[i] ) # Compare the component of the point to the min of the dimension
            bbox[1][i] = max( bbox[1][i] , point[i] ) # Compare the component of the point to the max of the dimension
    return bbox

def AABB( ptsList ):
    """ Axis-Aligned Bounding Box, Alias for 'bbox_from_points' """
    return bbox_from_points( ptsList )

def AABB_span( ptsList ):
    """ Return the span of each dimension covered by the AABB of 'ptsList' """
    bbox = bbox_from_points( ptsList )
    span = [ 0 for i in xrange( len( bbox[0] ) ) ]
    for i in xrange( len( bbox[0] ) ):
        span[i] = abs( bbox[1][i] - bbox[0][i] )
    return span

# == Plotting Helpers ==

def split_to_components( vecList ): 
    """ Separate a list of R^n vectors into a list of n lists of components , in order """ # because matplotlib likes it that way
    components = [ [] for dim in xrange( len( vecList[0] ) ) ] # NOTE: This function assumes that all vectors of 'vecList' are the same dimensionality
    for vec in vecList:
        for i , elem in enumerate( vec ):
            components[i].append( elem )

# __ End Helpers __


# == Printing Helpers ==

def matx_2D_pretty_print( matx ):
    """ Pretty print a 2D 'matx' , NOTE: This will also print lists of lists with variable row length """
    maxWidth = 0
    # 1. Find the column width
    for row in matx:
        for col in row:
            maxWidth = max( maxWidth , len( str( col ) ) )
    # 2. Print the matrix row by row with uniform columns
    dsplyStr = "[ "
    frontPad = len( dsplyStr ) * ' '
    N = len( matx ) 
    for i , row in enumerate( matx ):
        M = len( row )
        if i > 0:
            dsplyStr += frontPad + "[ "
        else:
            dsplyStr += "[ "
        for j , col in enumerate( row ):
            if j < M - 1:
                dsplyStr += str( col ).rjust( maxWidth , ' ' ) + " , "
            else:
                dsplyStr += str( col ).rjust( maxWidth , ' ' )
        if i < N - 1:
            dsplyStr += " ]" + endl
        else:
            dsplyStr += " ]"
    dsplyStr += " ]"
    print dsplyStr


# __ End Printing __
