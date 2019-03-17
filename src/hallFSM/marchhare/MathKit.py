#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Template Version: 2017-01-09

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""
MathKit.py , Built on Spyder for Python 2.7
James Watson, 2017 December
Math helper functions including: { trigonometry , statistics }
"""

# ~~ Imports ~~
# ~ Standard ~
import os , operator
from random import random
from math import sqrt , sin , cos , tan , atan2 , asin , acos , atan , degrees , radians , factorial , pi , modf
# ~ Special ~
import numpy as np

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7 # Assume floating point errors below this level
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl = os.linesep # Line separator
pyEq = operator.eq # Default python equality
piHalf = pi/2

# === Mathematics ===

# == General Math Helpers ==

# = Equality Tests =

def eq( op1 , op2 ): 
    """ Return true if op1 and op2 are close enough """
    return abs(op1 - op2) <= EPSILON

def eq_margin( op1 , op2 , margin = EPSILON ): 
    """ Return true if op1 and op2 are within 'margin' of each other, where 'margin' is a positive real number """
    return abs( op1 - op2 ) <= margin

def equality_test_w_margin( margin = EPSILON ):
    """ Return a function that performs an 'eq' comparison with the specified margin """
    def eq_test( op1 , op2 ):
        return eq_margin( op1 , op2 , margin )
    return eq_test

def round_small( val ): 
    """ Round a number to 0 if it is within 'EPSILON' of 0 , otherwise return the number """
    # print "Compare to zero:" ,  val
    return 0.0 if eq( val , 0.0 ) else val

def eq_in_list( pLst , item , eqFunc = eq ):
    """ Return true if there is at least one element in 'pLst' that is equal to 'item' according to 'eqFunc' """
    # NOTE: This function is not recursive
    hasEq = False
    for elem in pLst:
        # dbgLog(1, elem,',', item)
        if eqFunc(elem, item):
            hasEq = True
            break
    return hasEq
    
def eq_list( lst1 , lst2 , eps = EPSILON ): 
    """ Determine if every item in 'lst1' and 'lst2' are close enough """ 
    # NOTE: This function is not recursive
    if len( lst1 ) != len( lst2 ): # If the lists do not have equal length, lists are not equal
        return False
    else: # else lists are of equal length, iterate
        for index in xrange( len( lst1 ) ): # for every item in the list
            if not abs( lst1[ index ] - lst2[ index ] ) <= eps: # test equality, if not equal, then return False
                return False
        return True # Made it through the list without failed tests, return True
    
def index_eq( pList , num , margin = EPSILON ):
    """ Return the index of the first occurrence of 'num' in 'pList' , otherwise return None """
    for index , elem in enumerate( pList ):
        if eq_margin( num , elem , margin ):
            return index
    return None

def product( iterable ):
    """ Return the product of all the elements in 'iterable' """
    # NOTE: This function assumes that 'iterable' has at least one element
    rtnVal = iterable[0]
    for i in xrange( 1 , len( iterable ) ):
        rtnVal *= iterable[i]
    return rtnVal

# _ End Equality _

def wrap_normalize( wrapBounds , number ):
    """ Normalize 'number' to be within 'wrapBounds' on a number line that wraps to 'wrapBounds[0]' when 'wrapBounds[1]' is surpassed 
    and vice-versa """
    span = abs( wrapBounds[1] - wrapBounds[0] )
    if number < wrapBounds[0]:
        return wrapBounds[1] - ( abs( wrapBounds[0] - number ) % span )
    elif number > wrapBounds[1]:
        return ( number % span ) + wrapBounds[0]
    else:
        return number

def within_wrap_bounds( wrapBounds , checkBounds , number ):
    """ Return True if 'number' falls within 'checkBounds' on a wrapped number line defined by 'wrapBounds' , 
    Both directions possible , closed interval """
    # Normalize the number and the checked bounds within wrap bounds so that we can deal with them in a simple way
    number = wrap_normalize( wrapBounds , number )
    checkBounds = [ wrap_normalize( wrapBounds , checkBounds[0] ) , wrap_normalize( wrapBounds , checkBounds[1] ) ] 
    if checkBounds[0] <= checkBounds[1]: # Increasing order indicates a normal , number-line bounds check
        return ( number >= checkBounds[0] ) and ( number <= checkBounds[1] )
    else: # Decreasing order indicates that the checked bounds cross the wrapping boundary
        return ( number <= checkBounds[0] ) or  ( number >= checkBounds[1] )
    
def wrap_bounds_fraction( wrapBounds , checkBounds , number ):
    """ If 'number' is within 'checkBounds' return a number on a scale with checkBounds[0]-->0 and checkBounds[1]-->1 , 
    else return None , Both directions possible """
    # NOTE: 'wrapBounds' MUST be specified in increasing order!
    # Normalize the number and the checked bounds within wrap bounds so that we can deal with them in a simple way
    number = wrap_normalize( wrapBounds , number )
    checkBounds = [ wrap_normalize( wrapBounds , checkBounds[0] ) , wrap_normalize( wrapBounds , checkBounds[1] ) ] 
    if checkBounds[0] <= checkBounds[1]: # Increasing order indicates a normal , number-line bounds check
        if ( number >= checkBounds[0] ) and ( number <= checkBounds[1] ):
            return abs( number - checkBounds[0] ) * 1.0 / abs( checkBounds[1] - checkBounds[0] )
        elif checkBounds[0] == checkBounds[1]:
            return 1
        else:
            return None # Careful , 0 == False!
    else: # Decreasing order indicates that the checked bounds cross the wrapping boundary
        if ( number <= checkBounds[0] ) or ( number >= checkBounds[1] ):
            return abs( number - checkBounds[0] ) * 1.0 / ( abs( checkBounds[1] - wrapBounds[0] ) + abs( wrapBounds[1] - checkBounds[0] ) )
        elif checkBounds[0] == checkBounds[1]:
            return 1
        else:
            return None
        
def sum_abs_diff_lists( op1 , op2 ):
    """ Return the cumulative, absolute, element-wise difference between two lists of equal length """
    # NOTE: This function is not recursive
    return sum( np.abs( np.subtract( op1 , op2 ) ) ) # NOTE: There will be an error if lists not of equal length

def roundint( num ):
    """ Round 'num' to the nearest int """
    return int( round( num ) )

def decimal_part( num ):
    """ Return the decimal part of a number """
    return modf( num )[0]

def copysign( magPart , sgnPart ):
    """ Construct a number that is the magnitude of 'magPart' and the sign of 'sgnPart' """
    # URL , copysign: https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    return abs( magPart ) * np.sign( sgnPart )

# __ End Math Helpers __


# == Trigonometry ==
    
def ver( theta ):
    """ Versine , radians """
    return 1 - cos( theta )

# = Trig in Degrees =

def cosd( angleDeg ):
    """ Return the cosine of the angle specified in degrees """
    return cos( radians( angleDeg ) )

def sind( angleDeg ):
    """ Return the sine of the angle specified in degrees """
    return sin( radians( angleDeg ) )

def tand( angleDeg ): 
    """ Return the tangent of the angle specified in degrees """
    return tan( radians( angleDeg ) )
    
def atan2d( y , x ):
    """ Return the angle, in degrees, of a vector/phasor specified by 'y' and 'x' """
    return degrees( atan2( y , x) )
    
def asind( ratio ):
    """ Return the arcsine of a ratio, degrees """
    return degrees( asin( ratio ) ) 
    
def acosd( ratio ):
    """ Return the arccosine of a ratio, degrees """
    return degrees( acos( ratio ) )
    
def atand( ratio ):
    """ Return the arctangent of a ratio, degrees """
    return degrees( atan( ratio ) )

def verd( theta ):
    """ Versine , degrees """
    return degrees( 1 - cos( theta ) )

# _ End Deg Trig _

# __ End Trig __

# == Statistics ==

def itself( item ): return item # dummy function, return the argument itself # Added to ResearchEnv 2016-09-13
    
def accumulate( pLst , func = itself ):
    """ Return the sum of func(item) for all items in 'pLst'. Return the total number of non-list/tuple items in 'pLst'. Recursive """
    total = 0 # Accumulated total for results of 'func(item)'
    N = 0 # Number of items encountered
    for item in pLst: # for each item in the list
        if isinstance( item , ( list , tuple ) ): # if the list item is itself an iterable
            partTot, partN = accumulate( item , func ) # recur on item
            total += partTot # Accumulate results from greater depth
            N += partN
        else: # else assume item is a number
            total += func( item ) # invoke 'func' on item and accumulate
            N += 1 # count the item
    return total, N # Return the accumulation total and the number of items

def avg( *args ): 
    """ Average of args, where args can be numbers, a list, or nested lists """
    total , N = accumulate( args ) # Accumulate a straight sum
    if N == 0:
        print "avg: Undefined for 0 items!"
        return None
    return float( total ) / N # return mean
    
def variance(*args): # >>> resenv
    """ Variance of args, where args can be numbers, a list of numbers, or nested lists of numbere """
    total , N = accumulate( args ) # calc mean
    if N == 0:
        print "variance: Undefined for 0 items!"
        return None
    print total , 
    mu = float(total) / N
    totSqDiffs , N = accumulate( args , lambda x: ( x - mu )**2 ) # calc the per-item variance
    print totSqDiffs
    return ( 1.0 / N ) * totSqDiffs # return variance

def std_dev( *args ):
    """ Standard deviation of args, where args can be numbers, a list of numbers, or nested lists of numbere """
    var = variance( *args )
    if var == None:
        print "std_dev: Undefined for 0 items!"
        return None
    return sqrt( var )
    
def percent_change( oldVal , newVal ):
    """ Return the precent change from 'oldVal' to 'newVal' , This version avoids div/0 errors """
    if eq( oldVal , 0 ): # If the old value is zero
        if eq( newVal , 0 ):
            return 0.0 # If both values are zero, no change
        else:
            return infty # else div/0 , undefined , return infinity
    return ( newVal - oldVal ) / oldVal * 100.0

# = Dice Rolls =

def flip_weighted( truProb ):
    """ Return True with probability 'truProb' , Otherwise return False """
    return random() <= truProb

def normalize_die( distribution ): 
    """ Given relative odds, return partitions of a distribution on a number line from 0 to 1 """
    # This function assumes that all numbers in the distribution are positive
    total = sum( distribution ) # get the sum of all items
    normed = [ prob / total for prob in distribution ] # normalize the distribution
    accum = 0 # current partition boundary
    die = [] # monotonically increasing partitions for a dice roll
    for prob in normed: # Accumulate the total probability of sampling lesser than or equal to the partition
        accum += prob
        die.append( accum )
    return die # return partitions in [0,1]
    
def roll_die( distribution ): 
    """ Roll a die with a distribution of increasing values ending in 1 , as created by normalize_die """
    sample = random() # sample from a uniform distribution [0,1)
    i = 0 # index of the partition
    while distribution[i] < sample and i < len( distribution ): # while sample is greater than or equal to partition
        i += 1 # advance partition
    return i # This is the index of the least partition greater than the sample
    
def named_odds_to_distribution( oddsDict ):
    """ Unspool the 'oddsDict' into a pairing of ordered names and odds , then normalize the odds into a probability distribution """
    nameList = []
    distList = []
    for name , odds in oddsDict.iteritems():
        nameList.append( name )
        distList.append( odds )
    distList = normalize_die( distList )
    return ( tuple( nameList ) , tuple( distList ) )
    
def roll_for_outcome( namedDist ):
    """ Roll the die on an ordered ( ( NAMES ... ) , ( PROBS ... ) ) tuple with named outcomes and probabilities associated by index """
    return namedDist[0][ roll_die( namedDist[1] ) ] # Return the name corresponding with the index chosen by die roll

def sample_unfrm_real( rMin , rMax ):
    """ Sample from a uniform distribution [ rMin , rMax ) """
    span = abs( rMax - rMin )
    return random() * span + rMin

# _ End Rolls _

def nCr( n , r ): 
    """ Number of combinations for 'n' Choose 'r' """
    return int( factorial( n ) / ( factorial( r ) * factorial( n - r ) ) )

# __ End Stats __

def clamp_val( val , lims ): 
    """ Return a version of val that is clamped to the range [ lims[0] , lims[1] ] , inclusive """
    if val < lims[0]:
        return lims[0]
    if val > lims[1]:
        return lims[1]
    return val

# == Linear Algrebra ==
    
def left_divide( A , b ):
    """ Least-squares solution to an under to over-determined linear system  
        x = left_divide( A , b ) is the solution to dot(A,x) = b , and equivalent to MATLAB A\b """
    x , resid , rank , s = np.linalg.lstsq( A , b )
    return x

# __ End Linalg __

# ___ End Math ___
