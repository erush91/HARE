#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "rospy_helpers.py"
__version__  = "2019.01" 
__desc__     = "Utilities for writing rospy nodes"
"""
James Watson , Template Version: 2018-05-14
Built on Wing 101 IDE for Python 2.7

Dependencies: rospy , numpy
"""


"""  
~~~ Developmnent Plan ~~~
[ ] ITEM1
[ ] ITEM2
"""

# === Init Environment =====================================================================================================================
# ~~~ Prepare Paths ~~~
import sys, os.path
SOURCEDIR = os.path.dirname( os.path.abspath( __file__ ) ) # URL, dir containing source file: http://stackoverflow.com/a/7783326
PARENTDIR = os.path.dirname( SOURCEDIR )
# ~~ Path Utilities ~~
def prepend_dir_to_path( pathName ): sys.path.insert( 0 , pathName ) # Might need this to fetch a lib in a parent directory

# ~~~ Imports ~~~
# ~~ Standard ~~
from math import pi , sqrt
# ~~ Special ~~
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
# ~~ Local ~~

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === FUNCTIONS ============================================================================================================================


# == Time ==

def get_ROS_ntime_secs():
    """ The the current time in decimal seconds down to decimal nanosecond """
    now = rospy.get_rostime()
    rtnNum = now.secs + now.nsecs / 1e10 
    return rtnNum

# __ End Time __


# == Geo Messages ==

def unpack_ROS_xform( xform ):
    """ Unpack the ROS transform message into position and orientation """
    posn = [ xform.transform.translation.x , xform.transform.translation.y , xform.transform.translation.z ] 
    ornt = [ xform.transform.rotation.x    , xform.transform.rotation.y    , xform.transform.rotation.z    , xform.transform.rotation.w ]
    return posn , ornt

# __ End Geo __


# == Math Helpers ==

def eq_margin( op1 , op2 , margin = EPSILON ):
    """ Return true if op1 and op2 are within 'margin' of each other, where 'margin' is a positive real number """
    return abs( op1 - op2 ) <= margin
    
def clamp_val( val , lims ):
    """ Return a version of val that is clamped to the range [ lims[0] , lims[1] ] , inclusive """
    if val < lims[0]:
        return lims[0]
    if val > lims[1]:
        return lims[1]
    return val

# __ End Math __


# ___ END FUNC _____________________________________________________________________________________________________________________________


# === CLASSES ==============================================================================================================================





    

# ___ END CLASS ____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare __________________________________________________
