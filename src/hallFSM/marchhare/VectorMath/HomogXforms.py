#!/usr/bin/env python
# -*- coding: utf-8 -*-

##### MIT LICENSE BEGIN ##############################################################################
##                                                                                                  ##
##    Copyright 2018 James Watson, University of Colorado Boulder                                   ##
##                                                                                                  ##
##    Permission is hereby granted, free of charge, to any person obtaining a copy of this          ##
##    source file and associated documentation (the "File"), to deal in the File                    ##
##    without restriction, including without limitation the rights to use, copy, modify, merge,     ##
##    publish, distribute, sublicense, and/or sell copies of the File, and to permit persons        ##
##    to whom the File is furnished to do so, subject to the following conditions:                  ##
##                                                                                                  ##
##    The above copyright notice and this permission notice shall be included                       ##
##    in all copies or substantial portions of the File.                                            ##
##                                                                                                  ##
##    THE FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,               ##
##    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                               ##
##    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.                                         ##
##    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,                   ##
##    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,              ##
##    ARISING FROM, OUT OF OR IN CONNECTION WITH THE FILE                                           ##
##    OR THE USE OR OTHER DEALINGS IN THE FILE.                                                     ##
##                                                                                                  ##
##### MIT LICENSE END ################################################################################

import numpy as np

# == Homogeneous Transformations ==
        
def x_rot( theta ):
    """ Return the 3x3 matrix that performs a rotation of 'theta' about the X axis """
    return [ [  1            ,  0            ,  0            ] , 
             [  0            ,  cos( theta ) ,  sin( theta ) ] , 
             [  0            , -sin( theta ) ,  cos( theta ) ] ]
    
def x_trn( theta ):
    """ Return the 3x3 matrix that performs a rotation of 'theta' about the X axis """
    return [ [  1            ,  0            ,  0            ] , 
             [  0            ,  cos( theta ) , -sin( theta ) ] , 
             [  0            ,  sin( theta ) ,  cos( theta ) ] ]

    
def y_rot( theta ):
    """ Return the 3x3 matrix that performs a rotation of 'theta' about the Y axis """
    return [ [  cos( theta ) ,  0            , -sin( theta ) ] , 
             [  0            ,  1            ,  0            ] , 
             [  sin( theta ) ,  0            ,  cos( theta ) ] ]
    
def z_rot( theta ):
    """ Return the 3x3 matrix that performs a rotation of 'theta' about the Z axis """
    return [ [  cos( theta ) ,  sin( theta ) ,  0            ] , 
             [ -sin( theta ) ,  cos( theta ) ,  0            ] , 
             [  0            ,  0            ,  1            ] ]
    
def z_trn( theta ):
    """ Return the 3x3 matrix that performs a rotational transformation of 'theta' about the Z axis """
    return [ [  cos( theta ) , -sin( theta ) ,  0            ] , 
             [  sin( theta ) ,  cos( theta ) ,  0            ] , 
             [  0            ,  0            ,  1            ] ]
    
def rot_matx_ang_axs( theta , k  ):
    """ Return the 3x3 rotation matrix for the given angle 'theta' and axis 'k' """
    
    k = vec_unit( k )
    # ver = marchhare.marchhare.ver
    return [ [ k[0]*k[0]*ver(theta) + cos(theta)      , k[0]*k[1]*ver(theta) - k[2]*sin(theta) , k[0]*k[2]*ver(theta) + k[1]*sin(theta) ] , 
             [ k[1]*k[0]*ver(theta) + k[2]*sin(theta) , k[1]*k[1]*ver(theta) + cos(theta)      , k[1]*k[2]*ver(theta) - k[0]*sin(theta) ] , 
             [ k[2]*k[0]*ver(theta) - k[1]*sin(theta) , k[2]*k[1]*ver(theta) + k[0]*sin(theta) , k[2]*k[2]*ver(theta) + cos(theta)      ] ]
    
def ang_axs_from_rot_matx( R ):
    """ Return the angle 'theta' and axis 'k' for the given 3x3 rotation matrix 'R' """
    # NOTE : This function returns only one solution out of 2 possible , these solution are equivalen with opposite
    theta = acos( ( np.trace( R ) - 1.0 ) / 2.0 )
    k = np.multiply( [ R[2][1] - R[1][2] , 
                       R[0][2] - R[2][0] , 
                       R[1][0] - R[0][1] ] , 0.5 * sin( theta ) )
    return theta , k
    
def homogeneous_Z( zTheta , pos ):
    """ Return the Homogeneous Transformation for the given parameters """
    return np.vstack( ( np.hstack( (  z_rot( zTheta )  , [ [ pos[0] ] , [ pos[1] ] , [ pos[2] ] ]  ) ) ,
                        np.hstack( (  [ 0 , 0 , 0 ]    , [ 1 ]                                     ) ) ) )
    
def homog_ang_axs( theta , k , pos ):
    """ Return the Homogeneous Transformation for the given angle , axis , and position """
    return np.vstack( ( np.hstack( (  rot_matx_ang_axs( theta , k  ) , [ [ pos[0] ] , [ pos[1] ] , [ pos[2] ] ]  ) ) ,
                        np.hstack( (  [ 0 , 0 , 0 ]                  , [ 1 ]                                     ) ) ) )
    
def apply_homog( homogMat , vec3 ):
    """ Apply a homogeneous transformation to a 3D vector """
    return ( np.dot( homogMat , [ vec3[0] , vec3[1] , vec3[2] , 1 ] ) )[:3]

def homog_xform( E , r ): 
    """ Return the combination of rotation matrix 'E' and displacement vector 'r' as a 4x4 homogeneous transformation matrix """
    return np.vstack( ( np.hstack( (  E                              , [ [ r[0] ] , [ r[1] ] , [ r[2] ] ]  ) ) ,
                        np.hstack( (  [ 0 , 0 , 0 ]                  , [ 1 ]                               ) ) ) )

def skew_sym_cross( vecR ):
    """ Return the skew symmetic matrix for the equivalent cross operation: [r_cross][v] = cross( r , v ) """
    return [ [  0       , -vecR[2] ,  vecR[1] ] , 
             [  vecR[2] ,  0       , -vecR[0] ] ,
             [ -vecR[1] ,  vecR[0] ,  0       ] ]

def pos_from_xform( xform ):
    """ Get the position vector from the homogeneous transformation """
    return [ xform[0][3] , xform[0][1] , xform[0][2] ]

def get_basis_vectors_for_xform( xform ):
    """ Return the basis vector for the transformation """
    xBasis = apply_homog( xform , [1,0,0] )
    yBasis = apply_homog( xform , [0,1,0] )
    zBasis = apply_homog( xform , [0,0,1] )
    return xBasis , yBasis , zBasis
        
# __ End Homogeneous __
