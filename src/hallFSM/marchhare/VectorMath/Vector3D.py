#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""
Vector3D.py , Built on Spyder 2 for Python 2.7
James Watson, 2017 March
Vector operations and geometry in 3 dimensions
"""

# ~~ Imports ~~
# ~ Standard ~
from math import atan2 , acos , cos , sin , sqrt , asin , radians , pi
# ~ Special ~
import numpy as np
# ~ Local ~
# import marchhare.marchhare

from marchhare.Vector import vec_mag , vec_unit , vec_proj , np_add , vec_round_small , vec_angle_between , vec_eq , vec_copy , vec_linspace , vec_NaN
from HomogXforms import *
from marchhare.MathKit import ver , eq , round_small , decimal_part , copysign
from marchhare.marchhare import format_dec_list , incr_max_step


# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
# endl = os.linesep # Line separator
# pyEq = operator.eq # Default python equality
piHalf = pi/2
DISPLAYPLACES = 3

# === 3D Geometry ===

# == Spherical Coordinates ==

def cart_2_sphr( cartCoords ):
    """ Convert Cartesian coordinates to spherical coordinates , [ x , y , z ] --> [ r , \theta , \phi ] """ 
    # URL: https://en.wikipedia.org/wiki/Spherical_coordinate_system#Coordinate_system_conversions
    r = vec_mag( cartCoords )
    return [
        r , # ------------------------------------ r      , magnitude of the vector
        atan2( cartCoords[1] , cartCoords[0] ) , # \theta , Angle in the XY plane , with X+ as \theta = 0
        acos( cartCoords[2] / r ) # -------------- \phi   , declination from the vertical Z+
    ]

def sphr_2_cart( sphrCoords ):
    """ Convert spherical coordinates to Cartesian coordinates , [ r , \theta , \phi ] --> [ x , y , z ] """
    # URL: http://keisan.casio.com/exec/system/1359534351
    return [
        sphrCoords[0] * cos( sphrCoords[1] ) * sin( sphrCoords[2] ) , # x
        sphrCoords[0] * sin( sphrCoords[1] ) * sin( sphrCoords[2] ) , # y
        sphrCoords[0] * cos( sphrCoords[2] ) # ------------------------ z
    ]

def cart_2_sphr_YUP( cartCoords ):
    """ Convert Cartesian coordinates to spherical coordinates , [ x , y , z ] --> [ r , \theta , \phi ] , Y+ = UP """ 
    # URL: https://en.wikipedia.org/wiki/Spherical_coordinate_system#Coordinate_system_conversions
    r = vec_mag( cartCoords )
    return [
        r , # ------------------------------------ r      , magnitude of the vector
        atan2( cartCoords[0] , cartCoords[2] ) , # \theta , Angle in the ZX plane , with Z+ as \theta = 0
        acos( cartCoords[1] / r ) # -------------- \phi   , declination from the vertical Y+
    ]

def sphr_2_cart_YUP( sphrCoords ):
    """ Convert spherical coordinates to Cartesian coordinates , [ r , \theta , \phi ] --> [ x , y , z ] , Y+ = UP """
    # URL: http://keisan.casio.com/exec/system/1359534351
    return [
        sphrCoords[0] * sin( sphrCoords[1] ) * sin( sphrCoords[2] ) , # x
        sphrCoords[0] * cos( sphrCoords[2] ) , # ---------------------- y
        sphrCoords[0] * cos( sphrCoords[1] ) * sin( sphrCoords[2] ) # - z
    ]
    
# = Mouselook Angles =

def cart_2_radPtcYaw( cartCoords ):
    """ Convert a Cartesian vector to [ radius , pitch , yaw ] for the purpose of mouselook """
    r , theta , phi = cart_2_sphr( cartCoords )
    return [ r , piHalf - phi , theta ]

def radPtcYaw_2_cart( radPtcYaw ):
    """ Convert [ radius , pitch , yaw ] vector to a Cartesian vector [ x , y , z ] for the purpose of mouselook """
    return sphr_2_cart( [ radPtcYaw[0] , radPtcYaw[2] , piHalf - radPtcYaw[1] ] ) # [ r , \theta , \phi ] --> [ x , y , z ]

def cart_2_radPtcYaw_YUP( cartCoords ):
    """ Convert a Cartesian vector to [ radius , pitch , yaw ] for the purpose of mouselook , Y+ = UP """
    r , theta , phi = cart_2_sphr_YUP( cartCoords )
    return [ r , piHalf - phi , theta ]

def radPtcYaw_2_cart_YUP( radPtcYaw ):
    """ Convert [ radius , pitch , yaw ] vector to a Cartesian vector [ x , y , z ] for the purpose of mouselook , Y+ = UP """
    return sphr_2_cart_YUP( [ radPtcYaw[0] , radPtcYaw[2] , piHalf - radPtcYaw[1] ] ) # [ r , \theta , \phi ] --> [ x , y , z ]

# = End Mouselook =

# == End Spherical ==

def vec_proj_to_plane( vec , planeNorm ):
    """ Return a vector that is the projection of 'vec' onto a plane with the normal vector 'planeNorm', using numpy """
    # URL, projection of a vector onto a plane: http://www.euclideanspace.com/maths/geometry/elements/plane/lineOnPlane/
    # NOTE: This function assumes 'vec' and 'planeNorm' are expressed in the same bases
    if vec_eq( vec , [ 0 , 0 , 0 ] ):
        return [ 0 , 0 , 0 ]
    else:
        projDir = vec_unit( np.cross( np.cross( planeNorm , vec ) , planeNorm ) ) # direction of the projected vector, normalize
        projMag = vec_proj( vec , projDir ) # magnitude of the vector in the projected direction
        return np.multiply( projDir , projMag ) # scale the unit direction vector to the calculated magnitude and return

def pnt_proj_to_plane( queryPnt , planePnt , normal ):
    """ Return a point that is 'queryPnt' projected onto a plane with a given 'normal' and passing through 'planePnt' """
    relPnt = np.subtract( queryPnt , planePnt ) # Compute the vector offset from the arbitrary plane point to the point under scrutiny
    offset = vec_proj_to_plane( relPnt , normal )
    return np.add( planePnt , offset )

def vec_dist_to_plane( queryPnt , planePnt , normal ):
    """ Return the distance from 'queryPnt' to a plane with 'normal' and that contains 'planePnt' (any point on the plane) """
    relPnt = np.subtract( queryPnt , planePnt ) # Compute the vector offset from the arbitrary plane point to the point under scrutiny
    return vec_proj( relPnt , normal ) # Projection of the relative vector to the normal is the shortest distance to the plane

def vec_from_pnt_to_plane( queryPnt , planePnt , normal ):
    """ Return the vector that points from 'queryPnt' to that point projected on a plane defined be 'planePnt' and 'normal' """
#    print "DEBUG , Query:" , queryPnt , ", Plane Point:" , planePnt , ", Normal:" , normal
#    print "DEBUG , Projected query:" , pnt_proj_to_plane( queryPnt , planePnt , normal )
    return np.subtract( pnt_proj_to_plane( queryPnt , planePnt , normal ) , queryPnt )

# URL , Intersection of ray and triangle: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
def ray_intersects_triangle( rayOrigin , rayVector , CCWtriCoords ):
    """ Möller–Trumbore intersection algorithm for whether a ray intersects a triangle in R3 , 'rayVector' can be any length 
        return: [ bool: Ray intersects triangle , R3: Line intersection point , float: signed distance from ray origin to intersection ] """
    # Note: If there is no line intersection , then the second and third elements will be populated with NaN
    vertex0 = CCWtriCoords[0]
    vertex1 = CCWtriCoords[1]
    vertex2 = CCWtriCoords[2]
        
    edge1 = np.subtract( vertex1 , vertex0 )
    edge2 = np.subtract( vertex2 , vertex0 )
    h = np.cross( rayVector , edge2 ) # rayVector.crossProduct(edge2);
    a = np.dot( edge1 , h ) # edge1.dotProduct(h);
    if ( a > -EPSILON ) and ( a < EPSILON ):
        return [ False , vec_NaN( 3 ) , float('NaN') ] # No intersection , Return False and in invalid vec
    f = 1.0 / a;
    s = np.subtract( rayOrigin , vertex0 )
    u = f * np.dot( s , h ) # u = f * (s.dotProduct(h));
    if ( u < 0.0 ) or ( u > 1.0 ):
        return [ False , vec_NaN( 3 ) , float('NaN') ] # No intersection , Return False and in invalid vec
    q = np.cross( s , edge1 ) # s.crossProduct(edge1);
    v = f * np.dot( rayVector , q ) # f * rayVector.dotProduct(q);
    if ( v < 0.0 ) or ( u + v > 1.0 ):
        return [ False , vec_NaN( 3 ) , float('NaN') ] # No intersection , Return False and in invalid vec
    # At this stage we can compute t to find out where the intersection point is on the line.
    t = f * np.dot( edge2 , q ) # f * edge2.dotProduct(q);
    outIntersectionPoint = np.add( rayOrigin , np.multiply( rayVector , t ) )
    intersectionMag      = np.dot( np.subtract( outIntersectionPoint , rayOrigin ) , vec_unit( rayVector ) )
    if t > EPSILON: # ray intersection
        return [ True  , outIntersectionPoint , intersectionMag ]
    else: # This means that there is a line intersection but not a ray intersection.
        return [ False , outIntersectionPoint , intersectionMag ]

def segment_intersects_triangle( segment , CCWtriCoords ):
    """ Möller–Trumbore intersection algorithm for whether a line segment intersects a triangle in R3 
        return: [ bool: Segment intersects triangle , R3: Line intersection point , float: signed distance from segment[0] to intersection ] """
    # Note: If there is no line intersection , then the second and third elements will be populated with NaN
    # This function just runs the Möller–Trumbore intersection algorithm and determines if the intersection is within the segment
    rayOrigin = segment[0]
    rayVector = np.subtract( segment[1] , segment[0] )
    segLen    = vec_mag( rayVector )
    [ intersectBool , intPnt , intMag ] = ray_intersects_triangle( rayOrigin , rayVector , CCWtriCoords )
    if intersectBool:
        if ( intMag >= 0.0 ) and ( intMag <= segLen ):
            return [ True , intPnt , intMag ]
        else:
            return [ False , intPnt , intMag ]
    else:
        return [ False , intPnt , intMag ]

# == Mesh VFN ==

class MeshVFN:
    """ Container class for graphics-style tri-mesh """
    def __init__( self , V , F , N ):
        self.V = V
        self.F = F
        self.N = N
    
def segment_intersects_VFN( segment , V , F , N ):
    """ Determine if 'segment' intersects any of the triangles defined by the graphics-style V-F-N matrices 
        return: [ bool: Does the segment intersect any F? , float: Greatest penetration depth                          , 
                  [ R3 ]: List of intersection points     , [ float ]: List of penetration depth for each intersection ] """
    # This function just runs the Möller–Trumbore intersection algorithm for each mesh facet
    intersectionPoints = [] # -- All of the intersections encountered
    penDepths          = [] # -- All of the penetration depths recorded
    segBool            = False # Flag for collision
    greatestDepth      = 0.0 # - Greatest depth penetrated by any segment collision
    # For each triangle in the mesh
    for i , f_i in enumerate( F ):
        [ intersectBool , intPnt , intMag ] = segment_intersects_triangle( segment , [ V[ f_i[0] ] , V[ f_i[1] ] , V[ f_i[2] ] ] ) # Collision check
        if intersectBool: # If there is an intersection , log intersection data 
            segBool = True # Flag intersection
            intersectionPoints.append( intPnt ) # Log the intersection point
            penDepths.append( abs( min( vec_dist_to_plane( segment[0] , V[ f_i[0] ] , N[ i ] ) ,     # Endpoints must represent penetration extremes
                                        vec_dist_to_plane( segment[1] , V[ f_i[0] ] , N[ i ] ) ) ) ) # for a straight segment
            greatestDepth = max( greatestDepth , penDepths[-1] ) # keep track of the deepest penetration
    return [ segBool , greatestDepth , intersectionPoints , penDepths ] # Load and return the intersection info

# __ End VFN __

# == Vector Spaces , R3 ==

def transform_by_bases( vec_A , xBasis_B , yBasis_B , zBasis_B ):
    """ Transform a 'vec' in basis A to basis B, with 'x/y/zBasis' expressed in B """
    # NOTE: This function assumes that all basis vectors are unit vectors, otherwise a non-unity scaling will be applied to the result
    return np_add( np.multiply( xBasis_B , vec_A[0] ) , # x component
                   np.multiply( yBasis_B , vec_A[1] ) , # y component
                   np.multiply( zBasis_B , vec_A[2] ) ) # z component
                   
def transform_to_frame( vec_A , origin , xBasis_B , yBasis_B , zBasis_B ):
    """ Transform a 'vec' in basis A to basis B, with 'x/y/zBasis' expressed in B """
    return np.add( origin , transform_by_bases( vec_A , xBasis_B , yBasis_B , zBasis_B ) )
    
def point_basis_change( point , origin , xBasis , yBasis , zBasis ):
    """ Express a 'point' in a new basis, according to 'origin', 'xBasis', 'yBasis', 'zBasis' (all param coordinates in old basis) """
    offset = np.subtract( point , origin )
    # NOTE: This is probably faster as a matrix operation
    return ( vec_proj( offset , xBasis ) , vec_proj( offset , yBasis ) , vec_proj( offset , zBasis ) )
 
def check_orthonormal(basis1,basis2,basis3):
    """ Return True if bases are mutually orthogonal, False otherwise , does not indicate which criterion fails """
    orthogonal = round_small( vec_proj(basis1,basis2) ) == 0 and round_small( vec_proj(basis1,basis3) ) == 0 and round_small( vec_proj(basis2,basis3) ) == 0
    # dbgLog(-1, round_small( vec_proj(basis1,basis2) )  , round_small( vec_proj(basis1,basis3) )  , round_small( vec_proj(basis2,basis3) ) )
    normal = eq( vec_mag(basis1) , 1) and eq( vec_mag(basis2) , 1) and eq( vec_mag(basis3) , 1)
    # dbgLog(-1, vec_mag(basis1)  , vec_mag(basis2) , vec_mag(basis3) )
    return orthogonal and normal
       
# == End Spaces ==

# == Rotation Matrices ==

class RotnMatx(object):
    """ A wrapper for a rotation matrix """
    # NOTE: This class is bookkeeping to maintain the row-vector convention in Vector and does not stem from a great technical need
    
    def __init__( self , multiDimList ):
        """ Convert a 2D Python list into a numpy matrix """
        self.matx = np.array( multiDimList )
        
    def apply_to( self , vec ):
        """ Apply a rotation to a row vector and return a row vector """
        # Make the row vector into a column, multiply, flatten to row vector, and ruturn the resultant row vector
        return np.reshape( self.matx.dot( np.reshape( vec , (3,1) ) ) , 3 ) 
    
def rotation_matrix_from_angle_spec( specification , op1 , op2 , op3 ):
    if specification in ( 'rpy' , 'fxyz' ):
        psi    = op1
        theta  = op2
        phi    = op3
        return RotnMatx( 
            [ [  cos(phi)*cos(theta) ,  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) ,  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi) ] ,
              [  sin(phi)*cos(theta) ,  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi) ,  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi) ] ,
              [ -sin(theta)          ,  cos(theta)*sin(psi)                              ,  cos(theta)*cos(psi)                              ] ] 
        )
    else:
        raise ValueError( "rotation_matrix_from_angle_spec: The given specification, " + str(specification) + " , was not recognized!" )
        
# == End R Matrices ==
        
# == class Quaternion ==

# TODO: See if the matrix formulation of Quaternions is faster! , Matrices are good for parallel operations
        
class Quaternion(object):
    """ Quaternion representation for 3D rotations """
    
    def __init__(self, q_0, q_1, q_2, q_3): # ( w , x , y , z )
        """ Return a quaternion specified by the scalar part followed by the components of the vector part """
        self.sclr = q_0
        self._vctr = np.array( [ q_1 , q_2 , q_3 ] )
        
    @property
    def vctr(self):
        """ Because np.array([1,2,3])[:] returns a reference to the array instead of a copy of it! """
        return self._vctr.copy() 
    
    @vctr.setter
    def vctr(self, arr_or_list):
        """ Set the vector component, make sure it is an 'np.array' """
        self._vctr = np.array( arr_or_list )
        
    def get_copy( self ):
        """ Return an identical Quaternion """
        return Quaternion( self.sclr , *self._vctr )
        
    def get_WXYZ(self):
        """ Return the scalar followed by vector components, as a flat list """
        return [ self.sclr , self._vctr[0] , self._vctr[1] , self._vctr[2] ]
        
    def get_XYZW(self):
        """ Return the vector components followed by the scalar, as a flat list """
        return [ self._vctr[0] , self._vctr[1] , self._vctr[2] , self.sclr ]
        
    @staticmethod
    def from_XYZW_quat(quatArr):
        """ Return a Quaternion from the array [ qx , qy , qz , qw ] """
        return Quaternion( quatArr[3] , quatArr[0] , quatArr[1] , quatArr[2] ) # Basically rearranging the elements
     
    @staticmethod
    def k_rot_to_Quat(k, rot):
        """ Return a quaternion representing a rotation of 'rot' about 'k' 
        k :   3D vector that is the axis of rotation
        rot : rotation angle expressed in radians  """
        k = vec_unit(k)
        return Quaternion( cos(rot/2), \
                           k[0] * sin(rot/2), \
                           k[1] * sin(rot/2), \
                           k[2] * sin(rot/2) )
                     
    def set_k_rot(self, k, rot):
        """ Set the quaternion components based on an axis-angle (radians) specification of rotation """
        k = vec_unit(k)
        self.sclr = cos(rot/2)
        self._vctr = np.array([ k[0] * sin(rot/2) , \
                                k[1] * sin(rot/2) , \
                                k[2] * sin(rot/2) ])
                                
    def get_k_rot(self):
        """ Retrieve the rotation axis 'k' and 'rot' angle from the Quaternion """
        # URL, Quaternion to Axis-Angle: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
        if self.sclr > 1:
            self.normalize() # Prevent imaginary results in the case that it is not a unit quaternion
        if self.sclr == 1: # if the scalar component is 1, assume it is a no-turn quat 
            k = [1,0,0] # no rotation about an arbitrary vector
            rot = 0
        else:
            k = [ self._vctr[0] / sqrt( 1 - self.sclr ** 2 ) , 
                  self._vctr[1] / sqrt( 1 - self.sclr ** 2 ) , 
                  self._vctr[2] / sqrt( 1 - self.sclr ** 2 ) ]
            rot = 2 * acos( self.sclr )
        return k , rot
       
    @staticmethod              
    def compose_rots( q1 , q2 ):
        """ Return quaternion that is the rotation 'q1' followed by rotation q2 """
        scalar = (q1.sclr * q2.sclr) - np.dot(q1._vctr , q2._vctr)
        vector = np.multiply( q1._vctr , q2.sclr) + \
                 np.multiply( q2._vctr , q1.sclr) + \
                 np.cross(q2._vctr , q1._vctr)
        return Quaternion(scalar, vector[0], vector[1], vector[2])
        
    @staticmethod
    def serial_rots( *rotQuats ):
        """ Return quaternion that is the application of rotations in order passed to function """
        # NOTE: This is the same as applying successived rotations about vectors defined in the fixed frame
        if len(rotQuats) > 1: # If there were more than two Quaternions passed, compose them in succession
            temp = Quaternion.compose_rots(rotQuats[0], rotQuats[1]) # there must be at least two args
            for rotDex in range(2, len(rotQuats)): # if there are more, then compose them onto the final rotation
                temp = Quaternion.compose_rots(temp, rotQuats[rotDex])
        elif len(rotQuats) > 0: # If there was only Quaternion passed, return it
            temp = rotQuats[0]
        else: # no arguments passed, return the no-rotation Quaternion
            temp = Quaternion.no_turn_quat()
        return temp
            
    def apply_to(self, vec):
        """ Apply quaternion to a 3D vector as a rotation, return transformed vector """
        # dbgLog(-1, "Quaternion.apply_to: Got vector", vec )
        term1 = np.multiply( vec , ( self.sclr ** 2 - np.dot(self._vctr , self._vctr) ) )
        term2 = np.multiply( np.cross(self._vctr , vec) , ( 2 * self.sclr ) )
        term3 = np.multiply( self._vctr , ( 2 * np.dot( self._vctr , vec ) ) )
        temp = term1 + term2 + term3
        # dbgLog(-1, str(temp) )
        # vec_round_small(temp) # Round float remnants to zero
        return temp
        
    def __str__(self):
        """ Represent the Quat as a string, mostly for debugging """
        return '[ ' + str(self.sclr) + ' , ' + format_dec_list( self._vctr , places = 4 ) + ' ]' # There will be major deserialization headaches unless '_vctr' is printed correctly
        
    @staticmethod
    def principal_rot_Quat( basis1 , basis2 , basis3 ):
        """ For new 'bases' expressed in the old, find the quaternion that rotates the old onto the new """
        C = [ vec_unit( basis1 ) , vec_unit( basis2 ) , vec_unit( basis3 ) ] # Asssumed normalization enough times that I'll just do it
        inside = 0.5 * ( C[0][0] + C[1][1] + C[2][2] - 1 )
        # print "DEBUG , 0.5 * ( C[0][0] + C[1][1] + C[2][2] - 1 ):" , inside
        if abs( inside ) > 1:
            # print "DEBUG , Applying correction!" , inside , "-->" ,
            inside = -decimal_part( inside )
            # print inside
        Phi = acos( inside )
        e1 = 0.5 * sin( Phi ) * ( C[1][2] - C[2][1] )
        e2 = 0.5 * sin( Phi ) * ( C[2][0] - C[0][2] )
        e3 = 0.5 * sin( Phi ) * ( C[0][1] - C[1][0] )
        return Quaternion.k_rot_to_Quat( [ e1 , e2 , e3 ] , Phi )
     
    @staticmethod
    def bases_to_Quat( xBasis , yBasis , zBasis ):
        """ For new bases expressed in the old, find the quaternion that rotates the old onto the new """
        # FIXME , THIS IS WRONG , COMPARE TO 'principal_rot_Quat' !! FIXME , THIS IS WRONG , COMPARE TO 'principal_rot_Quat' !!
        Q = [ vec_unit( xBasis ) , vec_unit( yBasis ) , vec_unit( zBasis ) ]
        [ [ Qxx , Qxy , Qxz ] , 
          [ Qyx , Qyy , Qyz ] , 
          [ Qzx , Qzy , Qzz ] ] = Q
        t = Qxx + Qyy + Qzz 
        r = sqrt( 1 + t )
        w = 0.5 * r
        x = copysign( 0.5 * sqrt( 1 + Qxx - Qyy - Qzz ) , ( Qzy - Qyz ) )
        y = copysign( 0.5 * sqrt( 1 - Qxx + Qyy - Qzz ) , ( Qxz - Qzx ) )
        z = copysign( 0.5 * sqrt( 1 - Qxx - Qyy + Qzz ) , ( Qyx - Qxy ) )
        temp = Quaternion( w , x , y , z )
        temp.normalize()
        return temp
    
    def norm( self ):
        """ Return the norm of the Quaternion """
        return sqrt( self.sclr ** 2 + self._vctr[0] ** 2 + self._vctr[1] ** 2 + self._vctr[2] ** 2 )

    def normalize(self):
        """ Normalize the Quaternion, destructive """
        self.scale( 1 / self.norm() )
        
    def round_small( self ):
        """ Round the very small components and renormalize """
        self.sclr = round_small( self.sclr )
        self.vctr = vec_round_small( self.vctr )
        self.normalize()
       
    @staticmethod
    def conjugate( pQuat ):
        """ Return the conjugate of the Quaternion """
        return Quaternion( pQuat.sclr , -pQuat._vctr[0] , -pQuat._vctr[1] , -pQuat._vctr[2] )
    
    def scale(self, operand):
        """ Multiply all the components of the Quaternion by 'operand', destructive """
        self.sclr = self.sclr * operand
        self._vctr = np.multiply( self._vctr , operand )
    
    @staticmethod
    def inverse( pQuat ):
        """ Return the inverse of the Quaternion """
        # URL, Inverse of a Quaternion: http://mathworld.wolfram.com/Quaternion.html
        temp = Quaternion.conjugate(pQuat)
        temp.scale( 1.0 / temp.norm() ** 2 )
        return temp
        
    @staticmethod
    def eq( p , q ):
        """ Determine if two quaterions are equal enough """
        return eq( p.sclr , q.sclr ) and vec_eq( p._vctr , q._vctr )
        
    @staticmethod
    def diff_from_to( p , q ):
        """ Return the Quaternion that rotates Quaternion 'p' to Quaternion 'q' """
        if Quaternion.eq( p , q ): # if the Quaternions are equal the operation will fail, so return a no-turn quat
            return Quaternion.no_turn_quat()
        else: # else Quaternions are not equal, proceed as ususal
            return Quaternion.compose_rots( Quaternion.inverse(p) , q )
    
    @staticmethod
    def blend_k_rot( k , rot , num ):
        """ Return a list of rotations about 'k' from 0 to 'rot' (inclusive) in 'num' steps """
        quatLst = []
        for blendRot in np.linspace( 0 , rot , num ):
            quatLst.append( Quaternion.k_rot_to_Quat( k , blendRot ) )
        return quatLst
        
    @staticmethod
    def blend_orientations( p , q , num ):
        """ Return a list of 'num' orientations that range from 'p' to 'q' (inclusive) along the shortest turn """
        k , rot = Quaternion.diff_from_to( p , q ).get_k_rot()
        quatLst = []
        for blendRot in np.linspace( 0 , rot , num ):
            quatLst.append( Quaternion.compose_rots( p , Quaternion.k_rot_to_Quat( k , blendRot ) ) )
        return quatLst
        
    @staticmethod
    def no_turn_quat():
        return Quaternion.k_rot_to_Quat( [0,0,1] , 0 )
        
    @staticmethod
    def shortest_btn_vecs( v1 , v2 ):
        """ Return the Quaternion representing the shortest rotation from vector 'v1' to vector 'v2' """
#        print "DEBUG , Got vectors:     " , v1 , v2
#        print "DEBUG , Crossed vectors: " , np.cross( v1 , v2 )
#        print "DEBUG , Crossed unit vec:" , vec_unit( np.cross( v1 , v2 ) )
#        print "DEBUG , Angle between:   " , vec_angle_between( v1 , v2 )
        angle = vec_angle_between( v1 , v2 )
        if eq( angle , 0.0 ):
            return Quaternion.no_turn_quat()
        elif eq( angle , pi ):
            # If we are making a pi turn, then just pick any axis perpendicular to the opposing vectors and make the turn
            # The axis that is picked will result in different poses
            return Quaternion.k_rot_to_Quat( vec_unit( np.cross( v1 , [ 1 , 0 , 0 ] ) ) , angle )
        else:
            return Quaternion.k_rot_to_Quat( vec_unit( np.cross( v1 , v2 ) ) , angle )
        
    def get_bases(self):
        """ Return the basis vectors that represent the orientation in the parent frame """
        return [ self.apply_to( [1,0,0] ) , self.apply_to( [0,1,0] ) , self.apply_to( [0,0,1] ) ]
    
    def serialize(self):
        """ Return a nested list structure of the form [ q0 , q1 , q2 , q3 ] """
        return self.get_WXYZ()
        
    @staticmethod
    def deserialize( struct ):
        """ Recover a Quaternion from a nested list structure of the form [ q0 , q1 , q2 , q3 ] """
        return Quaternion( *struct )
    
    @staticmethod
    def deserialize_nested( struct ):
        """ Recover a Quaternion from a nested list structure of the form [ q0 , [ q1 , q2 , q3 ] ] """
        return Quaternion( struct[0] , *struct[1] )    
    
    def get_euler_angles(self): # TODO: TEST
        """ Return the Euler angles corresponding to this quaternion """
        # URL , Convert quaternion to Euler angles: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        # NOTE: No effort is made to avoid gimbal lock
        return [ atan2( 2 * (self.sclr * self.vctr[0] + self.vctr[1] * self.vctr[2] ) , 1 - 2 * ( self.vctr[0] ** 2 + self.vctr[1] ** 2 ) ) , 
                 asin(  2 * (self.sclr * self.vctr[1] - self.vctr[2] * self.vctr[0] ) ) , 
                 atan2( 2 * (self.sclr * self.vctr[2] + self.vctr[0] * self.vctr[1] ) , 1 - 2 * ( self.vctr[1] ** 2 + self.vctr[2] ** 2 ) ) ]
    
    @staticmethod
    def angles_to_Quat( specification , op1 , op2 , op3 ):
        try:
            return Quaternion.xformLookup[ specification ]( op1 , op2 , op3 )
        except KeyError:
            raise KeyError( "Quaternion.angles_to_quat: Angle specification " + str(specification) + " was not recognized!" )
    
    @staticmethod
    def fixed_XYZ_to_Quat( gamma , beta , alpha ):
        """ Return a Quaterion that corresponds to the FIXED rotations XYZ, also known as RPY """
        return Quaternion.serial_rots( 
            Quaternion.k_rot_to_Quat( [1,0,0] , gamma ) ,
            Quaternion.k_rot_to_Quat( [0,1,0] , beta  ) ,
            Quaternion.k_rot_to_Quat( [0,0,1] , alpha )    
        )

Quaternion.xformLookup = { 'rpy' : Quaternion.fixed_XYZ_to_Quat , 
                          'fxyz' : Quaternion.fixed_XYZ_to_Quat }        
        
    
# = class Rotation =
        
class Rotation(Quaternion):
    """ Represents a rotation, specified in axis-angle notation, but stored as a quaternion so that 
    it retains the functions and operations available to a Quaternion
    Note: The axis vector is always converted to a unit vector """
    
    def __init__(self, pK, pTheta, pMinLimit = None, pMaxLimit = None):
        super(Rotation, self).set_k_rot(pK, pTheta) 
        self.k = vec_unit(pK)
        self.theta = pTheta
        self.hasLimits = pMinLimit != None or pMaxLimit != None
        self.minLimit = pMinLimit
        self.maxLimit = pMaxLimit
        
    def set_theta(self, pTheta):
        """ Update the angle and compute the quaternion """
        if self.hasLimits: # check that disallowed value was not passed, and raise error if one was
            if self.minLimit != None and pTheta < self.minLimit:
                raise ValueError("Rotation.set_theta : Attempt to set theta lesser than limit " + str(self.minLimit) )
            if self.maxLimit != None and pTheta > self.maxLimit:
                raise ValueError("Rotation.set_theta : Attempt to set theta greater than limit " + str(self.maxLimit) )
        self.theta = pTheta # no errors raised, assign value
        self.set_k_rot( self.k , self.theta ) # compute the quaternion
        
    def set_k(self, pK):
        """ Update the vector and compute the quaternion 
        This function assumes that pK is a Vector """
        self.k = vec_unit(pK)
        self.set_k_rot( self.k , self.theta ) # compute the quaternion
        
    def set_limits(self, pMinLimit = None, pMaxLimit = None):
        """ Set limits on 'self.theta' and set the limit flag """
        self.hasLimits = pMinLimit != None or pMaxLimit != None
        self.minLimit = pMinLimit
        self.maxLimit = pMaxLimit
        
class Rotationd(Rotation):
    """ Represents a rotation, specified in axis-angle notation (degrees), but stored as a quaternion 
    Note: The axis vector is always converted to a unit vector """
    
    def __init__(self, pK, pTheta, pMinLimit = None, pMaxLimit = None):
        """ Convert params to radians and init a Rotation """
        super(Rotationd,self).__init__(pK, radians(pTheta), \
                                           pMinLimit if pMinLimit == None else radians(pMinLimit), \
                                           pMaxLimit if pMaxLimit == None else radians(pMaxLimit) )
                                           
    def set_thetad(self, pThetad):
        """ Take a theta in degrees, convert to radians, and store """
        super(Rotationd,self).set_theta(radians(pThetad))
        
    def set_limitsd(self, pMinLimit = None, pMaxLimit = None):
        """ Take limits in degrees, convert to radians, and store """
        super(Rotationd, self).set_limits( pMinLimit if pMinLimit == None else radians(pMinLimit), \
                                           pMaxLimit if pMaxLimit == None else radians(pMaxLimit) )
        
# = End Rotation =

def principal_rotation_test():
    """ Test of 'Quaternion.principal_rot_Quat' """
    newBasisX = vec_unit([ 0.0 , -1.0 ,  1.0])
    newBasisY = vec_unit([ 1.0 ,  0.0 ,  0.0])
    newBasisZ = vec_unit([ 0.0 ,  1.0 ,  1.0])
    if check_orthonormal( newBasisX , newBasisY , newBasisZ ):
        print "New bases are orthonormal"
        principalQuat = Quaternion.principal_rot_Quat(newBasisX,newBasisY,newBasisZ)
        testBasisX = principalQuat.apply_to( [1,0,0] )
        testBasisY = principalQuat.apply_to( [0,1,0] )
        testBasisZ = principalQuat.apply_to( [0,0,1] )
        print "New basis X" , newBasisX , "and transformed basis X" , testBasisX , "are equal:" , vec_eq(newBasisX,testBasisX)
        print "New basis Y" , newBasisY , "and transformed basis Y" , testBasisY , "are equal:" , vec_eq(newBasisY,testBasisY)
        print "New basis Z" , newBasisZ , "and transformed basis Z" , testBasisZ , "are equal:" , vec_eq(newBasisZ,testBasisZ)
    else:
        print "New bases are not orthonormal"
#principal_rotation_test()

# == End Quaternion ==


""" NOTES: 3D Geometry classes take cues from libraries used by Robot Operating System in that position and orientation are
           combined in a single structure called a pose. """

# == class Pose ==
     
class Pose( object ):
    """ A simple container for position and orientation """
    def __init__( self , 
                  pPos = [ 0.0 , 0.0 , 0.0 ] , 
                  pQuat = Quaternion.k_rot_to_Quat( [0.0,0.0,1.0] , 0.0 ) ):
        # dbgLog(-1, "Pose:",len(locals().values()) , locals().values() )
        self.position = vec_copy( pPos ) # --- R3 vector for position, relative to the parent frame
        self.orientation = pQuat # Quaternion for orientation, relative to the parent frame
        
    def __str__(self):
        """ Return a string representation of the form ( str(position) , str(orientation) ) """
        return "( " + format_dec_list( self.position , DISPLAYPLACES ) + " , " + format_dec_list( self.orientation.serialize() , DISPLAYPLACES ) + " )"
    
    def get_copy( self ):
        """ Return a Pose with the same position and orientation as this Pose """
        return Pose( self.position , self.orientation.get_copy() )
        
    @staticmethod
    def compose( aPose , bPose ):
        """ Add 'bPose' to 'aPose' , expressed in the 'aPose' reference frame """
        return Pose( # Treat b as a link downstream of a
            np.add( aPose.position ,  bPose.position ) ,    
            Quaternion.serial_rots( bPose.orientation , aPose.orientation ) 
        )
        
    @staticmethod
    def pos_no_turn( R3triple ):
        """ Pose with a position and a zero-turn orientatrion """
        return Pose( R3triple , Quaternion.no_turn_quat() )

    @staticmethod
    def diff_from_to( aPose , bPose ):
        """ Return the relative pose from 'aPose' to 'bPose', assuming both are expressed in the same frame """
        return Pose( # Pose is the difference in a common frame, this does not express bPose as though aPose were its parent frame
            np.subtract( bPose.position , aPose.position ) ,
            Quaternion.diff_from_to( aPose.orientation , bPose.orientation )
        )
        
    @staticmethod
    def diff_mag( aPose , bPose ):
        """ Return the linear and angular difference between Poses """
        diff = Pose.diff_from_to( aPose , bPose )
        return vec_mag( diff.position ) , diff.orientation.get_k_rot()[1]
        
    @staticmethod
    def blend_poses( aPose , bPose , numPts ):
        """ Blend from 'aPose' to 'bPose' (inclusive) along a straight line and the shortest turn """
        poseDiff = Pose.diff_from_to( aPose , bPose ) # Obtain the difference between the two poses
        positions = vec_linspace( aPose.position , bPose.position , numPts  ) # Create evenly spaced points between the two positions
        k , rot = poseDiff.orientation.get_k_rot() # Get the axis-angle of the orientation difference
#        if get_dbg_lvl() == 1:
#            print "blend_poses: axis-angle of shortest rotation" , k , rot
        rotations = []
        for blendRot in np.linspace( 0 , rot , numPts ): # For each angle increment: Apply a rotation increment
            rotations.append( Quaternion.compose_rots( aPose.orientation , Quaternion.k_rot_to_Quat( k , blendRot ) ) )
        rtnPoses = []
        for index in xrange(len(positions)): # For each position-orientation pair
            rtnPoses.append( Pose( positions[index] , rotations[index] ) ) # construct a Pose and append
        return rtnPoses # All done, return
   
    @staticmethod
    def blend_in_steps( aPose , bPose , linStep = 0.02 , rotStep = 0.087 ):
        """ Blend from 'aPose' to 'bPose' (inclusive) along a straight line and the shortest turn, not exceeding 'linStep' or 'rotStep' """
        poseDiff = Pose.diff_from_to( aPose , bPose ) # Obtain the difference between the two poses
        dist = vec_mag( poseDiff.position )
        k , rot = poseDiff.orientation.get_k_rot() # Get the axis-angle of the orientation difference
        if dist / linStep >= rot / rotStep: # If more linear steps are needed than rotational steps
            distances = incr_max_step( 0 , dist , linStep )
            angles = np.linspace( 0 , rot , len(distances) )
        else: # else more rotational steps are needed than linear steps
            angles = incr_max_step( 0 , rot , rotStep )
            distances = np.linspace( 0 , dist , len(angles) )
        
        direction = vec_unit( poseDiff.position )
        positions = [ np.add( aPose.position , np.multiply( direction , disp ) ) for disp in distances ]
        orientations = [ Quaternion.compose_rots( aPose.orientation , Quaternion.k_rot_to_Quat( k , blendRot ) ) for blendRot in angles ]
        
        rtnPoses = []
        for index in xrange( len( positions ) ): # For each position-orientation pair
            rtnPoses.append( Pose( positions[index] , orientations[index] ) ) # construct a Pose and append
        return rtnPoses # All done, return   
   
    @staticmethod
    def step_from_to( aPose , bPose , linStep , rotStep ):
        """ Return a linear step and a rotational step from 'aPose' towards 'bPose' for use with Jacobian trajectories """
        poseDiff = Pose.diff_from_to( aPose , bPose ) # Obtain the difference between the two poses
        dist = vec_mag( poseDiff.position )
        vecStep = vec_unit( poseDiff.position ) * linStep if dist > linStep else poseDiff.position
        vecStep = np.multiply( vec_unit( poseDiff.position ) , linStep ) if dist > linStep else poseDiff.position
        k , rot = poseDiff.orientation.get_k_rot() # Get the axis-angle of the orientation difference
        rotStep = rotStep if rotStep > rot else rot
        trnStep =  np.multiply( k , rotStep )
        #      [ delta x    , delta y    , delta z    , k0 * d theta , k1 * d theta , k2 * d theta ]
        return [ vecStep[0] , vecStep[1] , vecStep[2] , trnStep[0]   , trnStep[1]   , trnStep[2]   ]
        
    @staticmethod
    def single_step_from_to( aPose , bPose ):
        """ Return the linear and rotational difference in the form [ delta x , delta y , delta z , k0 * d theta , k1 * d theta , k2 * d theta ] """
        poseDiff = Pose.diff_from_to( aPose , bPose ) # Obtain the difference between the two poses
        vecStep = vec_unit( poseDiff.position )
        k , rot = poseDiff.orientation.get_k_rot() # Get the axis-angle of the orientation difference
        trnStep =  np.multiply( k , rot )
        #      [ delta x    , delta y    , delta z    , k0 * d theta , k1 * d theta , k2 * d theta ]
        return [ vecStep[0] , vecStep[1] , vecStep[2] , trnStep[0]   , trnStep[1]   , trnStep[2]   ]
        
    
    @staticmethod
    def blend_orbit_center( aPose , bPose , center , numPts ):
        """ Blend from 'aPose' to 'bPose' (inclusive) along a curved path orbiting a 'center' along the shortest turn """
        # NOTE: In this version, the direction of the axis about which the trajectory orbits is completely dependent on the input poses and
        #       The shortest turn that will bring one to the other. It is possible for this the pose to be turning on an axis that is different
        #       from the axis of the trajectory, see 'blend_orbit_k'
        cenToA = np.subtract( aPose.position , center ) # THe vector from the center to the A pose position
        cenToB = np.subtract( bPose.position , center ) # THe vector from the center to the B pose position
        kOrbit , rotOrbit = Quaternion.shortest_btn_vecs( cenToA , cenToB ).get_k_rot()
        orientations = Quaternion.blend_orientations( aPose.orientation , bPose.orientation , numPts )
        rtnPoses = []
        for bDex , blendRot in enumerate( np.linspace( 0 , rotOrbit , numPts ) ): # For each angle increment: Rotate the position vector about the orbit
            rtnPoses.append( Pose( Quaternion.k_rot_to_Quat( kOrbit , blendRot ).apply_to( cenToA ) , orientations[bDex] ) )
        return rtnPoses
    
    @staticmethod
    def origin_bases_to_Pose( origin , xBasis , yBasis , zBasis ):
        """ Return a Pose at 'origin' , with an orientation defined by the given basis vectors """
        return Pose( origin , Quaternion.principal_rot_Quat( xBasis , yBasis , zBasis ) )
        # return Pose( origin , Quaternion.bases_to_Quat( xBasis , yBasis , zBasis ) )
        
    def serialize( self ):
        """ Encode the Pose into a tuple of two lists """ # NOTE: The unpacked version of the returned values is the format that regrasp 'arm_seek_pose' expects
        return self.position , self.orientation.serialize()
    
    def translate( self , moveVec ):
        """ Move the center of the Pose by 'moveVec' without changing the orientation """
        self.position = np.add( self.position , moveVec )
        
    def set_same_as( self , otherPose ):
        """ Set this pose to be the same as the 'otherPose' """
        temp = otherPose.get_copy() # Make a copy so that Poses do not track each other
        self.position    = temp.position
        self.orientation = temp.orientation
        
    def round_small( self ):
        """ Get rid of exremely small components """
        self.position = vec_round_small( self.position )
        self.orientation.round_small()

# == End Pose ==     
     
     
# == R3 Frames ==     
"""
Frames inherit Pose
Frame-derived classes that have a specifific color scheme should define a 'CLASS.colorize' that asks the Canvas to perform this colorization
"""

# [ ] TODO: Refactor Frame2D so that it holds the points

# = class Frame =
     
class Frame3D( Pose ):
    """ A Frame is a container for geometric objects, it is defined by a Pose that is relative to the parent Frame 
    
    self.position: ---------- position in the parent frame , a position added to upstream frames
    self.orientation: ------- orientation in the parent frame , an orientation composed with upstream frames
    self.labPose.position: -- position in the lab frame , the summation of this and all upstream frames
    self.labPose.orientation: orientation in the labe frame , the composition of this and all upstream frames 
    """
    
    def __init__( self , pPos , pQuat , relPointsList ):
        """ Constructor: set up the Frame with an initial pose """
        Pose.__init__( self , pPos , pQuat ) # - Inherits position and orientation from Pose
        self.labPose   = Pose.get_copy( self ) # Lab Pose
        self.ntnPose   = Pose.get_copy( self ) # Notional Pose
        self.points    = relPointsList[:] # ---- List of points contained by this frame
        self.labPts    = self.points[:] # ------ Lab frame points
        self.ntnPts    = self.points[:] # ------ Notional frame points
        self.subFrames = [] # ------------------ Nested frames
        self.objs      = [] # ------------------ Contained geo and render objects
        self.parent    = None # ---------------- Reference to the frame that contains this one
    
    def attach_sub(self, subFrm):
        """ Attach a subframe to this frame with appropriate connections """
        subFrm.parent = self # A frame can have only one parent
        self.subFrames.append( subFrm ) # Add 'subFrm' to the list of subframes
    
    def transform_contents(self, upPosition = None, upOrientation = None):
        """ Transform the coordinates of contained geometric objects and request sub-Frames to transform """
        # dbgLog(-1, "Frame.transform_contents, position:", self.position )
        # dbgLog(-1, "Frame.transform_contents, orientation:", self.orientation )
        
        # Set the orientation that will rotate the contents to the lab frame
        if upOrientation != None: # If an orientation was passed from upstream, then compose self orientation with it
            self.labPose.orientation = Quaternion.serial_rots( self.orientation , upOrientation ) # compose in upstream direction
        else: # else there is no parent frame, self orientation is the only one that influences coords
            self.labPose.orientation = self.orientation        
         
        # Using the above orientation, rotate the Frame origin in the lab frame
        if isinstance( upPosition , ( list , np.ndarray ) ): # If coordinates passed from upstream are not 'None', add them to all containing objects
            self.labPose.position = np.add( upPosition[:] , upOrientation.apply_to(self.position) ) # Calc the position in the labe frame, given the above
        else: # else there is no parent frame, objects are expressed in this frame only
            self.labPose.position = self.position[:] # Calc the position in the labe frame, given the above
        
        for pntDex , pnt in enumerate( self.points ): # Transform the contained points 
            self.labPts[ pntDex ] = np.add( self.labPose.position , self.labPose.orientation.apply_to( pnt ) )
        
        for obj in self.objs: # For each of the contained objects
            # dbgLog(-1, "Frame.transform_contents, transforming:", obj )
            for pntDex , point in enumerate(obj.coords): # For each point in the contained object
                # dbgLog(-1, "Point",point,"in",obj )
                obj.labCoords[pntDex] = np.add( self.labPose.position , self.labPose.orientation.apply_to( point ) )
                    
        for frame in self.subFrames: # for each of the sub-frames, recursively transform
            frame.transform_contents( self.labPose.position , self.labPose.orientation )

    def transform_points_to_lab( self , *ptsLst ):
        """ Transform each of the elements of 'ptsLst' to the lab frame as though they were in this reference frame """
        rtnList = []
        self.transform_contents()
        for pnt in ptsLst: #                     vvv--- Note for spatial points there is an offset
            # rtnList.append( vec_round_small( np.add( self.labPose.position , self.labPose.orientation.apply_to( pnt ) ) ) )
            rtnList.append( np.add( self.labPose.position , self.labPose.orientation.apply_to( pnt ) ) )
        return rtnList
            
    def transform_vectors_to_lab( self , *vecLst ):
        """ Transform each of the elements of 'vecLst' to the lab frame as though they were in this reference frame """
        rtnList = []
        self.transform_contents()
        for vec in vecLst: #             vvv--- No offset , Rotation only
            rtnList.append( self.labPose.orientation.apply_to( vec ) )
        return rtnList
    
    def transform_poses_to_lab( self , *poseList ):
        """ Transform each of the elements of 'poseList' to the lab frame as though they were in this reference frame """
        rtnList = []
        self.transform_contents()
        for pose in poseList:
            rtnList.append(  
                Pose( np.add( self.labPose.position , self.labPose.orientation.apply_to( pose.position ) ) , 
                      # Quaternion.serial_rots( self.orientation , pose.orientation ) )
                      Quaternion.serial_rots( pose.orientation , self.orientation ) )
            )
        return rtnList

# = End Frame =

# = class LinkFrame =

class LinkFrame(Frame3D):
    """ A Frame that behaves like a DH robot link """
    
    def __init__(self, pPos , quatList, *containedObjs):
        """ Constructor: set up the Frame with an initial pose """
        super(LinkFrame, self).__init__(pPos, Quaternion.no_turn_quat(), *containedObjs)
        self.rotations = quatList # List of rotations to apply in order to this frame, Theta before Alpha
        self.jointRotDex = 0 # Rotation that corresponds to a rotational joint, default is the first rotation
        
    def set_theta(self, pTheta):
        """ Set the angle 'pTheta' of the rotational joint """
        # NOTE: This function assumes that the Quatenion at 'jointRotDex' is a Rotation, otherwise an error will result
        self.rotations[self.jointRotDex].set_theta( pTheta )
        
    def transform_contents(self, upPosition = None, upOrientation = None):
        """ Transform the coordinates of contained geometric objects and request sub-Frames to transform """
        self.orientation = Quaternion.serial_rots( *self.rotations ) # compose all the rotations that will orient the frame
        super(LinkFrame, self).transform_contents( upPosition , upOrientation )
        
# = End LinkFrame =

    
# == Drawing Helpers and Misc ==
    
def circle_arc_3D( axis , center , radius , beginMeasureVec , theta , N ):
    """ Return points on a circular arc about 'axis' at 'radius' , beginning at 'beginMeasureVec' and turning 'theta' through 'N' points """
    thetaList = np.linspace( 0 , theta , N )
    k         = vec_unit( axis )
    rtnList   = []
    # 1. Project the theta = 0 vector onto the circle plane
    measrVec = vec_unit( vec_proj_to_plane( beginMeasureVec , axis ) )
    # 2. For each theta
    for theta_i in thetaList:
        # 3. Create a rotation matrix for the rotation
        R = rot_matx_ang_axs( theta_i , k  )
        # 4. Apply rotation to the measure vector && 5. Scale the vector to the radius && 6. Add to the center && 7. Append to the list
        rtnList.append( np.add( center , np.multiply( np.dot( R , measrVec ) , radius ) ) )
    # 8. Return
    return rtnList
    
# __ End Misc __
