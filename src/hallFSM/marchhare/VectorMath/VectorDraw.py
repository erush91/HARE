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
import Tkinter
from math import sqrt, cos, sin, radians, acos, pi, atan2, asin, e
from numbers import Number # for checking if a thing is a NumberR3triple
from copy import deepcopy
from random import random
# ~ Special Libraries ~
import numpy as np
# ~ Local Libraries ~
# from ResearchUtils.DebugLog import *
from AsmEnv import PriorityQueue , elemw , format_dec_list # Needed for 'point_in_poly_w', etc
from Vector2D import *

# set_dbg_lvl(1) # Transformation of objects contained in Frames

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026

# = Drawing Init =    # ARE THESE USED? SEE IF THEY BREAK    
def attach_geometry(rootFrame, pCanvas): # <<< resenv
    """ Traverse geometry from the root frame to the all subframes, recursively, attaching all drawable geometry to canvas """
    for obj in rootFrame.objs:
        obj.attach_to_canvas( pCanvas )
    for frame in rootFrame.subFrames:
        attach_geometry( frame , pCanvas )
        
def attach_transform( rootFrame, pTransform ): # <<< resenv
    """ Traverse geometry from the root frame to the all subframes, recursively, attaching a coordinate transformation to each drawable """
    for obj in rootFrame.objs:
        obj.transform = pTransform
    for frame in rootFrame.subFrames:
        attach_transform( frame , pTransform )
        
def color_all(rootFrame, pColor): # <<< resenv
    """ Traverse geometry from the root frame to the all subframes, recursively, setting all graphics to 'pColor' """
    if 'colorize' in rootFrame.__dict__:
        rootFrame.colorize(pColor)
    else:
        for obj in rootFrame.objs:
            obj.set_color( pColor )   
    for frame in rootFrame.subFrames:
        color_all( frame , pColor )
# = End Drawing =        
# == End Frames ==
        
# TODO : Consider whether having a parent class geometric object would overcomplicate things

""" NOTES: In a previous implementation, every geometrical object was essentially a Frame that could contain other objects
           that could be nested indefinitely. That probably overcomplicated things a great deal. For now will only consider
           a line segment to be just that, and not represent anything greater than a straight connection between two points. 
           
           For now, not considering a point to be its own class. A point can be just a simple triple represented with np.ndarray 
           
           For now, leaving all the Tkinter stuff inside the geometry classes, even though I would like geometry and its calculations
           to remain separate from drawing. These have to be linked in some way to be useful, at any rate. """

# == class Segment ==

class Segment(object): # <<< resenv
    """ A line segment to be displayed on a Tkinter canvas """ # TODO: * Generalize for any display?
#                                                                      * Consider extending this class for particular displays?
       
    
    def __init__(self , pCoords = None , TKcanvas=None , color=None): # candidate super signature
        """ Assign vars and conditionally create the canvas object 'self.drawHandle' """
        #dbgLog(-1, "Segment:",len(locals()),"args passed" )
        #dbgLog(-1, "Segment locals are:",locals() )
	
        self.transform = self.dummy_transform # Optionally change this for a different rendering engine
        self.display_xform = self.scaled_coords_to_list
        self.displayScale = 1 # /4.0
        #                    v--- Need to make a copy here, otherwise relative coords will be transformed
        self.coords = vec_copy_deep( pCoords ) if pCoords else [ [0.0 , 0.0] , [0.0 , 0.0] ] # Coordinates expressed in the parent reference Frame
        # Coordinates expressed in the lab frame, used for drawing and lab frame calcs
        self.labCoords = vec_copy_deep( pCoords ) if pCoords else [ [0.0 , 0.0] , [0.0 , 0.0] ] 
        #dbgLog(-1, "coords[0]" , self.coords[0] , "coords[1]" , self.coords[1] )
        self.canvas = None
        if TKcanvas: # If canvas is available at instantiation, go ahead and create the widget
            #dbgLog(-1, "Segment: init with canvas")
            self.canvas = TKcanvas
            self.drawHandle = TKcanvas.create_line( self.coords[0][0] , self.coords[0][1] , self.coords[1][0] , self.coords[1][1])
            #dbgLog(-1, "Item",self.drawHandle,"created on canvas")
            if color:
                self.canvas.itemconfig(self.drawHandle,fill=color)
                #dbgLog(-1, "Item",self.drawHandle,"has color", color)
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
        # print "Coords:" , coords
        for pair in coords:
            # print "Pair:" , pair
            rtnList.extend( np.multiply( pair , scale ) ) # FIXME: Scaling this twice seems redundant
        return rtnList 
        
    def dummy_transform(self, pntList, scale): # candidate super fuction
        """ Dummy coordinate transformation, to be replaced with whatever the application calls for """
        # print "Points List:" , pntList
        return pntList # no actual transformation done to coords
    
    def set_color(self, color): # candidate super functions
        """ Set the 'color' of the line """
        self.canvas.itemconfig(self.drawHandle,fill=color)
        self.canvas.itemconfig(self.drawHandle, width=3)
        
    def attach_to_canvas(self, TKcanvas): # candidate super function
        """ Given a 'TKcanvas', create the graphics widget and attach it to the that canvas """
        self.drawHandle = TKcanvas.create_line( -10 , -10 , -5 , -5 ) # Init to dummy coords x1, y1 , x2 , y2
        self.canvas = TKcanvas
            
    def update(self): # candidate super function
        """ Update the position of the segment on the canvas """
        # print "Segment:" , self
        
        temp1 = self.transform( self.labCoords, self.displayScale ) # Project 3D to 2D
        # print "temp1:" , temp1
        temp2 = self.display_xform( temp1 , 1 )
        # print "temp2:" , temp2 , endl
        # print self.labCoords , temp , self.transform.__name__
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


# == class Ray ==

class Ray(Segment): # <<< resenv
    """ Represents a ray, extending from the first point, through a second point, and to the end of the canvas """
    
    def __init__( self , pSimFrame , pCoords=None , TKcanvas=None , color=None ):
        """ Assign vars and conditionally create the canvas object 'self.drawHandle' """
        super( Ray , self ).__init__( pCoords , TKcanvas , color )
        self.simFrame = pSimFrame
        self.transform = self.ray_extend
        print "Ray created!"
        
    def ray_extend( self , pntList , scale ):
        """ Dummy coordinate transformation, to be replaced with whatever the application calls for """
        # NOTE: This function assumes that the first point is the origin, and the other end of the segment will extend to the edge of the screen
        
        end = None
        
        print "self.simFrame.boundRays:" , self.simFrame.boundRays
        
        for bound in self.simFrame.boundRays:
            intersect = intersect_ray_2D( self.labCoords , bound )
            if intersect:
                print "Found intersection:" , intersect
                end = intersect
                break
        if end == None:
            end = pntList[1]
        
        return [ pntList[0] , end ] 
        
# == End Ray ==
        
# === End Geometry ===


# === Assembly Planning Structures ===

class Polyline2D(Frame2D): # <<< resenv
    """ Represents generic 2D figures """
    
    def __init__( self , pPos , pTheta , *coords ):
        """ Set up segments for drawing """
        super( Polyline2D , self ).__init__( pPos , pTheta )
        if len( coords ) % 2 == 0: # Enforce: There must be an even number of vertices
            self.points = [ vec_copy( pnt ) for pnt in coords ] # Enforce: Points must be on the local X-Y plane
            self.labPts = [ vec_copy( pnt ) for pnt in coords ]
            # For every other point starting with the first, connect the point with the next with a Segment
            for crDex , coord in enumerate( coords ):
                if crDex % 2 == 0:
                    self.objs.append( Segment( [ self.points[crDex] , self.points[crDex+1] ] ) )
        else:
            raise ValueError( "Polyline2D.__init__: Figure must have an even number of vertices!" )

# == Special Figures ==

class CrossEven(Polyline2D):
    """ Simple plus-style cross of two equal length segments """
    
    def __init__( self , pPos , pTheta , beamLen ):
        """ Set up segments for drawing """
        super( CrossEven , self ).__init__( pPos , pTheta , [  0 ,  beamLen/2 ] , [  0 , -beamLen/2 ] , [ -beamLen/2 ,  0 ] , [  beamLen/2 ,  0 ] )

# == End Special ==



        
# === End Planning Structures ===   