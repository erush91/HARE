#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "hallway_FSM.py"
__version__  = "2019.02" 
__desc__     = "FSM planner for a 2D car"
"""
James Watson , Template Version: 2018-05-14
Built on Wing 101 IDE for Python 2.7

Dependencies: numpy
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
# ~~ Local ~~
from marchhare.PolyWorldApp import *
from marchhare.VectorMath.Vector2D import *

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === Main Application =====================================================================================================================


# = Program Classes =

class Car:
	""" Simplest car imaginable """
	
	def __init__( self ):
		""" Create car """ 
		self.body = Poly2D( [0,0] , 0.0 , 
							[ -20 , -20 ] , 
							[  20 , -20 ] , 
							[  12 ,  20 ] , 
							[ -12 ,  20 ] )

# _ End Class _


# ~~ Init Sim ~~
car     = Car()
_dTheta = np.pi / 30

# = Program Functions =

def kb_cb( event ):
	print "event.char:" , repr( event.char ) , "event.keysym:" , event.keysym
	if event.keysym in ( "Up" , "Down" , "Left" , "Right" ):
		print "ARROWED!"
	if event.keysym == "Left":
		car.body.rotate(  _dTheta )
	if event.keysym == "Right":
		car.body.rotate( -_dTheta )	

def mouse_cb( event ):
    frame.focus_set()
    print "clicked at" , event.x , event.y

# _ End Func _




if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:] # Terminal arguments , if they exist
    
    # Create objects to add to the simulation
    hept = Poly2D.regular( 7 , 200 , [0,0] , np.pi / 17 )
    
    
    # Write function(s) for stuff to do each frame of the simulation
    def update_shape():
        """ Spin the heptagon, yeah! """
        hept.rotate( -np.pi / 128 )
    
    # ~~ Instantiation ~~
    foo = Poly2DApp( 650 , 500 ) # ----- Create the sim display with a black canvas ( xWidth , yHeight ) pixels
    foo.set_title( "2D Hallway" ) # This will appear in the top bar of the simulation window
    foo.rootWin.bind( "<Key>"      , kb_cb )
    
    # ~~ Setup ~~
    foo.calcFunc = [ update_shape ] # ------ Give the app work to do each frame
    
    #~ foo.attach_drawables( hept ) # ----- Add Frame2D/Poly objects to the world so that they can be drawn
    foo.attach_drawables( car.body ) # ----- Add Frame2D/Poly objects to the world so that they can be drawn
    
    foo.color_all( 'green' ) # --------- Default segment color is black 
                                         # TODO: Make a new default color!
    
    # ~~ Run ~~
    foo.update_all( ) # ---------------- Update once before starting the sim so that everything gets placed at the proper lab coordinates
    foo.rootWin.after( 100 , foo.run ) # Start the simulation after a 100 ms delay
    foo.rootWin.mainloop() # ----------- Show window and begin the simulation / animation
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________
