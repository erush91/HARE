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

Dependencies: numpy , marchhare
"""


"""  
~~~ Developmnent Plan ~~~
[Y] Implement non-holon car
[Y] Build a hallway env
[ ] Build a sensor model
[ ] Finite State Machine
	[ ] Identify lines
	[ ] Wall Follow State
	[ ] Right Turn state
{ } Remove "upness" from 'Vector2D' and 'PolyWorldApp'
"""

"""
~~~ NOTES ~~~
2019-03-03: * RM Res --- 1920 x 1080
			* flat Res - 1920 x 1080

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
from math import pi , sqrt , tan
# ~~ Special ~~
import numpy as np
# ~~ Local ~~
from marchhare.PolyWorldApp import *
from marchhare.VectorMath.Vector2D import *
from marchhare.MathKit import clamp_val

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
	# 2019-03-03: All units are in pixels for now
	
	def __init__( self , thetaLim , fwdVelLimit , bckVelLimit ):
		""" Create car """ 
		# ~ Car Geo ~
		self.body    = Poly2D( [ 0 , 0 ] , 0 , # Polygonal representation of the car itself
							   [ -20 , -20 ] , 
							   [  20 , -20 ] , 
							   [  12 ,  20 ] , 
							   [ -12 ,  20 ] )
		self.frame   = Frame2D( [ 0 , 0 ] , 0 )  ;  self.frame.attach_sub( self.body )
		self.whlBase = 40 # ---------------------- Perpendicular distance between the front and rear "axles" [px] , NOT USED?
		# ~ Controls and Their Limits ~
		self.turnLim = [ -thetaLim   ,  thetaLim   ] # [ right , left ] steering angle limits [rad] , Symmetric
		self.velcLim = [ bckVelLimit , fwdVelLimit ] # [ backwards , forwards ] linear velocity limits [px/s]
		# ~ State / Config ~
		self.x       = 0.0 # x position 
		self.y       = 0.0 # y position
		self.theta   = 0.0 # orientation [rad] from vertical
		self.wlTheta = 0.0 # Steering angle
		self.linrVel = 0.0 # Linear Velocity , disregarding all wheel dynamics
		
	def set_state_XYTh( self , pX , pY , pTheta ):
		""" Set the car state """
		# NOTE: This function does not handle collisions
		self.x       = pX # --- x position 
		self.y       = pY # --- y position
		self.theta   = pTheta # orientation [rad] from vertical
		self.frame.set_pos( [ self.x , self.y ] )		
		self.frame.set_theta( self.theta )
		
	def set_ctrl( self , turnAngRad , linearVelMS ):
		""" Set the control effort for the car , Making sure to keep control efforts within the set limits """
		self.wlTheta = clamp_val( turnAngRad  , self.turnLim )
		self.linrVel = clamp_val( linearVelMS , self.velcLim )
		
	def delta_steer( self , delTheta ):
		""" Adjust the steering angle by 'delTheta' """
		self.set_ctrl( self.wlTheta + delTheta , self.linrVel ) # 'set_ctrl' enforces control limits
		
	def non_hol_fwd_dyn( self , Del_T , dt ):
		""" Simple Non-Holonimic FK: Given the current linear speed , steering angle, duration , timestep : Return sequence of states """
		# NOTE: This function does not handle collisions
		# NOTE: This function uses the simplest, linearized bycycle model, and does not account for slip, skid, or drift AT ALL.
		#       Model is based on constant linear speed and steering angle
		# NOTE: This function does not alter car state
		tElps = 0.0 # Time elapsed during sim
		rtnStates = [] # sequence of [ ... , [ x , y , theta ] , ... ] states
		# 0. Init state for sim
		x  = self.x
		y  = self.y
		th = self.theta
		# 1. While elapsed time is less than or equal to running time
		while tElps <= Del_T:
			# 2. Advance timestep
			tElps += dt 
			# 3. Compute timestep state , Taken from 'SIMULATE_CARBOT' (HW03)
			x  = x  + -self.linrVel * cos( th - np.pi/2 ) * dt;
			y  = y  + -self.linrVel * sin( th - np.pi/2 ) * dt;
			th = th + ( self.linrVel / self.whlBase ) * tan( self.wlTheta ) * dt;
			# 4. Append state
			rtnStates.append( [ x , y , th ] )
		return rtnStates
			

# _ End Class _


# ~~ Init Sim ~~
car      = Car( np.pi / 4.0 , 20 , -10 )
_dTheta  = np.pi / 30
_initSpd = 15 # [px/s]
car.set_state_XYTh( 0,0,-np.pi/2 )
car.set_ctrl( 0 , _initSpd )

# = Program Functions =

_RUNNING = True

def kb_cb( event ):
	""" Process keyboard events """
	global _RUNNING
	if event.keysym == "Left":
		car.delta_steer(  _dTheta )
		print "Steering Angle:" , car.wlTheta
	if event.keysym == "Right":
		car.delta_steer( -_dTheta )	
		print "Steering Angle:" , car.wlTheta
	if event.keysym == "Escape":
		_RUNNING = False
		print "SHUTDOWN"
	
	#~ print "event.char:" , repr( event.char ) , "event.keysym:" , event.keysym
	#~ if event.keysym in ( "Up" , "Down" , "Left" , "Right" ):
		#~ print "ARROWED!"
	#~ if event.keysym == "Left":
		#~ car.body.rotate(  _dTheta )
	#~ if event.keysym == "Right":
		#~ car.body.rotate( -_dTheta )	

def mouse_cb( event ):
    frame.focus_set()
    print "clicked at" , event.x , event.y

# _ End Func _

scrnWdth = 1600
scrnHght =  800
wallThic =   20
hallWdth =  220

walls = [ Poly2D.rectangle( scrnWdth-50            , wallThic                        , [ 0 ,  ( scrnHght // 2 - 25 ) ]    , 0.0 ) , 
		  Poly2D.rectangle( scrnWdth-50            , wallThic                        , [ 0 , -( scrnHght // 2 - 25 ) ]    , 0.0 ) , 
		  Poly2D.rectangle( wallThic               , scrnHght-50-wallThic            , [ scrnWdth//2-37.5 , 0 ]           , 0.0 ) , 
		  Poly2D.rectangle( wallThic               , scrnHght-50-wallThic-2*hallWdth , [ scrnWdth//2-37.5-hallWdth , 0 ]  , 0.0 ) , 
		  Poly2D.rectangle( scrnWdth-2*hallWdth-50 , wallThic                        , [ 0 ,  (scrnHght//2-25)-hallWdth ] , 0.0 ) ,
		  Poly2D.rectangle( scrnWdth-2*hallWdth-50 , wallThic                        , [ 0 , -(scrnHght//2-25)+hallWdth ] , 0.0 ) ,
		  Poly2D.rectangle( wallThic               , scrnHght-50-wallThic            , [ -scrnWdth//2+37.5 , 0 ]          , 0.0 ) ,  
		  Poly2D.rectangle( wallThic               , scrnHght-50-wallThic-2*hallWdth , [ -scrnWdth//2+37.5+hallWdth , 0 ] , 0.0 ) ]

car.set_state_XYTh( -scrnWdth//2 + hallWdth + 50     , 
				     scrnHght//2 - hallWdth//2 - 25  ,
				    -np.pi/2                         )

if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:] # Terminal arguments , if they exist
    
    # ~~ Instantiation ~~
    foo = Poly2DApp( scrnWdth , scrnHght ) # ----- Create the sim display with a black canvas ( xWidth , yHeight ) pixels
    foo.set_title( "2D Hallway" ) # This will appear in the top bar of the simulation window
    foo.rootWin.bind( "<Key>"      , kb_cb )
    
    # Write function(s) for stuff to do each frame of the simulation
    def update_car():
        """ Get and set the next car state """
        seq = car.non_hol_fwd_dyn( foo.stepTime / 1000.0 , foo.stepTime / 1000.0 )
        #~ print "New State:" , seq[-1]
        car.set_state_XYTh( seq[-1][0] , seq[-1][1] , seq[-1][2] )
        
        foo.winRunning = _RUNNING
    
    # ~~ Setup ~~
    foo.calcFunc = [ update_car ] # ------ Give the app work to do each frame
    
    #~ foo.attach_drawables( hept ) # ----- Add Frame2D/Poly objects to the world so that they can be drawn
    foo.attach_drawables( car.frame ) # ----- Add Frame2D/Poly objects to the world so that they can be drawn
    foo.attach_drawables( *walls )
    
    foo.color_all( 'green' ) # --------- Default segment color is black 
                                         # TODO: Make a new default color!
    
    # ~~ Run ~~
    foo.update_all( ) # ---------------- Update once before starting the sim so that everything gets placed at the proper lab coordinates
    foo.rootWin.after( 100 , foo.run ) # Start the simulation after a 100 ms delay
    foo.rootWin.mainloop() # ----------- Show window and begin the simulation / animation
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________
