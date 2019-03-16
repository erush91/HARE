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
from marchhare.Vector import vec_dif_unt
from marchhare.VectorMath.Vector2D import *
from marchhare.MathKit import clamp_val

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === RANSAC ===============================================================================================================================

def score_segment_model( data , segmentFit , distAccept , beyondAccept ):
    """ Return the fraction of 'data' that is within perpendicular 'distAccept' of 'segmentFit' """
    dataLen   = len( data )
    numAccept = 0
    inliers   = []
    for datDex , datum in enumerate( data ):
        dist = d_point_to_segment_2D( datum , segmentFit )
        if dist <= distAccept and dist_beyond_segment( datum , segmentFit ) < beyondAccept:
            numAccept += 1
            inliers.append( datDex )
    return numAccept / dataLen , inliers

def segment_2D_RANSAC( data , iterLimit , sampleFrac , distCrit , acceptFrac ):
    """ Attempt to a line segment to noisy data """
    # The input to the RANSAC algorithm is:
    #    a set of observed data values, 
    #    a way of fitting some kind of model to the observations, 
    #    and some confidence parameters. 
    
    # ~~ Init ~~
    itNum      = 0
    dataLen    = len( data )
    mid        = dataLen // 2
    bgnRng     = [ 0   , mid+1   ]
    endRng     = [ mid , dataLen ]
    slcNum     = int( ceil( dataLen * sampleFrac ) )
    bestModel  = [[0,0],[0,0]]
    bestScore  = 0.0
    bestDices  = []

    # 0. RANSAC achieves its goal by repeating the following steps until some iteration limit is reached:
    while itNum < iterLimit:
        
        # 1. Select a random subset of the original data. Call this subset the hypothetical inliers.
        bgnSmp = []
        endSmp = []
        for i in range( slcNum ):
            bgnSmp.append( data[ randrange( *bgnRng ) ][:] )
            endSmp.append( data[ randrange( *endRng ) ][:] )
            
        # 2. A model is fitted to the set of hypothetical inliers.
        bgnPnt   = vec_avg( *bgnSmp )
        endPnt   = vec_avg( *endSmp )
        model    = [  bgnPnt[:] , endPnt[:] ]
        midPnt   = vec_avg( endPnt , bgnPnt )
        dirVec12 = vec_dif_unt( endPnt , bgnPnt )
        dirVec21 = vec_dif_unt( bgnPnt , endPnt )
        bgnPnt   = midPnt
        endPnt   = midPnt
        
        # Extend , Try to find the edges of the inliers (This is splitting model calc a bit)
        for i in range( slcNum ):
            datum1 = np.subtract( bgnSmp[i] , bgnPnt )
            if vec_proj( datum1 , dirVec12 ) < 0:
                if d_point_to_segment_2D( bgnSmp[i] , model ) < distCrit:
                    bgnPnt = np.add( bgnPnt , vec_proj_onto( datum1 , dirVec12 ) )
            datum2 = np.subtract( endSmp[i] , endPnt )
            if vec_proj( datum2 , dirVec21 ) < 0:
                if d_point_to_segment_2D( endSmp[i] , model ) < distCrit:
                    endPnt = np.add( endPnt , vec_proj_onto( datum2 , dirVec21 ) )
            
        # 3. All other data are then tested against the fitted model. 
        #    Those points that fit the estimated model well, according to some model-specific loss function, 
        #    are considered as part of the consensus set.
        score , inliers = score_segment_model( data , [ bgnPnt , endPnt ] , distCrit , distCrit )
        # 3.5. We must return a model
        if score > bestScore:
            bestModel = [ bgnPnt , endPnt ]
            bestScore = score
            bestDices = inliers
        # 4. The estimated model is reasonably good if sufficiently many points have been classified as part of the consensus set.
        if bestScore >= acceptFrac:
            break
        # 5. Afterwards, the model may be improved by reestimating it using all members of the consensus set.
        # FIXME : IMPROVE?
        itNum += 1
    return bestModel , inliers

# ___ END RANSAC ___________________________________________________________________________________________________________________________


# === Main Application =====================================================================================================================


# = Program Classes =

class LIDAR:
    """ Fan of distance readings """
    
    def __init__( self , angleSpread , numReadings , maxDistance , pPos , pTheta ):
        """ Create a cheesy LIDAR with the given params """
        self.maxDist   = maxDistance # ----------------------------------------------- Maximum distance that the sensor will return
        self.angSpread = angleSpread # ----------------------------------------------- Double-sided, symmetic FOV [rad]
        self.angles    = np.linspace( -angleSpread/2 , angleSpread/2 , numReadings ) # Angle for each reading
        self.num       = numReadings # ----------------------------------------------- Number of readings per scan, spaced evenly in spread
        self.reading   = [ maxDistance ] * numReadings # ----------------------------- Current reading
        self.sensor    = Frame2D( pPos , pTheta ) # ---------------------------------- Frame for transforming rays


    def scan( self , polyList , noiseWidth = 0.0 ):
        """ Return the distances and lab-frame collisions , with optional noise added """
        rayOrg = self.sensor.labPose.position[:]
        rtnPts = []
        # 1. For each ray in the spread
        for angDex , rayAng in enumerate( self.angles ):
            clstDist = self.maxDist
            # 1.5. Construct the ray
            rayPnt = np.add( polr_2_cart_0Y( [ self.maxDist , rayAng - self.sensor.labPose.orientation.theta ]  ) , rayOrg )
            ray = [ rayOrg , rayPnt ]
            rayDir = vec_dif_unt( rayPnt , rayOrg )
            # 2. For each object in the env
            for poly in polyList:
                # 3. Get the nearest collision distance between the ray and the polygon
                dist = poly.ray_nearest_exterior_clsn_dist_lab( ray )
                if dist == None:
                    dist = self.maxDist
                if dist < clstDist:
                    clstDist = dist
            # 4. Record the reading and the collision point
            self.reading[ angDex ] = clstDist
            #~ print rayOrg , rayDir , clstDist
            rtnPts.append(  np.add( rayOrg , np.multiply( rayDir , clstDist ) )  )
        return rtnPts
    
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
        self.frame   = Frame2D( [ 0 , 0 ] , 0 ) 
        self.frame.attach_sub( self.body )
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
        # ~ Sensing ~
        self.lidar = LIDAR( np.pi , 100 , 200 , [0,0] , 0.0 )
        self.frame.attach_sub( self.lidar.sensor )
        
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
_initSpd = 60 # [px/s]
car.set_state_XYTh( 0,0,-np.pi/2 )
car.set_ctrl( 0 , _initSpd )

# = Program Functions =

_RUNNING = True

def kb_cb( event ):
    """ Process keyboard events """
    global _RUNNING
    if event.keysym == "Left":
        car.delta_steer(  _dTheta )
        #~ print "Steering Angle:" , car.wlTheta
    if event.keysym == "Right":
        car.delta_steer( -_dTheta )    
        #~ print "Steering Angle:" , car.wlTheta
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

car.set_state_XYTh( 0                                , 
                     scrnHght//2 - hallWdth//2 - 25  ,
                    -np.pi/2                         )

def dirty_convert( ptsList ):
    """ THIS ENV IS WET TRASH """
    rtnPts = [  ]
    for pnt in ptsList:
        x = pnt[0] 
        y = pnt[1]
        rtnPts.append( 
            [ x + scrnWdth/2.0 , scrnHght/2.0 - y ] 
        )
    return rtnPts
    

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
        
        car.set_state_XYTh( seq[-1][0] , seq[-1][1] , seq[-1][2] )
        
        hits = dirty_convert( car.lidar.scan( walls ) )
        #print hits
        
        foo.temp_points( hits , 20 , 'red' )
        
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
