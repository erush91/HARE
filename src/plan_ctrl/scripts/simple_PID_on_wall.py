#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "simple_PID_on_wall.py"
__version__  = "2019.04"
__desc__     = "A Finite State Machine for autonomous car motion planning"
"""
James Watson , Template Version: 2019-02-21
Built on Wing 101 IDE for Python 2.7

Dependencies: numpy , rospy

NOTE: Incomming array will come in CW instead of CCW as in the sim!
"""


"""
~~~ Developmnent Plan ~~~
[ ] Step 1
[ ] Step 2
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
from math import pi , sqrt , sin , cos

# ~~ Special ~~
import numpy as np
import rospy
from sensor_msgs.msg import Joy # NOT USED?
# ~ from ackermann_msgs.msg import AckermannDriveStamped
# ~ from sensor_msgs.msg import LaserScan # http://www.theconstructsim.com/read-laserscan-data/
from std_msgs.msg import String , Float32MultiArray , Bool
from geometry_msgs.msg import PoseStamped, Point, Twist
from ros_pololu_servo.msg import MotorCommand
from ros_pololu_servo.msg import HARECommand

# ~ Custom Messages ~
from plan_ctrl.msg import CarState
from rosflight_msgs.msg import RCRaw

# ~~ Local ~~
from rospy_helpers import ( eq_margin , clamp_val , )

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === Main Application =====================================================================================================================

# ~~ Program Constants ~~


# == Program Functions ==

def compose_ack_ctrl_msg( steerAngle , linearSpeed , orgnStr = "FSM" ):
    """ Return an Ackerman control effort with a target steering angle and linear speed """
    # Create msg
    ack_msg = AckermannDriveStamped()
    # Compose msg
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = orgnStr
    ack_msg.drive.steering_angle = steerAngle
    ack_msg.drive.speed = linearSpeed
    # Return msg
    return ack_msg

def compose_HARE_ctrl_msg( steerAngle , linearSpeed , orgnStr = "FSM" ):
    """ Return an Ackerman control effort with a target steering angle and linear speed """
    if eq_margin( linearSpeed , 0.0 ):
        mode = 1 # STOP
        speedMag = 0.0
    elif linearSpeed > 0:
        mode = 2 # FORWARD
        speedMag = linearSpeed
    else:
        mode = 3 # BACKWARD
        speedMag = abs( linearSpeed )
    # Create msg
    ack_msg = HARECommand()
    # Compose msg
    ack_msg.steering_angle = clamp_val( steerAngle , [ -pi/2 ,  pi/2 ] ) # Position to move to in radians ( -pi/2 -> pi/2)
    ack_msg.throttle_cmd   = speedMag # Speed to move at (0 -> 1)
    ack_msg.throttle_mode  = mode
    # Return msg
    return ack_msg

_STATETRANS = [ 0.10 , 0.00 , -0.10 , 0.00 ]

def eq_margin( op1 , op2 , margin = EPSILON ):
    """ Return true if op1 and op2 are within 'margin' of each other, where 'margin' is a positive real number """
    return abs( op1 - op2 ) <= margin

def avg_filter( inLst , width = 3 ):
    """ Return a version of 'inLst' that is the average of 'width' values centered at each index """
    # NOTE: This function always applies a window width of at least 3
    lstLen = len( inLst )
    fltLst = []
    if width < 3:
        width = 3
    elif width % 2 == 0:
        width -= 1
    half = max( 1 , width//2  )
    for i in xrange( lstLen ):
        mnDex = max( 0        , i-half   )
        mxDex = min( lstLen   , i+half+1 )
        fltLst.append( sum( inLst[ mnDex:mxDex ] ) * 1.0 / ( mxDex - mnDex ) )
    return fltLst

def max_dex( numLst ):
    """ Return the maximum index , First incidence """
    # NOTE: This function assumes that 'numLst' has only numeric elements
    return numLst.index( max( numLst ) )            

# __ End Func __


# == Program Classes ==

# = Utility Classes =

class ListRoll( list ):
    """ Rolling List """

    def __init__( self , pLength ):
        """ Create a list with a marker for the current index """
        list.__init__( self , [ 0 for i in xrange( pLength ) ] )
        self.length  = pLength
        self.currDex = 0

    def add( self , element ):
        """ Insert the element at the current index and increment index """
        self[ self.currDex ] = element
        self.currDex = ( self.currDex + 1 ) % self.length

    def zero_out( self ):
        """ Set numeric data and index to zero """
        self.__init__( self.length )

# _ End Util _


class CarFSM:
    """ A Finite State Machine for autonomous car motion planning """

    def __init__( self ):
        """ Start car """
        # 1. Start the node
        rospy.init_node( 'CarFSM' )
        self.suppress_stop = False
        # 2. Set rate
        try:
            # Attempt to set a refresh rate commensurate with the lidar update rate
            self.heartBeatHz = int( rospy.get_param( "/publ_rate" ) )
        except:
            self.heartBeatHz = 20 # Node refresh rate [Hz]
            rospy.logwarn( "CarFSM: Unable to retrieve update rate! , Setting to " + str( self.heartBeatHz ) + "Hz ..." )

        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time

        # 3. Start subscribers and listeners
        rospy.Subscriber( "/filtered_distance" , Float32MultiArray , self.scan_cb )
        rospy.Subscriber( "/alternat_distance" , Float32MultiArray , self.ocld_cb )

        # Init Flags
        self.useStopDetect = False
        if self.useStopDetect:
            rospy.Subscriber( "/stop_sign" , Bool , self.stop_cb )
        
        # 3.5. Init scan math
        # 3.5.1. Driving Scan 
        self.numReadings     = 100
        self.num_right_scans =   5
        self.scanCenter      = int( self.numReadings//2 ) # + 5 # Cetner of scan with an offset
        self.lastScan        = [ 0.0 for i in range( self.numReadings ) ]
        self.lastScanNP      = np.asarray( self.lastScan )
        # 3.5.2. Collision Scan
        self.ocldScan        = [ 0.0 for i in range( self.numReadings ) ]
        self.ocldScanNP      = np.asarray( self.ocldScan )        

        # 4. Start publishers
        try:
            self.driveTopic = str( rospy.get_param( "/drivtopic" ) )
        except:
            self.driveTopic = "HARE_high_level_command"
            rospy.logwarn( "CarFSM: Unable to retrieve control topic! , Setting to " + self.driveTopic )

        self.drive_pub = rospy.Publisher( 'HARE_high_level_command' , HARECommand , queue_size = 10 )
        self.state_pub = rospy.Publisher( 'ctrl_state_report'       , CarState    , queue_size = 10 )

        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec() # Time that the node was started
        self.lastTime = self.initTime # ----------- Time that the last loop began
        self.prntTime = self.initTime

        # ~~ PID ~~
        self.K_d    = rospy.get_param( "D_VALUE" )
        self.K_i    = rospy.get_param( "I_VALUE" )
        self.K_p    = rospy.get_param( "P_VALUE" )
        self.currUp = 0.0
        self.currUi = 0.0
        self.currUd = 0.0
        # ~ Control Output ~
        self.FLAG_newCtrl   = False # Flag for whether we accept the new control signal
        self.steerAngle     = 0.0
        self.prevSteerAngle = 0.0
        self.prevLinarSpeed = 0.0
        self.angleDiffMin   = rospy.get_param( "ANGLE_DIFF_MIN" ) # limits micro commands
        self.linearSpeed    = rospy.get_param( "LINEAR_SPEED"   ) # [ -1 ,  1 ]    
        # ~ Tracking Memory ~
        self.trackDex = 0
        self.targetLc = False    

        # 8. Test vars
        self.t_test = 0.0
        self.t_incr = 0.025
        self.t_curr = 0.0
        self.t_last = rospy.Time.now().to_sec()
        self.Tstate = 0

        # 9. RC_Control vars
        # Throttle
        self.rc_throttle     =    0.0
        self.throttle_scale  = rospy.get_param( "RC_THROTTLE_SCALE" )
        self.rc_throttle_max = 2000
        self.rc_throttle_mid = 1500
        self.rc_throttle_min = 1000
        # Steering
        self.rc_steering     =  0.0
        self.rc_steering_max = 2003
        self.rc_steering_mid = 1500
        self.rc_steering_min = 1010
        # Flags
        self.FLAG_estop   = False
        self.FLAG_rc_ovrd = False
        rospy.Subscriber( "/rc_raw", RCRaw, self.rc_cb )
        self.rc_msg = RCRaw()    
        self.stopSgnDetected      = False
        #self.suppress_stop        = False
        self.FLAG_stopped_at_sign = False # Have we seen a stop sign?

        # 7. FSM Vars
        self.set_FSM_vars()
        
    def scan_cb( self , msg ):
        """ Process the scan that comes back from the scanner """
        # NOTE: Scan progresses from least theta to most theta: CCW
        if 0: # DIRTY HACK
            self.lastScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]
        else:
            self.lastScan = msg.data
            
    def ocld_cb( self , msg ):
        """ Receive the tight scan, And convert to np array """
        # NOTE: Last three rows of the scan config should be set to whatever avoids obstacles the best
        self.ocldScan   = msg.data
        self.ocldScanNP = np.asarray( self.ocldScan )   

    def rc_cb( self , msg ):
        """ Receive manual commands from the radio """
        self.rc_msg = msg
        # Check for rc control override
        if self.rc_msg.values[6] > 1500:
            rospy.loginfo_throttle(1,"MANUAL RC CONTROL ACTIVE")
            self.FLAG_rc_ovrd = True
        else:
            self.FLAG_rc_ovrd = False

        # Check for e-stop
        if self.rc_msg.values[7] > 1500:
            # INITIALIZE E-STOP
            rospy.loginfo_throttle(1,"E_STOP ACTIVE")
            self.FLAG_estop = True
        else:
            self.FLAG_estop = False

        # Place a deadband around throttle
        band = 50
        if ((self.rc_msg.values[2] < (self.rc_throttle_mid + band)) and (self.rc_msg.values[2] > (self.rc_throttle_mid - band))):
            self.rc_throttle = 0
        else:
            # Scale the incoming pwm signals to usable control commands
            self.rc_throttle = ((self.rc_msg.values[2] - self.rc_throttle_mid)/(self.rc_throttle_max - self.rc_throttle_min))*self.throttle_scale

        self.rc_steering = round((pi/2)*(self.rc_msg.values[0] - self.rc_steering_mid)/(self.rc_steering_max - self.rc_steering_min),2)

    def stop_cb( self , msg ):
        """ Set the stop sign flag """
        # 1. Collect the detection state
        self.stopSgnDetected = msg.data
        # 2. Overwrite the detection state if we wish to suppress
        if self.suppress_stop:
            self.stopSgnDetected = False
            # 3. If the suppression time has expired, cease suppression
            if rospy.Time.now().to_sec() - self.stopped_begin_time > self.stop_suppress_duratn:
                self.suppress_stop      = False
                self.stopped_begin_time = 0.0

    def reset_time( self ):
        """ Set 'initTime' to the current time """
        self.initTime = rospy.Time.now().to_sec() # Time that the node was "started"

    # === DRIVE FINITE STATE MACHINE =======================================================================================================

    def set_FSM_vars( self ):
        """ Set the variables necessary for the FSM controller """

        # ~ Driving Vars ~
        self.err_hist_window = 25 # Width of the integration window
        self.err_hist        = [0]*self.err_hist_window 
        self.tim_hist        = [0]*self.err_hist_window 
        self.wallSetPnt      =  1.0 # [m]
        self.nearN           = 30 # Count this many points as near the average
        self.slope_window    =  2 # Look this many points in the past to compute slope
        self.sum_err         = 0 # used for integral error   
        self.err_win_old     = [0]*2
        self.err_win_new     = [0]*2    
        self.err_derivative  = 0 # for derivative error

        # ~ FSM Vars ~
        self.state           = self.STATE_init # Currently-active state, the actual function
        self.prevState       = self.STATE_init # Previous state, the actual function
        self.seq             =  0 # ------------ Sequence number to give ROS
        self.FLAG_goodScan   = False # --------- Was the last scan appropriate for straight-line driving
        self.reason          = "INIT" # -------- Y U change state?
        self.occlude_indices = [] # ------------ Currently occluded indices
        self.FLAG_backup     = False # --------- Flag set at the beginning of the recovery phase
        self.FLAG_creepF     = False # --------- Flag set at the beginning of the recovery phase
        self.rcovr_bgn_time  = 0.0 # ----------- 
        self.creep_bgn_time  = 0.0 # ----------- 
        self.num_largest     = 15 # ------------ Fix number of largest vals to search for 

        # ~~ State-Specific Constants ~~
        self.two_turn_gains = True # modify some gains to account for the physical differences in the second turn
        self.turn_count = 0 # counts turns
        self.turn_debounce = False
        self.turn_count_db_dur = 2.0 
        # ~ STATE_forward ~
        self.forward_timer = rospy.Time.now().to_sec()
        self.dragOn = 0
        self.drag_speed = 0.5 
        self.drag_duration = 2.0 # seconds 
        self.straight_speed  = 0.32 # Speed for 'STATE_forward' # 0.2 is a fast jog/run
        
        # * TURN TUNING *
        self.max_thresh_dist = 0.0
        self.turn_clamp_right = 0.05
        # Turn 1
        self.turn1_max_thresh_dist = 9.25 - 0.50 # ---------- Above this value we consider distance to be maxed out [m]  # TODO: Try 8 for tighter turns
        self.max_thresh_dist = self.turn1_max_thresh_dist # Turn 1 only
        self.clamp_turn1 = True
        # Turn 2
        self.turn2_max_thresh_dist = self.turn1_max_thresh_dist + 1.3
        self.clamp_turn2 = False
        # * TURN END *
        
        self.thresh_count    = 5 # ------------ If there are at least this many readings above 'self.max_thresh_dist'    
        self.straights_cent_setpoint  = int( self.numReadings/2 )  + 4.0  # Center of scan with an offset, a positive addition should push the car left
        self.straights_cent_setpoint2 = int( self.numReadings/2 )  + 2.0
        self.K_p_straight = self.K_p        
        self.K_d_straight = self.K_d 
        self.K_i_straight = self.K_i
        # ~ STATE_preturn ~
        self.right_side_boost = 2.5 # was 2 
        self.turns_cent_setpoint = int( self.numReadings/2 ) # Center of scan with an offset, a positive addition should push the car left
        self.K_p_turn = 0.10 - 0.00
        self.K_p_t2   = 0.10
        self.preturn_speed = 0.13 # Speed for 'STATE_preturn' # 0.2 is a fast jog/run        
        self.preturn_timer = 0.0
        
        # ** Drifting Vars **
        # Activation 
        self.tokyo_drift = True
        self.t1_drift_on = True
        self.t2_drift_on = True
        self.drift_speed = 1.0 # full speed to break free tires
        self.drift_start = 0.33 - 0.230 # 0.75 was this, setting to 0 to visualize when the steering angle trigger happens
        self.drift_duration = 0.280 + 0.05 # 0.100 # milliseconds, set very high to ensure spotting the angle trigger
        self.t2_drift_duration = 0.280 - 0.08
        self.turn_based_drift = True
        self.drift_steer_trigger = 0.75 
        self.enable_counter_steer = True
        self.counter_steer_angle = -1.0 # will need to tune this
        self.counter_steer_start = 0.400 # milliseconds of lag behind start of drift
        self.counter_steer_duration = 0.200
        # * End Drift *
        
        # ~ STATE_collide_recover ~
        self.recover_speed    = -0.15 # Back up at this speed
        self.recover_duration =  1.00 # Minimum time to recover
        self.recover_timeout  =  5.00 # Maximum time to recover
        self.K_p_backup       =  0.03 # Backup Kp, should be much less than forward as the dyn's are different
        self.occlude_dist     =  0.75  # Maximum distance for which a scan reading is considered occluded
        self.occlude_limit    = 30 # -- Minimum number of occluded scan readings that indicate view occlusion    
        # ~ STATE_seek_open ~
        self.seek_speed       =  0.10 # Creep forward at this speed
        self.K_p_creep        =  0.06 #- Fwd Kp, should be a little more than reverse to make sure we get around the foot/box
        self.creep_scan_dist  =  1.5 #- Distance criterion to declare an unobstructed path
        self.creep_scan_count = 20 # -- Count criterion to declare an unobstructed path

        # ~~~ CAREFUL SETTINGS : For slow challenges && Emergency Backup stable lap settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        self._CAREFUL_SETTINGS = 0 # NOTE: SET TO 1 FOR { A. BOX RUN , B. STOP SIGN CHALLENGE? }

        # FROZEN SETTINGS FOR CALM DRIVING
        if self._CAREFUL_SETTINGS:
            # ~~ State-Specific Constants ~~
            self.two_turn_gains = False
            # ~ STATE_forward ~
            self.straight_speed  = 0.13 # Speed for 'STATE_forward' # 0.2 is a fast jog/run
            self.max_thresh_dist = 9.0 # ---------- Above this value we consider distance to be maxed out [m]
            self.thresh_count    = 5 # ------------ If there are at least this many readings above 'self.max_thresh_dist'    
            self.straights_cent_setpoint = int( self.numReadings/2 ) # + 5  # Center of scan with an offset, a positive addition should push the car left
            self.K_p_straight = 0.0325        
            self.K_d_straight = 0.0085 
            self.K_i_straight = 0.0000
            # ~ STATE_preturn ~
            self.preturn_max_thresh_dist = 5.0
            self.right_side_boost        = 2.0
            self.turns_cent_setpoint = int( self.numReadings/2 ) # Center of scan with an offset, a positive addition should push the car left
            self.K_p_turn      = 0.015     
            self.preturn_speed = 0.15 # Speed for 'STATE_preturn' # 0.2 is a fast jog/run        
            self.tokyo_drift = False
            # ~ STATE_collide_recover ~
            self.recover_speed    = -0.15 # Back up at this speed
            self.recover_duration =  0.50 # Minimum time to recover
            self.recover_timeout  =  3.00 # Maximum time to recover
            self.K_p_backup = 0.01
            # ~ STATE_seek_open ~
            self.seek_speed    = 0.10 # Creep forward at this speed
            self.K_p_creep     = 0.025 # Proportional gain for obstacle avoidance
            self.creep_timeout = 3.00 # Maximum time to recover
            # STATE_stop_for_sign
            self.stopped_begin_time   =  0.0
            self.cached_state         = self.STATE_init
            self.stopsign_delay       =  0.00
            self.stopsign_duration    =  8.0    
            self.stop_suppress_duratn = 20.0

    def eval_scan( self ):
        """ Populate the threshold array and return its center """
        SHOWDEBUG = 0
        # 1. Determine which scan values are above the threshold
        self.lastScanNP = np.asarray( self.lastScan )
        self.above_thresh = np.where( self.lastScanNP > self.max_thresh_dist )[0]

        # 1.5 Grab N largest 
        self.sorted_scan_inds = self.lastScanNP.argsort() # sorted from smallest to largest
        self.N_largest_inds = self.sorted_scan_inds[-self.num_largest:]

        # 2. Predicate: Was the latest scan a good scan?
        if len( self.above_thresh ) >= self.thresh_count:
            self.FLAG_goodScan = True
        else:
            self.FLAG_goodScan = False
            self.lastScanNP[-20:] = self.lastScanNP[-20:] + self.right_side_boost
            self.sorted_scan_inds = self.lastScanNP.argsort() # sorted from smallest to largest
            self.N_largest_inds = self.sorted_scan_inds[-self.num_largest:]
            
        if SHOWDEBUG:
            print "Scan: ___________" , self.lastScan
            print "Scan np: ________" , self.lastScanNP
            print "Threshold Arr: __" , self.above_thresh
            print "Threshold Center:" , np.mean( self.above_thresh )
            print "Good Scan: ______" , ( "YES" if self.FLAG_goodScan else "NO" )
            print
        # 3. Return the center of the above-threshold values
        # return np.mean( self.above_thresh )
        return np.mean( self.N_largest_inds )

    def scan_occluded( self , distThresh , countThresh , offset = 0 ):
        """ Return True if the last scan has more than the designated number of very-near readings """
        # NOTE: This function assumes that eval_scan has already been run
        SHOWDEBUG = 0
        # 1. Determine which scan values are above the threshold
        self.occlude_indices = np.where( self.ocldScanNP <= distThresh + offset )[0]
        # 2. Predicate: Was the latest scan a good scan?
        if 1:
            if len( self.occlude_indices ) >= countThresh:
                return True
            else:
                return False    
        else:
            return False

    def report_state( self ):
        """ Accumulate and publish controller state """
        # 1. Create msg
        msg = CarState()
        # 2. Populate msg
        self.seq += 1
        msg.header.seq      = self.seq # ---------- Sequence number
        msg.header.stamp    = rospy.get_rostime() # Time info
        msg.header.frame_id = "car" # ------------- What is it
        msg.state           = self.state.__name__ # Name of the state
        msg.reason          = self.reason # ------- Reason for transition
        msg.FLAG_newCtrl    = self.FLAG_newCtrl # - Was there a new control command issued this timestep?
        msg.FLAG_goodScan   = self.FLAG_goodScan #- Was there a good scan noted this timestep?
        msg.steerAngle      = self.steerAngle  # -- Steering angle demanded
        msg.linearSpeed     = self.linearSpeed  # - Motor speed demanded
        msg.up              = self.currUp # ------- Proportional part of the control signal
        msg.ui              = self.currUi # ------- Integral part of the control signal
        msg.ud              = self.currUd # ------- Derivative part of the control signal
#        msg.turn_count      = int( self.turn_count ) # --- What turn is coming up?
#        msg.preturn_timer   = self.preturn_timer #- Drift timing
#        msg.forward_timer   = self.forward_timer #- Amount of time in straightaway
#        msg.turn_debounce   = self.turn_debounce  #- Are we suppressing a turn count?    
        # 3. Publish msg
        self.state_pub.publish( msg )

    def update_err( self , x_act , x_target ):
        """ Calc the scalar error = 'x_act - x_target' and update I and D structures """
        # 1. Calc the loop duration
        nowTime = rospy.Time.now().to_sec() # -- Get the current time
        loopDuration = nowTime - self.lastTime # Time delta between the start of this loop and the start of the last loop
        self.tim_hist.append( loopDuration )
        if len( self.tim_hist ) > self.err_hist_window:
            self.tim_hist.pop(0)
        self.lastTime = nowTime
        # 2. Calc the error
        err = x_act - x_target
        self.err_hist.append( err )
        if len( self.err_hist ) > self.err_hist_window:
            self.err_hist.pop(0)

        self.err_win_new = self.err_hist[-2:]
        self.err_win_old = self.err_hist[-4:-2]
        self.err_derivative  = np.mean(self.err_win_new) - np.mean(self.err_win_old) 
        # acts as antiwindup... ignoring case when theta is at max, but thats uncommon in our scenario
        if self.FLAG_goodScan and self.state == self.STATE_forward: 
            self.sum_err += err
        return err

    def integrate_err( self ):
        """ Rectangular integration of error over time """
        curveArea = 0.0
        for i in xrange( self.err_hist_window ):
            curveArea += self.err_hist[i] * self.tim_hist[i]
        return curveArea

    def rate_err( self ):
        """ Average time rate of change in error over a window """
        method  = 1
        # A. Average Slope
        if method:
            errChange = self.err_hist[ -1 ] - self.err_hist[ -self.slope_window ]
            timChange = sum( self.tim_hist[ -self.slope_window: ] )
            return errChange / timChange
        # B. Average of Slopes
        else:
            rateTot = 0.0
            for i in xrange( self.slope_window ):
                change = self.err_hist[ -(i) ] - self.err_hist[ -(i+1) ]
                if self.tim_hist[ -(i) ] > 0:
                    rateChange = change / self.tim_hist[ -(i) ]
                else:
                    rateChange = 0.0
                rateTot += rateChange
            return rateTot / self.slope_window
        
    def clear_PID( self ):
        """ Clear the PID history left by a previous state """
        self.err_hist = [0]*self.err_hist_window 
        self.tim_hist = [0]*self.err_hist_window 
        self.sum_err = 0
        self.err_derivative  = 0
        
    def steer_center( self , P_gain , reverse = 0 , useAlt = True ):
        """ PID Controller on the max value of the scan , Return steering command """
        useID = False
        qFltr = False
        # 1. Use the alternate (tight/occlusion) scan unless the user specifies
        if useAlt:
            if qFltr:
                filtScan = avg_filter( self.ocldScanNP ) # does some level of filtering on the scan
            else:
                filtScan = list( self.ocldScanNP )
        else:
            filtScan = avg_filter( self.lastScanNP ) # does some level of filtering on the scan
        centrDex = max_dex( filtScan )
        # 2. Reverse if the user specifies
        if reverse:
            factor = -1.0
        else:
            factor =  1.0
        # Calc a new effort 
        translation_err  = self.update_err( centrDex , self.cent_setpoint )
        self.currUp      = P_gain * translation_err
        if useID:
            self.currUi      = self.K_i * self.integrate_err()
            self.currUd      = self.K_d * self.rate_err() # This should be a
        else:
            self.currUi = 0.0
            self.currUd = 0.0
        auto_steer       = self.currUp + self.currUi + self.currUd # NOTE: P-ctrl only for demo
        # Return the total PID steer effort, reversing if 
        return auto_steer * factor
    
    @staticmethod
    def index_of_max_half( arr ):
        """ Choose the half of 'arr' that has the highest average , and return the overall index of the max of that half """
        mid   = len( arr ) // 2
        half  = mid // 2
        left  = arr[ : mid ]
        rght  = arr[ mid : ]

        return mid + rght.index( max( rght ) )

        #if np.average( left ) > np.average( rght ):
        #    return left.index( max( left ) )
        #else:
        #    return mid + rght.index( max( rght ) )
    
    def lock_and_seek( self , P_gain , reverse = 0 , useAlt = True ):
        """ Find the center of the half with the highest average, and track it """
        searchWidth = 2
        cenDex      = self.numReadings//2
        # 0. Load appropriate arr
        if useAlt:
            trackScan = self.ocldScan
        else:
            trackScan = self.lastScan
        # 1. If we have not locked onto a target, then do so
        offset = 30
        isLeftLock = False
        if not self.targetLc:
            self.trackDex = CarFSM.index_of_max_half( trackScan )
            self.targetLc = True
            if self.trackDex < 50:
                isLeftLock = True
#            if self.trackDex < 50:
 #              self.trackDex += offset
  #          else:
   #            self.trackDex -= offset
    #        self.targetLc = True
        # 2. Assuming we have target lock, search the viscinity of the last lock and update lock index
        loDex  = max( 0                , self.trackDex - searchWidth )
        hiDex  = min( len( trackScan ) , self.trackDex + searchWidth )
        window = trackScan[ loDex : hiDex ]
        self.trackDex = loDex + window.index( max( window ) )
        # 2. Reverse if the user specifies
        if reverse:
            factor = -1.0
        else:
            factor =  1.0    
        # 3. Calc error to lock location
        offset = 5
        if isLeftLock:
            tempDex = max( 0 , self.trackDex - offset )
            
        else:
            tempDex = min( 100 , self.trackDex + offset )
        translation_err = self.update_err( tempDex , cenDex )
        self.currUp     = P_gain * translation_err
        auto_steer      = self.currUp + self.currUi + self.currUd # NOTE: P-ctrl only for demo
        # Return the total PID steer effort, reversing if 
        return ( auto_steer * factor ) , ( translation_err < searchWidth )

    def seek_largest_opening( self , threshDist , P_gain , reverse = 0 , useAlt = True ):
        """ Look for the largest free space and drive for it , RATIONALE: Avoid trying to preserve a target lock between iterations """
        # WHY: 1. Max is noisy , 2. Actual target may be moving outside of our lock window which voids our assumption
        # 0. Load appropriate arr
        if useAlt:
            trackScan = self.ocldScan
        else:
            trackScan = self.lastScan        
        # 1. Separate the scan into near and far blocks , Find the longest far block, and return it's center
        bestLen  = 0
        bestCnt  = 0.0
        currLen  = 0
        bgnBlock = 0
        endBlock = 0
        blockOn  = 0
        for i , reading in enumerate( trackScan ):
            if reading > threshDist:
                if not blockOn:
                    blockOn  = 1
                    bgnBlock = i
                    currLen  = 1
                else:
                    currLen += 1
            else:
                if blockOn:
                    blockOn  = 0
                    endBlock = i - 1
                    if currLen > bestLen:
                        bestCnt = ( bgnBlock + endBlock ) / 2.0
                currLen = 0
                blockOn = 0
        else:
            if blockOn:
                endBlock = len( trackScan ) - 1
                if currLen > bestLen:
                    bestCnt = ( bgnBlock + endBlock ) / 2.0
        # 2. Reverse if the user specifies
        if reverse:
            factor = -1.0
        else:
            factor =  1.0     
        # 3. Calc control effort
        cenDex          = self.numReadings//2
        translation_err = self.update_err( bestCnt , cenDex )
        self.currUp     = P_gain * translation_err
        auto_steer      = self.currUp # + self.currUi + self.currUd # NOTE: P-ctrl only for demo
        # Return the total PID steer effort, reversing if 
        return auto_steer * factor       
                

    """
    STATE_funcname
    # ~   I. State Calcs   ~
    # ~  II. Set controls  ~
    # ~ III. Transition Determination ~
    # ~  IV. Clean / Update ~
    """

    def STATE_init( self ):
        """ Initial state , Determine the drving mode """
        
        SHOWDEBUG = 0
        if SHOWDEBUG:
            print "STATE_init" , self.reason

        # ~   I. State Calcs   ~
        self.eval_scan() # Assess straight-line driving
        self.reset_time()
        self.turn_count = 0 # reset the turn count
        self.turn_debounce = False # ensure debounce is reset

        # ~  II. Set controls  ~
        self.steerAngle  = 0.0
        self.linearSpeed = 0.0

        # ~ III. Transition Determination ~
        # NOTE: At the moment, safest transition is probably to wait for the first straightaway
        # A. If there is a good straightaway scan, then set the forward state
        if self.FLAG_goodScan:
            self.state  = self.STATE_forward
            self.reason = "OVER_THRESH"
        # B. Otherwise, wait for a good hallway scan (This way we can troubleshoot just by wathcing wheel motion)
        else:
            self.state  = self.STATE_init
            self.reason = "UNDER_THRESH"

        # ~  IV. Clean / Update ~
        # NONE

    def STATE_forward( self ):
        """ Straightaway driving , Monitor for turn or fault """
        self.cent_setpoint = self.straights_cent_setpoint

        if rospy.Time.now().to_sec() - self.forward_timer > self.turn_count_db_dur and not self.turn_debounce:
            self.turn_debounce = True
            #if self.turn_count == 1:
            self.turn_count += 1

        SHOWDEBUG = 0
        if SHOWDEBUG:
            print "STATE_forward" , self.reason

        # turn two specifics
        if self.two_turn_gains and self.turn_count == 2:
            self.max_thresh_dist = self.turn2_max_thresh_dist
        else: 
            self.max_thresh_dist = self.turn1_max_thresh_dist

        if self.turn_count > 1:
            self.straights_cent_setpoint = self.straights_cent_setpoint2

        if rospy.Time.now().to_sec() - self.forward_timer < 0.5:
            self.sum_err = 0

        if self.dragOn:
            if self.turn_count < 2 and (rospy.Time.now().to_sec() - self.forward_timer) < self.drag_duration:
                self.linearSpeed = self.drag_speed  
            else: 
                self.linearSpeed = self.straight_speed


        # ~   I. State Calcs   ~
        # 1. Calculate and store error
        input_center = self.eval_scan()

        # ~  II. Set controls  ~
        # Calc a new forward effort only if there is a good hallway scan
        if self.FLAG_goodScan:
            # translation_err  = input_center * 1.0 - self.scanCenter
            translation_err  = self.update_err( input_center , self.cent_setpoint )
            self.currUp      = self.K_p_straight * translation_err
            self.currUi      = self.K_i_straight * self.sum_err
            self.currUd      = self.K_d_straight * self.err_derivative 
            auto_steer       = self.currUp + self.currUi + self.currUd 

            # Control Effort
            self.steerAngle  = auto_steer
            # self.steerAngle  = -auto_steer
            self.linearSpeed = self.straight_speed 

        # ~ III. Transition Determination ~
        if self.FLAG_goodScan:
            self.state  = self.STATE_forward
            self.reason = "OVER_THRESH"
        else:
            self.state  = self.STATE_pre_turn
            self.reason = "UNDER_THRESH"
            self.preturn_start_time = rospy.Time.now().to_sec()
            self.one_shot = True 
        # Z. Crash Recover Override
        if self.scan_occluded( self.occlude_dist , self.occlude_limit ) and self._CAREFUL_SETTINGS:
            self.state  = self.STATE_collide_recover
            self.reason = "OCCLUSION"        

        # ~  IV. Clean / Update ~
        # If we are exiting the state, then clear the PID
        if self.state != self.STATE_forward:
            self.clear_PID()

    def STATE_pre_turn( self ):
        """ Approaching halway end, watch for corner detector """
        self.cent_setpoint = self.turns_cent_setpoint
        self.drift_stop = self.drift_start + self.drift_duration
        self.counter_steer_stop = self.counter_steer_start + self.counter_steer_duration

        SHOWDEBUG = 0
        if SHOWDEBUG:
            print "STATE_pre_turn" , self.reason

        # ~   I. State Calcs   ~
        self.preturn_timer = rospy.Time.now().to_sec() - self.preturn_start_time
        self.turn_debounce = False
        input_center  = self.eval_scan()

        # turn two specifics
        if self.two_turn_gains and self.turn_count >= 2:
            self.dirft_duration = self.t2_drift_duration

        # ~  II. Set controls  ~
        if self.FLAG_goodScan: pass # will transition below
        else: 
            translation_err  = self.update_err( input_center , self.cent_setpoint )
            self.currUp      = self.K_p_turn * translation_err
            auto_steer       = self.currUp
            self.steerAngle  = auto_steer # Control Effort
            
            # --------- drifting code 
            # A. Decide to drift , based on selection flags
            if self.turn_count == 1:
                driftNow = self.tokyo_drift and self.t1_drift_on
            elif self.turn_count == 2:
                driftNow = self.tokyo_drift and self.t2_drift_on
            else:
                driftNow = False
            # B. If we have decided to drift, apply drift control
            if driftNow:
                if self.turn_based_drift: # reset the timer so it starts during the turn section
                    if self.currUp > self.drift_steer_trigger and self.one_shot: # start timer once the car begins its right turn
                        self.preturn_start_time = rospy.Time.now().to_sec()
                        self.one_shot = False
                    elif self.one_shot:
                        self.preturn_start_time = rospy.Time.now().to_sec() # keep spinning the timer till turn angle is hit
                if self.drift_start < self.preturn_timer < self.drift_stop:
                    self.linearSpeed = self.drift_speed
                    self.steerAngle = 1.5 # might need to ditch this....
                else: 
                    self.linearSpeed = self.preturn_speed    
                if self.enable_counter_steer:
                    if self.drift_start + self.counter_steer_start < self.preturn_timer < self.drift_start + self.counter_steer_stop:
                        self.steerAngle = self.counter_steer_angle
            # --------- end drifting code 
            else: 
                self.linearSpeed = self.preturn_speed 

        # Clamp steering to prevent indecision 
        # FIXME: before or after countersteer?
        if  ( ( self.turn_count == 1 ) and ( self.clamp_turn1 ) )  or  ( ( self.turn_count == 2 ) and ( self.clamp_turn2 ) ):
            self.steerAngle = max( self.steerAngle , self.turn_clamp_right )

        # ~ III. Transition Determination ~
        if self.FLAG_goodScan:
            self.state  = self.STATE_forward
            self.reason = "OVER_THRESH"
            self.clear_PID()
            self.forward_timer = rospy.Time.now().to_sec() # start forward timer
        else:
            self.state  = self.STATE_pre_turn
            self.reason = "UNDER_THRESH"

        # Z. Crash Recover Override
        if self.scan_occluded( self.occlude_dist , self.occlude_limit ) and self._CAREFUL_SETTINGS:
            self.state  = self.STATE_collide_recover
            self.reason = "OCCLUSION"                

        
    def STATE_collide_recover( self ):
        """ Back up from a collision """
        
        # ~   I. State Calcs   ~
        if not self.FLAG_backup:
            self.FLAG_backup    = True
            self.rcovr_bgn_time = rospy.Time.now().to_sec()
        # self.occlude_limit += 5
        # print "Occlude Limit:" , self.occlude_limit , ".. ADJUSTED!"
        # ~  II. Set controls  ~
        self.linearSpeed = self.recover_speed
        # self.steerAngle  = 0.00
        if 0:
            self.steerAngle = self.steer_center( self.K_p_backup , reverse = 1 )
        elif 0:
            self.steerAngle , found = self.lock_and_seek( self.K_p_backup , reverse = 1 )
        else:
            self.steerAngle = 0.0
        # ~ III. Transition Determination ~
        # A. If the min time has not passed, continue to recover
        nowTime = rospy.Time.now().to_sec()
        if nowTime - self.rcovr_bgn_time <= self.recover_duration:
            self.state  = self.STATE_collide_recover
            self.reason = "UNDER_MIN_TIME"        
        # B. Else the minimum time has passed
        else:
            # i. If the occlusion has been cleared, then seek open space
            if not self.scan_occluded( self.occlude_dist , self.occlude_limit , 0.5):
                self.FLAG_backup = False
                self.eval_scan()
                # a. If there is an open hallway, GO
                self.state  = self.STATE_seek_open
                self.reason = "UNDER_THRESH"        
            # ii. Else the occlusion has not cleared
            else:
                # iii. If the max time has not elapsed, continue to recover
                if nowTime - self.rcovr_bgn_time <= self.recover_timeout:
                    self.state  = self.STATE_collide_recover
                    self.reason = "OCCLUDED_UNDER_MAX_TIME"                    
                # iv. If the max time has passed without clearing occlusion, fail to init
                else:
                    self.state       = self.STATE_init
                    self.reason      = "RECOVERY_FAILURE"        
                    self.FLAG_backup = False
        # ~  IV. Clean / Update ~        
        if self.state != self.STATE_collide_recover:
            self.clear_PID()
            self.targetLc = False
        
    def STATE_seek_open( self ):
        """ Slowly drive towards the most open portion of the scan """
        # ~   I. State Calcs   ~
        self.eval_scan()
        if not self.FLAG_creepF:
            self.FLAG_creepF    = True
            self.creep_bgn_time = rospy.Time.now().to_sec()        
        # ~  II. Set controls  ~
        self.linearSpeed = self.seek_speed
        if 0:
            self.steerAngle = self.steer_center( self.K_p_creep )    
        elif 1:
            self.steerAngle , found = self.lock_and_seek( self.K_p_creep )        
        else:
            threshDist = 2.0
            self.steerAngle = self.seek_largest_opening( threshDist , self.K_p_creep )
        # ~ III. Transition Determination ~
        
        nowTime = rospy.Time.now().to_sec()
        # a. If the min backup time not expired
        if nowTime - self.creep_bgn_time <= self.creep_timeout:
            self.state  = self.STATE_seek_open
            self.reason = "UNDER_MIN_TIME"     
        # b. Else min backup time expired, time to check for a clear path
        else:
        # c. If there is an open hallway, GO
            #if self.FLAG_goodScan:
#            if not self.scan_occluded( self.occlude_dist , self.occlude_limit ):
 #               self.state = self.STATE_collide_recover
  #              self.reason = "RE_COLLIDE"
   #             self.occlude_limit += 5
            if not self.scan_occluded( self.creep_scan_dist , self.creep_scan_count ):
                self.state  = self.STATE_forward
                self.reason = "OCCLUDE_SCAN_CLEAR"
            # d. Else creep forward
            else:
                self.state  = self.STATE_seek_open
                self.reason = "UNDER_THRESH" 
   
        # ~  IV. Clean / Update ~
        # If we are exiting the state, then clear the PID
        if self.state != self.STATE_seek_open:
            self.clear_PID()
            self.targetLc = False

    def STATE_stop_for_sign( self ):
        """ Idle at the stopsign for X seconds then resume at the saved state """
        # ~   I. State Calcs   ~
        # 1. If we just entered the state, set flag
        if not self.FLAG_stopped_at_sign:
            self.FLAG_stopped_at_sign = True
            self.stopped_begin_time   = rospy.Time.now().to_sec()
            self.cached_state         = self.STATE_forward
        # ~  II. Set controls  ~
        #self.linearSpeed = 0.0 # Obviously
        # ~ III. Transition Determination ~
        # 3. If the minimum dwell time has not elapsed, remain in this state
        if rospy.Time.now().to_sec() - self.stopped_begin_time <= self.stopsign_delay:
            print "ABOUT TO STOP"
            self.state  = self.STATE_stop_for_sign
            self.reason = "UNDER_DELAY_TIME"
            self.linearSpeed = 0.09
        elif rospy.Time.now().to_sec() - self.stopped_begin_time <= self.stopsign_duration:
            print "STOPPED"
            self.state  = self.STATE_stop_for_sign
            self.reason = "UNDER_DWELL_TIME"
            self.linearSpeed = 0.0
        # 4. Else restored the cached state
        else:
            print "RESUME"
            self.state                = self.STATE_forward # -- Restore prev state
            self.reason               = "RESTORE_AFTER_STOP" # We done stopping
            self.FLAG_stopped_at_sign = False # -------------- We done stopping
            self.suppress_stop        = True # --------------- Do not stop twice
            self.cached_state         = self.STATE_init # ---- Until it is overwritten
    
        # ~  IV. Clean / Update ~    
        self.reset_time() # Do not allow a stop to stall controls
        
    def hallway_FSM( self ):
        """ Execute state actions and record current status """
        # NOTE: Each state must handle its own data collection, processing, control setting, and transition
        self.prevState = self.state # Update previous
        self.state() # ------ State actions and transition
        self.report_state() # Publish state info
        if self.useStopDetect and self._CAREFUL_SETTINGS and self.stopSgnDetected:
            self.state  = self.STATE_stop_for_sign
            self.reason = "STOPSIGN_DETECTED"
        if self.state != self.prevState or ((rospy.Time.now().to_sec() - self.prntTime) > 0.5): # only print on transtion
            print self.state.__name__ , ',' , self.reason , ',' , self.steerAngle , ',' , self.linearSpeed
            self.prntTime = rospy.Time.now().to_sec()
        

    # ___ END FSM __________________________________________________________________________________________________________________________

    def dampen_micro_cmds( self ):
        """ Check to see if the new steering angle is large enough to warrant a command this prevents micro commands to the servo
            self.FLAG_newCtrl:
                True  - There is a sufficiently different command to publish
                False - Microcommand, do not publish """
        if ( np.abs( self.steerAngle - self.prevSteerAngle ) > self.angleDiffMin )  or  ( not eq_margin( self.linearSpeed , self.prevLinarSpeed ) ) \
        or ( rospy.Time.now().to_sec() - self.initTime < 0.25 ):
            self.prevSteerAngle = self.steerAngle
            self.prevLinarSpeed = self.linearSpeed
            self.FLAG_newCtrl   = True
        else:
            self.FLAG_newCtrl   = False

    def run( self ):
        """ Take a distance reading and generate a control signal """

        # 1. While ROS is running
        while ( not rospy.is_shutdown() ):

            # 1. Calculate a control effort
            #self.FLAG_newCtrl = True # HACKKKKKKKKKKKKKKKKKKKKKKKKKK
            if 1: # Enable Finite State Machine
                self.hallway_FSM()
                self.dampen_micro_cmds()
                
            else: # Enable test - Unchanging open loop command
                self.test_state()
                self.FLAG_newCtrl   = True

            if self.FLAG_estop:
                rospy.loginfo_throttle( 1 , "ESTOP ENGAGED" )
                self.linearSpeed = 0.0
                self.reset_time() # Reset the clock so that the car does not get stuck on resume
                self.clear_PID()
                self.forward_timer = rospy.Time.now().to_sec()  
                self.drive_pub.publish(  compose_HARE_ctrl_msg( self.steerAngle , self.linearSpeed )  )
                print('Turn count: ',self.turn_count,' this should never be more than 2!!')
            else: # only transmit control effort if not Estopped
                # 2. Transmit the control effort
                if (self.FLAG_newCtrl and not self.FLAG_rc_ovrd):
                    #print( "Steering Angle:" , self.steerAngle , ", Speed:" , self.linearSpeed )
                    self.drive_pub.publish(  compose_HARE_ctrl_msg( self.steerAngle , self.linearSpeed )  )
                elif (self.FLAG_rc_ovrd):
                    self.drive_pub.publish(  compose_HARE_ctrl_msg( self.rc_steering , self.rc_throttle )  )
                    self.reset_time() # Reset the clock so that the car does not get stuck on resume

            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep()

        # N. Post-shutdown activities
        else:
            print "Node Shutdown after" , rospy.Time.now().to_sec() - self.initTime , "seconds"


    def test_state( self ):
        """ Run the drive motor forwards and backwards while performing a sine sweep on the steering angle """
        # 1. Set the steering angle
        self.t_curr += self.t_incr
        self.steerAngle = pi/4 * sin( self.t_curr )
        print "Test Steering:" , self.steerAngle
        # 2. Set the throttle
        if 1:
            self.linearSpeed = 0.0
        else:
            self.t_curr = rospy.Time.now().to_sec()
            # A. If more than 1 second has passed since the last transition, advance to the next speed state
            if self.t_curr - self.t_last > 1.0:
                self.Tstate      = ( self.Tstate + 1 ) % len( _STATETRANS ) # Warning! Quickly reversing the motor will heat up the voltage converter A LOT
                self.linearSpeed = _STATETRANS[ self.Tstate ]
                self.t_last      = self.t_curr



# __ End Class __


if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:] # Terminal arguments , if they exist

    FOO = CarFSM()
    FOO.run()


# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________
