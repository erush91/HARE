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
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
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
        # self.FLAG_ = False

        # 3.5. Init scan math
        # 3.5.1. Driving Scan 
        self.numReadings     = 100
        self.num_right_scans =   5
        self.old_right_mean  =   0.0 # np.ones((self.num_right_scans))*20
        self.prev_rghtmost   =   0.0
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

        # ~ PID ~
        self.K_d = rospy.get_param( "D_VALUE" )
        self.K_i = rospy.get_param( "I_VALUE" )
        self.K_p = rospy.get_param( "P_VALUE" )

        # 7. FSM Vars
        self.set_FSM_vars()

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
        self.rc_steering     =    0.0
        self.rc_steering_max = 2000
        self.rc_steering_mid = 1500
        self.rc_steering_min = 1000
        # Flags
        self.FLAG_estop   = False
        self.FLAG_rc_ovrd = False
        rospy.Subscriber( "/rc_raw", RCRaw, self.rc_cb )
        self.rc_msg = RCRaw()

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

    def reset_time( self ):
        """ Set 'initTime' to the current time """
        self.initTime = rospy.Time.now().to_sec() # Time that the node was "started"

    # === DRIVE FINITE STATE MACHINE =======================================================================================================

    def set_FSM_vars( self ):
        """ Set the variables necessary for the FSM controller """

        # ~ Control Output ~
        self.FLAG_newCtrl   = False # Flag for whether we accept the new control signal
        self.steerAngle     = 0.0
        self.prevSteerAngle = 0.0
        self.prevLinarSpeed = 0.0
        self.angleDiffMin   = rospy.get_param( "ANGLE_DIFF_MIN" ) # limits micro commands
        self.linearSpeed    = rospy.get_param( "LINEAR_SPEED"   ) # [ -1 ,  1 ]    
        
        # ~ Driving Vars ~
        self.err_hist_window = 25 # Width of the integration window
        self.err_hist        = [0]*self.err_hist_window 
        self.tim_hist        = [0]*self.err_hist_window 
        self.wallSetPnt      =  1.0 # [m]
        self.nearN           = 30 # Count this many points as near the average
        self.slope_window    =  2 # Look this many points in the past to compute slope
        self.rhgt_rolling    = ListRoll( self.num_right_scans )    
        self.sum_err         = 0 # used for integral error   
        self.err_win_old     = [0]*2
        self.err_win_new     = [0]*2    
        self.err_derivative  = 0 # for derivative error

        # ~ FSM Vars ~
        self.state           = self.STATE_init # Currently-active state, the actual function
        self.seq             =  0 # ------------ Sequence number to give ROS
        self.FLAG_goodScan   = False # --------- Was the last scan appropriate for straight-line driving
        self.reason          = "INIT" # -------- Y U change state?
        self.occlude_dist    =  0.25 # --------- Maximum distance for which a scan reading is considered occluded
        self.occlude_limit   = 33 # ------------ Minimum number of occluded scan readings that indicate view occlusion
        self.occlude_indices = [] # ------------ Currently occluded indices
        self.FLAG_backup     = False # --------- Flag set at the beginning of the recovery phase
        self.rcovr_bgn_time  = 0.0
        self.num_largest = 15 # fix number of largest vals to search for 

        # ~ PID Vars ~
        self.currUp = 0.0
        self.currUi = 0.0
        self.currUd = 0.0

        # ~ State-Specific Constants ~
        # STATE_forward
        self.straight_speed  = 0.32 # Speed for 'STATE_forward' # 0.2 is a fast jog/run
        self.max_thresh_dist = 9.0 # ---------- Above this value we consider distance to be maxed out [m]
        self.thresh_count    = 5 # ------------ If there are at least this many readings above 'self.max_thresh_dist'    
        self.straights_cent_setpoint = int( self.numReadings/2 ) # + 5  # Center of scan with an offset, a positive addition should push the car left
        self.K_p_straight = self.K_p        
        self.K_d_straight = self.K_d 
        self.K_i_straight = self.K_i
        # STATE_preturn
        self.preturn_max_thresh_dist = 5.0
        self.right_side_boost = 2.0
        self.preturn_angle  = 0.5 # Hard-coded turn angle for preturn
        self.turns_cent_setpoint = int( self.numReadings/2 ) # Center of scan with an offset, a positive addition should push the car left
        self.K_p_turn = 0.07     
        self.preturn_speed = 0.25 # Speed for 'STATE_preturn' # 0.2 is a fast jog/run
        self.tokyo_drift = False
        self.drift_speed = 0.0 # full speed to break free tires
        self.preturn_start = 0.75
        self.preturn_stop = 0.85
        # TODO: control during preturn to 
        self.crnr_drop_dist = 0.65 # Increase in distance of the rightmost reading that will cause transition to the turn state
        # STATE_blind_right_turn
        self.turning_speed = 0.08 # Speed for 'STATE_blind_rght_turn'
        self.turning_angle = 1.5 # Turn angle for 'STATE_blind_rght_turn'
        # STATE_collide_recover
        self.recover_speed    = -0.15 # Back up at this speed
        self.recover_duration =  0.10 # Minimum time to recover
        self.recover_timeout  = 2.00 # Maximum time to recover
        self.K_p_careful = 0.014
        # STATE_seek_open 
        self.seek_speed = 0.10 # Creep forward at this speed

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
            self.above_thresh = np.where( self.lastScanNP > self.preturn_max_thresh_dist )[0]
            self.sorted_scan_inds = self.lastScanNP.argsort() # sorted from smallest to largest
            self.N_largest_inds = self.sorted_scan_inds[-self.num_largest:]
            if len( self.above_thresh ) >=self.thresh_count:
                pass
            else: print('didnt expect this, could be a problem')
            
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

    def scan_occluded( self ):
        """ Return True if the last scan has more than the designated number of very-near readings """
        # NOTE: This function assumes that eval_scan has already been run
        SHOWDEBUG = 0
        # 1. Determine which scan values are above the threshold
        self.occlude_indices = np.where( self.ocldScanNP <= self.occlude_dist )[0]
        # 2. Predicate: Was the latest scan a good scan?
        if len( self.occlude_indices ) >= self.occlude_limit:
            return True
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
        # self.rhgt_rolling.zero_out()
        
    def steer_center( self , P_gain , reverse = 0 , useAlt = True ):
        """ PID Controller on the max value of the scan , Return steering command """
        useID = False
        # 1. Use the alternate (tight/occlusion) scan unless the user specifies
        if useAlt:
            filtScan = avg_filter( self.ocldScanNP ) # does some level of filtering on the scan
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

    """
    STATE_funcname
    # ~   I. State Calcs   ~
    # ~  II. Set controls  ~
    # ~ III. Transition Determination ~
    # ~  IV. Clean / Update ~
    """

    def STATE_init( self ):
        """ Initial state , Determine the drving mode """
        self.one_shot = True # unused cuurently, note application if used

        SHOWDEBUG = 0
        if SHOWDEBUG:
            print "STATE_init" , self.reason

        # ~   I. State Calcs   ~
        self.eval_scan() # Assess straight-line driving
        self.reset_time()

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

        SHOWDEBUG = 0
        if SHOWDEBUG:
            print "STATE_forward" , self.reason

        # ~   I. State Calcs   ~
        # 1. Calculate and store error
        input_center = self.eval_scan()
        rightMost  = self.lastScanNP[-1]
        self.rhgt_rolling.add( rightMost )
        right_mean = np.mean( self.rhgt_rolling )

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
        # Z. Crash Recover Override
        if self.scan_occluded():
            self.state  = self.STATE_collide_recover
            self.reason = "OCCLUSION"        

        # ~  IV. Clean / Update ~
        self.old_right_mean = right_mean
        self.prev_rghtmost  = rightMost
        # If we are exiting the state, then clear the PID
        if self.state != self.STATE_forward:
            self.clear_PID()

    def STATE_pre_turn( self ):
        """ Approaching halway end, watch for corner detector """
        self.cent_setpoint = self.turns_cent_setpoint

        SHOWDEBUG = 0
        if SHOWDEBUG:
            print "STATE_pre_turn" , self.reason

        # ~   I. State Calcs   ~
        self.preturn_timer = rospy.Time.now().to_sec() - self.preturn_start_time
        # print(self.preturn_timer)
        cent_of_maxes = self.eval_scan()
        input_center  = self.eval_scan()
        rightMost     = self.lastScanNP[-1]
        self.rhgt_rolling.add( rightMost )
        right_mean = np.mean( self.rhgt_rolling )

        # ~  II. Set controls  ~
        # self.steerAngle = self.preturn_angle
        if self.FLAG_goodScan: pass # will transition below
        else: 
            # new scheme
            translation_err  = self.update_err( input_center , self.cent_setpoint )
            self.currUp      = self.K_p_turn * translation_err
            auto_steer       = self.currUp
            self.steerAngle  = auto_steer # Control Effort
            if self.tokyo_drift:
                if self.preturn_start < self.preturn_timer and self.preturn_timer < self.preturn_stop:
                    self.linearSpeed = self.drift_speed
                else: 
                    self.linearSpeed = self.preturn_speed
            else:
                self.linearSpeed = self.preturn_speed 
            # timing var here

        # IDEA: Possible deceleration phase ??

        # ~ III. Transition Determination ~
        if self.FLAG_goodScan:
            self.state  = self.STATE_forward
            self.reason = "OVER_THRESH"
#        else:
            #if self.crnr_drop_dist < ( rightMost - right_mean ):
                #self.state = self.STATE_blind_rght_turn
                #self.reason = "CORNER_DROP"
            #else:
                #self.state = self.STATE_pre_turn
                #self.reason = "UNDER_THRESH"
        else:
            self.state  = self.STATE_pre_turn
            self.reason = "UNDER_THRESH"

        # Z. Crash Recover Override
        if self.scan_occluded():
            self.state  = self.STATE_collide_recover
            self.reason = "OCCLUSION"                
        

        # ~  IV. Clean / Update ~
        self.prev_rghtmost  = rightMost
        self.old_right_mean = right_mean
        

    def STATE_blind_rght_turn( self ):
        """ Turn right at a preset radius until a clear straightaway signal is present """

        SHOWDEBUG = 0
        if SHOWDEBUG:
            print "STATE_blind_rght_turn" , self.reason

        # ~   I. State Calcs   ~
        self.eval_scan() # Assess straight-line driving

        # ~  II. Set controls  ~
        self.linearSpeed = self.turning_speed
        self.steerAngle  = self.turning_angle

        # ~ III. Transition Determination ~
        if self.FLAG_goodScan:
            self.state  = self.STATE_forward
            self.reason = "OVER_THRESH"
        else:
            self.state  = self.STATE_blind_rght_turn
            self.reason = "UNDER_THRESH"
        # Z. Crash Recover Override
        if self.scan_occluded():
            self.state  = self.STATE_collide_recover
            self.reason = "OCCLUSION"            

        # ~  IV. Clean / Update ~
        # NONE
        
    def STATE_collide_recover( self ):
        """ Back up from a collision """
        
        # ~   I. State Calcs   ~
        if not self.FLAG_backup:
            self.FLAG_backup    = True
            self.rcovr_bgn_time = rospy.Time.now().to_sec()
        
        # ~  II. Set controls  ~
        self.linearSpeed = self.recover_speed
        # self.steerAngle  = 0.00
        self.steerAngle  = self.steer_center(self.K_p_careful, reverse = 1 )
        
        # ~ III. Transition Determination ~
        # A. If the min time has not passed, continue to recover
        nowTime = rospy.Time.now().to_sec()
        if nowTime - self.rcovr_bgn_time <= self.recover_duration:
            self.state  = self.STATE_collide_recover
            self.reason = "UNDER_MIN_TIME"        
        # B. Else the minimum time has passed
        else:
            # i. If the occlusion has been cleared, then seek open space
            if not self.scan_occluded():
                self.FLAG_backup = False
                self.eval_scan()
                # a. If there is an open hallway, GO
                if self.FLAG_goodScan:
                    self.state  = self.STATE_forward
                    self.reason = "OVER_THRESH"
                # b. Else creep forward
                else:
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
        # NONE
        
    def STATE_seek_open( self ):
        """ Slowly drive towards the most open portion of the scan """
        # ~   I. State Calcs   ~
        self.eval_scan()
        # ~  II. Set controls  ~
        self.linearSpeed = self.seek_speed
        self.steerAngle  = self.steer_center(self.K_p)    
        # ~ III. Transition Determination ~
        # a. If there is an open hallway, GO
        if self.FLAG_goodScan:
            self.state  = self.STATE_forward
            self.reason = "OVER_THRESH"
        # b. Else creep forward
        else:
            self.state  = self.STATE_seek_open
            self.reason = "UNDER_THRESH" 
   
        # ~  IV. Clean / Update ~
        # If we are exiting the state, then clear the PID
        if self.state != self.STATE_seek_open:
            self.clear_PID()

    def hallway_FSM( self ):
        """ Execute state actions and record current status """
        # NOTE: Each state must handle its own data collection, processing, control setting, and transition
        self.state() # ------ State actions and transition
        print self.state.__name__ , ',' , self.reason , ',' , self.steerAngle , ',' , self.linearSpeed
        self.report_state() # Publish state info

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

            if self.FLAG_estop:
                rospy.loginfo_throttle( 1 , "ESTOP ENGAGED" )
                self.linearSpeed = 0.0
                self.reset_time() # Reset the clock so that the car does not get stuck on resume
                self.drive_pub.publish(  compose_HARE_ctrl_msg( self.steerAngle , self.linearSpeed )  )
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
        # 2. Set the throttle
        if 1:
            self.linearSpeed = 0.1
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
