#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "drive_FSM.py"
__version__  = "2019.03" 
__desc__     = "A Finite State Machine for autonomous car motion planning"
"""
James Watson , Template Version: 2019-02-21
Built on Wing 101 IDE for Python 2.7

Dependencies: numpy , rospy
"""


"""  
~~~ Developmnent Plan ~~~
[Y] Drive forward && Turn
[ ] Re-implement the Pygame demo
[ ] Test if the pygame controller will do corners on its own
[ ] If not, implement a dead simple turn
[ ] Panic Mode?
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
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan # http://www.theconstructsim.com/read-laserscan-data/
# ~~ Local ~~

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

def eq_margin( op1 , op2 , margin = EPSILON ): 
    """ Return true if op1 and op2 are within 'margin' of each other, where 'margin' is a positive real number """
    return abs( op1 - op2 ) <= margin

# __ End Func __


# == Program Classes ==

class CarFSM:
    """ A Finite State Machine for autonomous car motion planning """
    
    def __init__( self ):
        """ Start car """
        # 1. Start the node
        rospy.init_node( 'CarFSM' ) 
        # 2. Set rate
        try: 
            # Attempt to set a refresh rate commensurate with the lidar update rate
            self.heartBeatHz = int( 1.0 / rospy.get_param( "/racecar_simulator/update_pose_rate" ) )
        except:
            self.heartBeatHz = 300 # ------------------- Node refresh rate [Hz]
            rospy.logwarn( "CarFSM: Unable to retrieve update rate! , Setting to " + str( self.heartBeatHz ) + "Hz ..." )
            
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        
        # 3. Start subscribers and listeners
        rospy.Subscriber( "/scan" , LaserScan , self.scan_cb )
        self.numReadings = 100
	self.scanCenter  = int(self.numReadings//2)
        self.lastScan = [ 0.0 for i in range( self.numReadings ) ]
	self.lastScanNP = np.asarray( self.lastScan )
        self.num_right_scans = 5
        self.old_right_side_scans = np.ones((self.num_right_scans))*20
        
        self.good_signal = True
        self.one_shot = True
        self.close_eyes_and_go_straight_state = False
        self.close_eyes_and_turn_right_state = False
        
        
        # 4. Start publishers
        try:
            self.driveTopic = str( rospy.get_param( "/racecar_simulator/drive_topic" ) )
        except:
            self.driveTopic = "/drive"
            rospy.logwarn( "CarFSM: Unable to retrieve control topic! , Setting to " + self.driveTopic )
            
        self.drive_pub = rospy.Publisher( self.driveTopic , AckermannDriveStamped , queue_size = 10 )
        
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec() # Time that the node was started      
        self.lastTime = self.initTime # ----------- Time that the last loop began
        
        # 6. Driving Vars
        self.err_hist_window = 25 # Width of the integration window
        self.err_hist        = [ 0 for i in range( self.err_hist_window ) ] # Integration window
        self.tim_hist        = [ 0 for i in range( self.err_hist_window ) ] # Integration window
        self.wallSetPnt      =  3.0 # [m]
        self.nearN           = 30 # Count this many points as near the average
        self.slope_window    = 10 # Look this many points in the past to compute slope
	
	# 7. FSM Vars
	self.set_FSM_vars()
	
        # ~ PID ~
        self.K_d = 0.0080
        self.K_i = 0.00000
        self.K_p = 0.8000
        self.FSM_K_p = 0.005
        
        # ~ Control Output ~
        self.steerAngle  = 0.0
        self.linearSpeed = 4.0
        
    def scan_cb( self , msg ):
        """ Process the scan that comes back from the scanner """
        # NOTE: Scan progresses from least theta to most theta: CCW
        # print "Got a scanner message with" , len( msg.intensities ) , "readings!"
        # ~ print "Scan:" , self.lastScan
        # print "Scan Min:" , min( self.lastScan ) , ", Scan Max:" , max( self.lastScan )
        self.lastScan = msg.intensities # Do I need to copy this?
        
        
    # === DRIVE FINITE STATE MACHINE =======================================================================================================
        
    def set_FSM_vars( self ):
	""" Set the variables necessary for the FSM controller """
	# 7. FSM Vars
	self.state           = self.STATE_init # -- Currently-active state, the actual function
	self.max_thresh_dist =  9.0 # ------------- Above this value we consider distance to be maxed out
	self.thresh_count    = 10 # --------------- If there are at least this many readings above 'self.max_thresh_dist' 
	self.straight_speed  =  4.0 # ------------- Speed for 'STATE_forward'	
	self.FLAG_goodScan   = False # ------------ Was the last scan appropriate for straight-line driving
        
    def eval_scan( self ):
	""" Populate the threshold array and return its center """
	self.lastScanNP = np.asarray( self.lastScan )
	self.above_thresh = np.where( lastScanNP > self.max_thresh_dist )[0]
	return np.mean( self.above_thresh )
        
    def p_good_scan( self ):
	""" Predicate: Was the latest scan a good scan? """
	if len( above_thresh ) >= self.thresh_count:
	    return True
	else: 
	    return False
    
    """
    STATE_funcname
    # ~   I. State Calcs   ~
    # ~  II. Set controls  ~
    # ~ III. Transition Determination ~
    # ~  IV. Clean / Update ~
    """
    
    def STATE_init( self ):
	""" Initial state , Determine the drving mode """
	
	# ~   I. State Calcs   ~
	self.FLAG_goodScan = self.p_good_scan() # Assess straight-line driving
	
	# ~  II. Set controls  ~
	self.steerAngle  = 0.0
	self.linearSpeed = 0.0	
	
	# ~ III. Transition Determination ~	
	# NOTE: At the moment, safest transition is probably to wait for the first straightaway
	# A. If there is a good straightaway scan, then set the forward state
	if self.FLAG_goodScan:
	    self.state = self.STATE_forward
	# B. Otherwise, wait for a good hallway scan (This way we can troubleshoot just by wathcing wheel motion)
	else:
	    self.state = self.STATE_init
	    
	# ~  IV. Clean / Update ~
	# NONE
    
    def STATE_forward( self ):
	""" Straightaway driving , Monitor for turn or fault """
	
	# ~   I. State Calcs   ~
	# 1. Calculate and store error
	right_side_scans = np.mean( lastScanNP[ 0:self.num_right_scans ] )	
	
	# ~  II. Set controls  ~
	cent_of_maxes   = self.eval_scan()
	translation_err = cent_of_maxes * 1.0 - self.scanCenter
	u_p             = self.FSM_K_p * translation_err	
	auto_steer      = u_p # + u_i + u_d # NOTE: P-ctrl only for demo
	self.steerAngle = auto_steer	
	
	# ~ III. Transition Determination ~
	if lastScanNP[0] - self.old_right_side_scans > 0.3:
	    self.state = self.STATE_blind_rght_turn
	else:
	    self.state = self.STATE_forward
	
	# ~  IV. Clean / Update ~
	self.old_right_side_scans = right_side_scans
       
    def STATE_blind_rght_turn( self ):
	"""  """
	# ~   I. State Calcs   ~
	# ~  II. Set controls  ~
	# ~ III. Transition Determination ~
	# ~  IV. Clean / Update ~	
        
    def hallway_FSM( self ):
        """ State 1 - Steer at the center of the patch of max distances 
        
        transition, not enough scan entries greater than a set threshold that is near the camera max
        
        State 2 - Drive straight (maybe add some deceleration here), note this only works if our control coming
        into this state is relatively straight and stable
        
        transition, looking at the right side of the scan data, a large change indicates beam passed off the
        corner so the turn should begin
        
        State 3 - Drive with a set right hand turn angle 
        
        transition, camera maxes are once again found and State 1 is reentered
        """
        
        
		# this sets the state transition flag based on "good signal"
        
            
            # one shot state transition the first time we lose a good hall following signal
            if not self.close_eyes_and_go_straight_state and not self.close_eyes_and_turn_right_state:
				self.close_eyes_and_go_straight_state = True 
				
            elif self.close_eyes_and_go_straight_state:
                
					self.close_eyes_and_go_straight_state = False
					self.close_eyes_and_turn_right_state = True
            
            cent_of_maxes = 50
            
        
            
        

        if self.good_signal:
            
            print('In State 1 (hall follow) ','translation error:' , translation_err , 'steer:' , round(self.steerAngle,5))
        else: 
	    if self.close_eyes_and_go_straight_state:
		    self.steerAngle = self.steerAngle # do nothing
		    print('In State 2 (go straight till turn) ',round(lastScanNP[0] - self.old_right_side_scans,5))
	    if self.close_eyes_and_turn_right_state:
		    self.steerAngle = -0.1
		    print('In State 3 (turn till good hall signal found) ')
        
    # ___ END FSM __________________________________________________________________________________________________________________________
        
    def run( self ):
        """ Take a laser reading and generate a control signal """
        
        # 1. While ROS is running
        while ( not rospy.is_shutdown() ):
            
            # 1. Drive Test
            if 0:
                self.drive_pub.publish(  compose_ack_ctrl_msg( pi/8 , 2.0 )  )
                
            # 2. Generate a control effort -- should probably have a method for switching between control methods that's better than commenting
            # self.wall_follow_state() 
            self.hallway_FSM()
            
            # 3. Transmit the control effort
            if 1:
                self.drive_pub.publish(  compose_ack_ctrl_msg( self.steerAngle , self.linearSpeed )  )
            
            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep()        
        
        # N. Post-shutdown activities
        else:
            print "Node Shutdown after" , rospy.Time.now().to_sec() - self.initTime , "seconds"

    

    # === RIGHT-FACING CONTROLLER ==========================================================================================================
	    
    def near_avg( self ):
        """ Average of the nearest points """
        return np.mean( self.lastScan[ :self.nearN ] )
    
    def avg_nonzero( self ):
        """ Return the average of the nonzero elements only """
        numNonZ = 0.0 # Number of items that are nonzero
        totNonZ = 0.0 # Sum of items that are nonzero
        for elem in self.lastScan:
            if not eq_margin( elem , 0.0 ):
                numNonZ += 1.0
                totNonZ += elem
        if totNonZ > 0:
            return totNonZ / numNonZ
        else:
            return 0.0    
        
    def integrate_err( self ):
        """ Rectangular integration of error over time """
        curveArea = 0.0
        for i in xrange( self.err_hist_window ):
            curveArea += self.err_hist[i] * self.tim_hist[i]
        return curveArea
    
    def rate_err( self ):
        """ Average time rate of change in error over a window """
        rateTot = 0.0
        for i in xrange( self.slope_window ):
            change     = self.err_hist[ -(i) ] - self.err_hist[ -(i+1) ]
            if self.tim_hist[ -(i) ] > 0:
                rateChange = change / self.tim_hist[ -(i) ]
            else:
                rateChange = 0.0
            rateTot   += rateChange
        return rateTot / self.slope_window

    def wall_follow_state( self ):
        """ Try to maintain a set distance from the wall """
        
        # 0. record loop duration
        nowTime = rospy.Time.now().to_sec() # -- Get the current time
        loopDuration = nowTime - self.lastTime # Time delta between the start of this loop and the start of the last loop
        self.tim_hist.append( loopDuration )
        if len( self.tim_hist) >= self.err_hist_window:
            self.tim_hist.pop(0) 
        self.lastTime = nowTime
        
        # 1. Calculate and store error
        if 0:
            translation_err = ( self.wallSetPnt - np.mean( self.lastScan ) )
        elif 1:
            translation_err = ( self.wallSetPnt - self.avg_nonzero() )
        else:
            translation_err = ( self.wallSetPnt - self.near_avg() )
        self.err_hist.append( translation_err )
        
        if len( self.err_hist ) >= self.err_hist_window:
            self.err_hist.pop(0)
            u_i = self.K_i * self.integrate_err()
        else:
            u_i = 0

        if self.linearSpeed > 0.1: 
            u_d = self.K_d * self.rate_err( )
        else: 
            u_d = 0
        
        u_p = self.K_p * translation_err

        auto_steer = u_p + u_i + u_d
        self.steerAngle = auto_steer

        print 'translation error:' , translation_err , 'steer:' , self.steerAngle
        # ~ robot.move(steering_angle, speed)    

# __ End Class __


# == Program Vars ==



# __ End Vars __


if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:] # Terminal arguments , if they exist
    
    FOO = CarFSM()
    FOO.run()    
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________

