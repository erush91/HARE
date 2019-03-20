#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "simple_PID_on_wall.py"
__version__  = "2019.03" 
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
            self.heartBeatHz = int( rospy.get_param( "/publ_rate" ) )
        except:
            self.heartBeatHz = 20 # Node refresh rate [Hz]
            rospy.logwarn( "CarFSM: Unable to retrieve update rate! , Setting to " + str( self.heartBeatHz ) + "Hz ..." )
            
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        
        # 3. Start subscribers and listeners
        rospy.Subscriber( "/filtered_distance" , Float32MultiArray , self.scan_cb )
        self.numReadings = 100
        self.lastScan = [ 0 for i in range( self.numReadings ) ]
        
        
        # 4. Start publishers
        try:
            self.driveTopic = str( rospy.get_param( "/drivtopic" ) )
        except:
            self.driveTopic = "HARE_high_level_command"
            rospy.logwarn( "CarFSM: Unable to retrieve control topic! , Setting to " + self.driveTopic )
            
        self.drive_pub = rospy.Publisher( 'HARE_high_level_command' , HARECommand , queue_size = 10 )
        
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()       
        
        # 6. Driving Vars
        self.err_hist_window = 25 # Width of the integration window
        self.err_hist        = [ 0 for i in range( self.err_hist_window ) ] # Integration window
        self.wallSetPnt      =  1.0 # [m]
        self.nearN           = 30 # Count this many points as near the average
        self.slope_window    =  5 # Look this many points in the past to compute slope
        # ~ PID ~
        self.K_d = rospy.get_param("D_VALUE")
        self.K_i = rospy.get_param("I_VALUE")
        self.K_p = rospy.get_param("P_VALUE")
        # ~ Control Output ~
        self.steerAngle  = 0.0
        self.prevSteerAngle = 0.0
        self.angleDiffMin = rospy.get_param("ANGLE_DIFF_MIN") # limits micro commands
        self.linearSpeed = rospy.get_param("LINEAR_SPEED") # [ -1 ,  1 ]
        
        # 7. Test vars
        self.t_test = 0.0
        self.t_incr = 0.025
        self.t_curr = 0.0
        self.t_last = rospy.Time.now().to_sec()
        self.Tstate = 0
        
    def near_avg( self ):
        """ Average of the nearest points """
        return np.mean( self.lastScan[ :self.nearN ] )
        
    def scan_cb( self , msg ):
        """ Process the scan that comes back from the scanner """
        # NOTE: Scan progresses from least theta to most theta: CCW
        # print "Got a scanner message with" , len( msg.intensities ) , "readings!"
        # ~ print "Scan:" , self.lastScan
        # print "Scan Min:" , min( self.lastScan ) , ", Scan Max:" , max( self.lastScan )
        if 0: # DIRTY HACK
            self.lastScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]
        else:
            self.lastScan = msg.data
        
    def test_state( self ):
        """ Run the drive motor forwards and backwards while performing a sine sweep on the steering angle """
        # 1. Set the steering angle
        self.t_curr += self.t_incr
        self.steerAngle = pi/4 * sin( self.t_curr )
        # 2. Set the throttle
        if 1:
            self.linearSpeed = 0.0
        else:
            self.t_curr = rospy.Time.now().to_sec()
            # A. If more than 1 second has passed since the last transition, advance to the next speed state
            if self.t_curr - self.t_last > 1.0:
                self.Tstate = ( self.Tstate + 1 ) % len( _STATETRANS )
                self.linearSpeed = _STATETRANS[ self.Tstate ]
                self.t_last = self.t_curr

    def center_stop_state( self ):
        """ Set the steering and speed to 0 """
        self.steerAngle  = 0.0
        self.linearSpeed = 0.0
        
    def wall_follow_state( self ):
        """ Try to maintain a set distance from the wall """
        # 1. Calculate and store error
        translation_err = ( self.wallSetPnt - np.mean( self.lastScan ) )
        # ~ translation_err = ( self.wallSetPnt - self.near_avg() )
        self.err_hist.append( translation_err )
        
        if len( self.err_hist ) >= self.err_hist_window:
            self.err_hist.pop(0)
            u_i = self.K_i * sum( self.err_hist )
        else:
            u_i = 0

        if self.linearSpeed > 0.1: 
            u_d = self.K_d * ( self.err_hist[-1] - self.err_hist[ -(self.slope_window-1) ] ) 
        else: 
            u_d = 0
        
        u_p = self.K_p * translation_err

        auto_steer = u_p + u_i + u_d

        # check to see if the new steering angle is large enough to warrant a command
        # this prevents micro commands to the servo
        if np.abs(auto_steer - self.prevSteerAngle) > self.angleDiffMin:
            self.prevSteerAngle = self.steerAngle
            self.steerAngle = auto_steer

            #print 'mean:' , np.mean( self.lastScan ) ,' translation error:' , translation_err , 'steer:' , self.steerAngle
            return True
        else:
            return False
        
        
    def run( self ):
        """ Take a distance reading and generate a control signal """
        
        # 1. While ROS is running
        while ( not rospy.is_shutdown() ):
            
            # 1. Drive Test
            if 0:
                self.drive_pub.publish(  compose_ack_ctrl_msg( pi/8 , 2.0 )  )
                
            # 2. Generate a control effort
            sendCommand = True
            if 1:
                sendCommand = self.wall_follow_state()
            elif 0:
                self.test_state()
            else:
                self.center_stop_state()
            
            # 3. Transmit the control effort
            if sendCommand:
                #print( "Steering Angle:" , self.steerAngle , ", Speed:" , self.linearSpeed )
                self.drive_pub.publish(  compose_HARE_ctrl_msg( self.steerAngle , self.linearSpeed )  )
            
            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep()        
        
        # N. Post-shutdown activities
        else:
            print "Node Shutdown after" , rospy.Time.now().to_sec() - self.initTime , "seconds"
        

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

