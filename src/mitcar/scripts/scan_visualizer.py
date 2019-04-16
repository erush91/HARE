
import numpy as np
from numpy import pi
import rospy 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

# ~~ Setting and Constants ~~
numReadings   = 100
RunningOdroid =   1
threshold     =   9.0
plotCarts     =   1

# ~~ Variables ~~
lastScan = [ 0 for i in range( numReadings ) ]

def scan_cb( msg ):
    global lastScan
    """ Process the scan that comes back from the scanner """
    # NOTE: Scan progresses from least theta to most theta: CCW
    # print "Got a scanner message with" , len( msg.intensities ) , "readings!"
    # ~ print "Scan:" , self.lastScan
    # print "Scan Min:" , min( self.lastScan ) , ", Scan Max:" , max( self.lastScan )

    if RunningOdroid: 
        lastScan = msg.data  #lastScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]
    else: 
        lastScan = msg.intensities 

def polr_2_cart_0Y( polarCoords ): # 0 angle is +Y North 
    """ Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ """
    return [ polarCoords[0] * sin( polarCoords[1] ) , polarCoords[0] * cos(polarCoords[1])  ]

minAng = -pi/4.0
maxAng = minAng + pi/2.0

def cart_scan( arr ):
    """ Represent the points in cartesian coordinates """
    arrLen = len( arr )
    angles = linspace( minAng , maxAng , arrLen )
    return [ polr_2_cart_0Y(  [ arr[i] , angles[i] ] ) for i in xrange( arrLen ) ]
	
rospy.init_node( 'scan_sherlock' , anonymous = True )
	
if RunningOdroid: 
    rospy.Subscriber( "/filtered_distance" , Float32MultiArray , scan_cb )
else: 
    rospy.Subscriber( "/scan" , LaserScan , scan_cb )
	
try:
	while ( not rospy.is_shutdown() ):
		lastScanNP   = np.asarray( lastScan )
		above_thresh = np.where( lastScanNP > threshold )[0]
		# print(above_thresh)
		# print(lastScan)
		plt.clf() # Clear all figures
		
		# Figure 1 , Highlighted Scan
		plt.figure(1)
		plt.plot( lastScanNP , 'bo' )
		plt.hold( True )
		vis_maxes = np.zeros( ( len( lastScan ) ) )
		if len( above_thresh ) >=2:
			vis_maxes[ above_thresh ] = lastScanNP[ above_thresh ]
			plt.plot( vis_maxes , 'ro' )
			# print(np.mean(above_thresh))
		plt.hold( False )
		
		if plotCarts:
		    plt.figure(2)
		    points = cart_scan( lastScanNP )
		    X = [ elem[0] for elem in points ]
		    Y = [ elem[1] for elem in points ]		    
		    plt.scatter( X , Y )
		    plt.axis( 'equal' )
		
		plt.pause( 0.001 )
	plt.show()
except KeyboardInterrupt:
	pass

