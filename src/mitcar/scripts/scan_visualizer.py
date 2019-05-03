
import numpy as np
from numpy import pi
from math import sin, cos
import rospy 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from plan_ctrl.msg import CarState
import matplotlib.pyplot as plt

# ~~ Setting and Constants ~~
numReadings   = 100
RunningOdroid =   1
threshold     =   9.0
preturn_thresh = 5.0
plotCarts     =   1

# ~~ Variables ~~
lastScan = [ 0 for i in range( numReadings ) ]
ocldScan = [ 0 for i in range( numReadings ) ]
car_state = ''
up = [0]*30
ui = [0]*30
ud = [0]*30

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

def altr_cb( msg ):
    global ocldScan
    """ Process the scan that comes back from the scanner """
    # NOTE: Scan progresses from least theta to most theta: CCW
    # print "Got a scanner message with" , len( msg.intensities ) , "readings!"
    # ~ print "Scan:" , self.lastScan
    # print "Scan Min:" , min( self.lastScan ) , ", Scan Max:" , max( self.lastScan )

    if RunningOdroid: 
        ocldScan = msg.data  #lastScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]
    else: 
        ocldScan = msg.intensities 

def polr_2_cart_0Y( polarCoords ): # 0 angle is +Y North 
    """ Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ """
    return [ polarCoords[0] * sin( polarCoords[1] ) , polarCoords[0] * cos(polarCoords[1])  ]

minAng = -pi/4.0
maxAng = minAng + pi/2.0

def cart_scan( arr ):
    """ Represent the points in cartesian coordinates """
    arrLen = len( arr )
    angles = np.linspace( minAng , maxAng , arrLen )
    return [ polr_2_cart_0Y(  [ arr[i] , angles[i] ] ) for i in xrange( arrLen ) ]
    
def state_cb(state_rpt):
	global car_state, ui, up, ud
	up.pop(0)
	ui.pop(0)
	ud.pop(0)
	up.append(state_rpt.up)
	ui.append(state_rpt.ui)
	ud.append(state_rpt.ud)
	#print(round(state_rpt.up,3), round(state_rpt.ui,3), round(state_rpt.ud,3))
	car_state = state_rpt.state
	
rospy.init_node( 'scan_sherlock' , anonymous = True )
	
if RunningOdroid: 
    rospy.Subscriber( "/filtered_distance" , Float32MultiArray , scan_cb )
    rospy.Subscriber( "/alternat_distance" , Float32MultiArray , altr_cb )
    rospy.Subscriber( "/ctrl_state_report" , CarState , state_cb )
else: 
    rospy.Subscriber( "/scan" , LaserScan , scan_cb )
	
try:
	while ( not rospy.is_shutdown() ):
		lastScanNP   = np.asarray( lastScan )
		ocldScanNP   = np.asarray( ocldScan )
		above_thresh = np.where( lastScanNP > threshold )[0]

		sorted_scan_inds = lastScanNP.argsort() # sorted from smallest to largest
		N_largest_inds = sorted_scan_inds[-15:]
		largest = lastScanNP[N_largest_inds]

		plt.clf() # Clear all figures
		
		# Figure 1 , Highlighted Scan
		if plotCarts:
		    plt.figure(1)
		plt.plot( lastScanNP , 'bo' )
		plt.ylim( [ 0 , 20 ] )
		plt.hold( True )
		if len( above_thresh ) >=5:
			plt.plot( above_thresh, lastScanNP[ above_thresh ] , 'ro' )
			plt.plot(N_largest_inds, largest,'go')
		else:
			lastScanNP[-20:] = lastScanNP[-20:] + 2
			above_thresh = np.where( lastScanNP > preturn_thresh )[0]
			sorted_scan_inds = lastScanNP.argsort() # sorted from smallest to largest
			N_largest_inds = sorted_scan_inds[-15:]
			largest = lastScanNP[N_largest_inds]
			if len( above_thresh ) >=5:
				plt.plot( above_thresh, lastScanNP[ above_thresh ] , 'ro' )
				plt.plot(N_largest_inds, largest,'go')
		#elif len( above_thresh ) < 2 and np.std(lastScanNP) > 1:
			#plt.plot(N_largest_inds, largest,'go')
		plt.hold( False )
		plt.title(car_state)
		
		if plotCarts:
		    plt.figure(2)
		    points = cart_scan( lastScanNP )
		    X = [ elem[0] for elem in points ]
		    Y = [ elem[1] for elem in points ]		    
		    plt.scatter( X , Y )
		    plt.axis( 'equal' )
		    plt.title( "Regular Cart:\n" + str( car_state) )
		    
		    plt.figure(3)
		    points = cart_scan( ocldScanNP )
		    X = [ elem[0] for elem in points ]
		    Y = [ elem[1] for elem in points ]		    
		    plt.scatter( X , Y )
		    plt.axis( 'equal' )
		    plt.title( "Occlusion Cart:\n" + str( car_state ) )
		    
		if plotCarts: 
		    plt.figure(4)
		else: 
		    plt.figure(2)
		plt.plot(up,'k')
		plt.hold(True)
		plt.plot(ui,'g')
		plt.plot(ud,'r')
		plt.ylim([-0.5, 0.5])
		plt.xlim([0,25])
		plt.legend(['P','I','D'])
		plt.title('Control inputs')
		
		plt.pause( 0.001 )
	plt.show()
except KeyboardInterrupt:
	pass

