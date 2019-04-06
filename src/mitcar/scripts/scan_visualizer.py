
import numpy as np
import rospy 
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

numReadings = 100
lastScan = [ 0 for i in range( numReadings ) ]

def scan_cb( msg ):
	global lastScan
	""" Process the scan that comes back from the scanner """
	# NOTE: Scan progresses from least theta to most theta: CCW
	# print "Got a scanner message with" , len( msg.intensities ) , "readings!"
	# ~ print "Scan:" , self.lastScan
	# print "Scan Min:" , min( self.lastScan ) , ", Scan Max:" , max( self.lastScan )
	lastScan = msg.intensities # Do I need to copy this?
	
rospy.init_node('scan_sherlock', anonymous=True)
	
rospy.Subscriber( "/scan" , LaserScan , scan_cb )
	
# plt.ion()
while ( not rospy.is_shutdown() ):
	plt.clf()
	plt.plot(lastScan)
	plt.pause(0.001)

plt.show()
