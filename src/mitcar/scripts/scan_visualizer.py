
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
try:
	while ( not rospy.is_shutdown() ):
		lastScanNP = np.asarray(lastScan)
		above_thresh = np.where(lastScanNP > 9)[0]
		# print(above_thresh)
		# print(lastScan)
		plt.clf()
		plt.plot(lastScanNP,'bo')
		plt.hold(True)
		vis_maxes = np.zeros((len(lastScan)))
		if len(above_thresh) >=2:
			vis_maxes[above_thresh] = lastScanNP[above_thresh]
			plt.plot(vis_maxes,'ro')
			print(np.mean(above_thresh))
		plt.pause(0.001)
	plt.show()
except KeyboardInterrupt:
	pass

# plt.close

