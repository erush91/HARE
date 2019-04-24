#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib, rospy
# opencv imports
import cv2

# numpy imports - basic math and matrix manipulation
import numpy as np
import math
import std_msgs.msg
import operator

# imports for ROS image handling
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# message imports specific to this package
from plan_ctrl.msg import FourierCoefsMsg
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

# Fourier Residual Method
def calc_wfi_fourier_coeffs(gamma_size, Qdot_meas):

    # Specify gamma range parameter
    gamma_range =  0.50 * math.pi
    gamma_range_half = gamma_range/2
    one_over_gamma_range_half = 1/gamma_range_half

    # 
    gamma_start = -0.25 * math.pi
    gamma_end   =  0.25 * math.pi
    
    fourier_coeff_max = 10

    # Initialize nominal gamma angle
    gamma_nominal = np.linspace(gamma_start, gamma_end, gamma_size)

    # Initialize the Fourier templates
    gamma   = np.zeros((fourier_coeff_max, gamma_size))
    c_gamma = np.zeros((fourier_coeff_max, gamma_size))
    s_gamma = np.zeros((fourier_coeff_max, gamma_size))

    # Initialize Fourier coefficients
    a_0 = np.zeros(1)
    a = np.zeros(fourier_coeff_max)
    b = np.zeros(fourier_coeff_max)

    # Compute the Fourier templates
    for i in range(fourier_coeff_max):
        for j in range(gamma_size):
            
            gamma[i,j] = i * gamma_nominal[j]
            c_gamma[i,j]= math.cos(gamma[i,j])
            s_gamma[i,j]= math.sin(gamma[i,j])

    # Compute the Fourier coefficients
    for i in range(fourier_coeff_max):
        for j in range(gamma_size):
            if i == 1:
                a_0 = a_0 + c_gamma[1,j] * Qdot_meas[j]
            a[i] = a[i] + c_gamma[i,j] * Qdot_meas[j]
            b[i] = b[i] + s_gamma[i,j] * Qdot_meas[j]

    # Calculate gamma angle differential
    dg = gamma_nominal[2] - gamma_nominal[1]

    # Scale Fourier coefficients 
    a_0 = a_0 * dg * one_over_gamma_range_half
    a   = a   * dg * one_over_gamma_range_half
    b   = b   * dg * one_over_gamma_range_half

    # Create Fourier coefficient message
    wfi_fourier_coeffs = FourierCoefsMsg()
    wfi_fourier_coeffs.header.stamp = rospy.Time.now()
    wfi_fourier_coeffs.a_0 = a_0
    wfi_fourier_coeffs.a = a
    wfi_fourier_coeffs.b = b

    return wfi_fourier_coeffs
    
# Fourier Residual Method
def calc_wfi_forward_speed_control(wfi_fourier_coeffs):

    # Forward speed gain
    K_03 = 0.1

    # Scaling factor
    N    = 10

    # Refference velocity
    v_0  = 0.5

    # Fourier coefficient --> forward velocity
    b_01 = wfi_fourier_coeffs.b[1]
    
    wfi_forward_speed_control = K_03 * (N * v_0 - b_01)

    if wfi_forward_speed_control < 0.1:
        wfi_forward_speed_control = 0.1
    if wfi_forward_speed_control > 2.0:
        wfi_forward_speed_control = 2.0

    return wfi_forward_speed_control

def calc_wfi_yaw_rate_control(wfi_fourier_coeffs):

    # Lateral position gain
    K_01 = -0.500 # K_01 < 0 for stability
    
    # Yaw angle gain
    K_02 =  0.575

    # Fourier coefficient --> lateral position
    a_01 = wfi_fourier_coeffs.a[1]

    # Fourier coefficient --> yaw angle
    a_02 = wfi_fourier_coeffs.a[2]

    wfi_yaw_rate_control = K_01 * a_01 + K_02 * a_02

    if wfi_yaw_rate_control < -2.0:
        wfi_yaw_rate_control = -2.0
    if wfi_yaw_rate_control > 2.0:
        wfi_yaw_rate_control = 2.0

    return wfi_yaw_rate_control

class WFI_Calculator:
    def __init__(self):

        # Raw Line Scan Subscriber
        self.line_scan_sub = rospy.Subscriber( "/filtered_distance" , Float32MultiArray , self.scan_callback )

        # Publish WFI control command
        self.wfi_control_command_pub = rospy.Publisher("wfi_control_command", Twist, queue_size=10)

        # Pubish WFI Fourier coefficients
        self.wfi_fourier_coeffs_pub = rospy.Publisher("wfi_fourier_coeffs", FourierCoefsMsg, queue_size=10)

        self.gamma_size = 100 #40
        
        # Initialize controller message
        self.wfi_control_command = Twist()
        self.wfi_control_command.linear.x = 0.0
        self.wfi_control_command.linear.y = 0.0
        self.wfi_control_command.linear.z = 0.0
        self.wfi_control_command.angular.x = 0.0
        self.wfi_control_command.angular.y = 0.0
        self.wfi_control_command.angular.z = 0.0

    def scan_callback( self , msg ):

        self.lastScan = msg.data

        # Calculate Fourier coefficients
        self.wfi_fourier_coeffs = calc_wfi_fourier_coeffs(self.gamma_size, self.lastScan)

        # Calculate forward velocity command
        self.wfi_forward_speed_control = calc_wfi_forward_speed_control(self.wfi_fourier_coeffs)

        # Calculate yaw rate control command
        self.wfi_yaw_rate_control = calc_wfi_yaw_rate_control(self.wfi_fourier_coeffs)

        # Create the control command
        self.wfi_control_command.linear.x  = self.wfi_forward_speed_control
        self.wfi_control_command.angular.z = self.wfi_yaw_rate_control
        self.wfi_control_command_pub.publish(self.wfi_control_command)

        # Publish Fourier coefficients
        self.wfi_fourier_coeffs_pub.publish(self.wfi_fourier_coeffs)

################################################################################

def main():
  wfi_calculator = WFI_Calculator()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

################################################################################

if __name__ == '__main__':
    rospy.init_node('wfi_controller', anonymous=True)
    main()
