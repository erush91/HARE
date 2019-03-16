#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from ros_pololu_servo.msg import HARECommand

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to HARE_high_level_command!
---------------------------
'b' : Brake - Stops vehicle 
'g' : Go - Starts vehicle motion
'f' : Faster - Increase speed by 10%
's' : Slower - Descrease speed by 10%
'r' : Right Turn - Turn right by .05 radians
'l' : Left Turn - Turn left by .05 radians
'c' : Center - Put the steering back to straight
"""

moveBindings = {
        'b':(0,0,0,0),
        'g':(1,0,0,0),
        'c':(1,1,0,0),
        }

speedBindings={
        'f':(1,0),
        's':(-1,0),
        'l':(0,-1),
        'r':(0, 1)
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('HARE_high_level_command', HARECommand, queue_size = 1)
    rospy.init_node('HARE_teleop')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    mode = 0
    speed = 0
    turn = 0
    straight = 0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                mode = moveBindings[key][0] 
                straight = moveBindings[key][1] 
                speed = speed*mode
            elif key in speedBindings.keys():
                if mode == 0:
                    speed = 0
                speed = speed + .1*speedBindings[key][0]
                turn = turn + .05*speedBindings[key][1]
            else:
                if (key == '\x03'):
                    break
            
            command_out = HARECommand()
            if straight == 1:
                command_out.steering_angle = 0
            else:
                command_out.steering_angle = turn

            command_out.throttle_cmd = mode*speed
            command_out.throttle_mode = mode
            pub.publish(command_out)

    except Exception as e:
        print(e)

    finally:
        command_out = HARECommand()
        command_out.steering_angle = 0.0
        command_out.throttle_cmd = 0.0
        pub.publish(command_out)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
