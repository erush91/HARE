/*  joy_ctrl.cpp
    James Watson , 2019 March , Adapted from https://github.com/mit-racecar/
    Break out joystick conrol from the racecar
*/
    

#include <ros/ros.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

//~ #include "racecar_simulator/CtrlEffort.h"

using namespace racecar_simulator;

class JoystickRelay{

private:

    // A ROS node
    ros::NodeHandle nh;

    // Joystick parameters
    int joy_speed_axis , joy_angle_axis;
    double joy_max_speed;
    long int seqNum = 0;
    
    // Relevant car params
    double max_speed , max_steering_angle;
    
    // Topic Names
    std::string joy_topic , drive_topic;
    
    // Listen for drive and joystick commands
    ros::Subscriber joy_sub;
    
    // Publish drive commands
    ros::Publisher drive_pub;
    
    bool SHOWDEBUG = false;
    
public:

    JoystickRelay(){
        
        // Initialize the node handle
        nh = ros::NodeHandle("joy_ctrl");
        
        // Get the topic names        
        if(  !nh.getParam( "/racecar_simulator/joy_topic"   , joy_topic   )  ){
            ROS_ERROR( "FAILED TO RETRIEVE THE JOYSTICK TOPIC!" );
        }
        if(  !nh.getParam( "/racecar_simulator/drive_topic" , drive_topic )  ){
            ROS_ERROR( "FAILED TO RETRIEVE THE DRIVE TOPIC!" );
        }
        
        // Get joystick parameters
        bool joy;
        nh.getParam( "/racecar_simulator/joy" /* ---- */ , joy            );
        nh.getParam( "/racecar_simulator/joy_speed_axis" , joy_speed_axis );
        nh.getParam( "/racecar_simulator/joy_angle_axis" , joy_angle_axis );
        nh.getParam( "/racecar_simulator/joy_max_speed"  , joy_max_speed  );
        
        // Get relevant car params
        nh.getParam( "/racecar_simulator/max_speed" /* ---------- */ , max_speed );
        nh.getParam( "/racecar_simulator/max_steering_angle" /* - */ , max_steering_angle );
        
        // If the joystick is enabled
        if( joy ){
            // Start a subscriber to listen to joystick commands
            joy_sub = nh.subscribe( joy_topic , 1 , &JoystickRelay::joy_callback , this );
        }
        
        // Advertise the control effort publisher
        drive_pub = nh.advertise< ackermann_msgs::AckermannDriveStamped >( drive_topic , 1 );
        
        if( SHOWDEBUG ){
            std::cerr << "Found drive topic: " << drive_topic << std::endl;
            std::cerr << "Found joy topic: _ " << joy_topic   << std::endl;
        }
    }
    
    void joy_callback( const sensor_msgs::Joy& msg ){
        // Respond to a joystick message
        seqNum++; // Increment sequence
        
        if( SHOWDEBUG )  std::cerr << "Joy message received on " << joy_topic << std::endl;
        
        // Create a message
        ackermann_msgs::AckermannDriveStamped sendMsg{};
        // Compose header
        sendMsg.header.seq      = seqNum;
        sendMsg.header.stamp    = ros::Time::now();
        sendMsg.header.frame_id = joy_topic;
        // Compose control effort
        sendMsg.drive.steering_angle = max_steering_angle * msg.axes[ joy_angle_axis ];
        sendMsg.drive.speed /* -- */ = joy_max_speed * msg.axes[ joy_speed_axis ];
        
        // Send message
        drive_pub.publish( sendMsg );
    }
};

int main( int argc , char ** argv ){
    ros::init( argc , argv , "joy_ctrl" );
    JoystickRelay rs;
    ros::spin();
    return 0;
}
