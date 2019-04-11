/*
ROS_NODE_TEMPLATE.cpp
James Watson , YYYY MONTHNAME

% ROS Node %
A ONE-LINE DESCRIPTION OF THE NODE
Publises To ----> :
Subscribes To <-- :

Dependencies: ROS , Cpp_Helpers , ROS_Helpers
Template Version: 2018-06-25
*/

// === Imports =============================================================================================================================

// ~~~ Includes ~~~
// ~~ ROS ~~
#include <ros/ros.h> // --- ROS , Publishers , Subscribers
#include <ros/package.h> // Where are we? // http://wiki.ros.org/Packages#C.2B-.2B-
// ~ ROS Messages ~
// #include "PACKAGE_NAME/MSG_TYPE.h"
// #include "PACKAGE_NAME/SERVICE_TYPE.h"
// ~~ Local ~~
#include <Cpp_Helpers.h> // C++ Utilities and Shortcuts
#include <ROS_Helpers.h> // ROS Utilities and Shortcuts

// ___ End Import __________________________________________________________________________________________________________________________


/// === INIT ===============================================================================================================================

/// ######### Node Vars #########

string NODE_NAME = "NODE_NAME";
int    RATE_HZ   = 100;
int    QUEUE_LEN = 100;

/// ********* End Vars **********


// === Program Functions & Classes ===


// ___ End Functions & Classes ___


// === Program Vars ===


// ___ End Vars ___

/// ___ END INIT ___________________________________________________________________________________________________________________________


// === main ================================================================================================================================

int main( int argc , char** argv ){ // Main takes the terminal command and flags that called it as arguments
	srand( time( 0 ) ); // Random seed based on the current clock time
	
	/// === Preliminary { Setup , Instantiation , Planning  } ==============================================================================



	/// ___ End Preliminary ________________________________________________________________________________________________________________
	
	// 0. Init ROS  &&  Register node
	ros::init( argc , argv , NODE_NAME );
	
	// 1. Fetch handle to this node
	ros::NodeHandle nodeHandle;
	
	// 2. Init node rate
	ros::Rate heartbeat( RATE_HZ );
	
	// 3. Set up subscribers and publishers
	
	// ~ Publishers ~
	// ros::Publisher PUBLISHER_OBJ = nodeHandle.advertise<CATEGORY_MSGS::MSG_TYPE>( "TOPIC_NAME" , QUEUE_LEN );
	
	// ~ Subscribers ~
	// ros::Subscriber SUBSCRIBER_OBJ = nodeHandle.subscribe( "TOPIC_NAME" , QUEUE_LEN , CALLBACK_FUNCTION );
	
	// ~ Service Servers ~
	// ros::ServiceServer SERVER_OBJ = nodeHandle.advertiseService( "SERVICE_NAME" , SERV_CALLBACK );
	
	// ~ Service Clients ~
	// ros::ServiceClient CLIENT_OBJ = nodeHandle.serviceClient<CATEGORY::SERVICE_TYPE>( "SERVICE_NAME" );
	
	
	// N-2. Animation Init 
	//~ srand( ( time( 0 ) % 1 ) * 1000.0 + getpid() ); // Random seed based on the current clock time
	//~ RViz_MarkerManager mrkrMngr{};
	//~ visualization_msgs::MarkerArray& markerArr = mrkrMngr.get_arr();
	
	/// == PRE-LOOP WORK ===================================================================================================================
		
		
		
	/// __ END PRE-LOOP ____________________________________________________________________________________________________________________
	
	// N-1. Notify
	ros_log( "[" + NODE_NAME + "] Init OK and about to run ..." , INFO );
	
	// N. Main loop
	while( ros::ok() ){ // While neither node nor ROS has been shut down
		
		/// == LOOP WORK ===================================================================================================================
		
		break; // NOT EVEN ONCE
		
		/// __ END LOOP ____________________________________________________________________________________________________________________
		
		ros::spinOnce(); // - Process messages
		heartbeat.sleep(); // Sleep for remainder of period
	}
	
	// N+1. Notify  &&  Exit
	
	ros_log( "[" + NODE_NAME + "] Exit OK, Goodbye!" , INFO );
	
	return 0; // I guess everything turned out alright at the end!
}

// ___ End main ____________________________________________________________________________________________________________________________


/* === Spare Parts =========================================================================================================================



   ___ End Spare ___________________________________________________________________________________________________________________________
*/
