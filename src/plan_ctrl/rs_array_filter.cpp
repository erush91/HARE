/*
rs_array_filter.cpp
James Watson , 2019 March

% ROS Node %
Processes depth image matrices and returs an array in the form expected by the the 'mitcar' sim
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
// Subscribe
#include <image_transport/image_transport.h>
// Publish
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
// ~~ Local ~~
#include <Cpp_Helpers.h> // C++ Utilities and Shortcuts
#include <ROS_Helpers.h> // ROS Utilities and Shortcuts

// ___ End Import __________________________________________________________________________________________________________________________


/// === INIT ===============================================================================================================================

/// ######### Node Vars #########

string NODE_NAME = "cam_arr_filter";
int    RATE_HZ   = 100;
int    QUEUE_LEN = 100;

/// ********* End Vars **********


// === Program Vars ===
int /* --------- */ numPts; // ----- Number of points in the output array 
int /* --------- */ numLin; // ----- Number of rows to process
std::vector<int>    scanRows; // --- Indices of matrix rows to process
string /* ------ */ imageTopic; // - Topic that depth images will be streamed to 
std::vector<double> distance_arr; // Array to store filtered data
std::vector<double> sums; // ------- 
std::vector<double> blockLen; // --- Number of elements needed for each sampled point
std::vector<uint>   sampleBounds; // Indiced of the buckets to sample from
int /* --------- */ imageWidth; // - Width of the depth image
int /* --------- */ imageHeight; //- Height of the depth image
// ~ Defaults ~
std::vector<int>  dfltRows /*- */ = { 310 , 260 , 210 }; // ---------- Default rows to sample
string /* ---- */ defaultCamTopic = "/camera/depth/image_rect_raw"; // Default topic for depth image
// ___ End Vars ___


// === Program Functions & Classes ===

std::vector<uint> even_index_bounds( uint total , uint divisions ){
    // Return the ending indices of that will split 'total' into roughly-equal 'divisions'
    std::vector<uint> rtnBounds;
    double stepSize = total * 1.0 / divisions;
    double runTot = stepSize;
    while( runTot < total ){
        rtnBounds.push_back(  (uint) round( runTot )  );
        runTot += stepSize;
    }
    if( rtnBounds.size() < divisions ){  rtnBounds.push_back( total );  }
    return rtnBounds;
}

std::vector<uint> elem_count_bounds( const std::vector<uint>& bucketBounds ){
    // Return the number of elements in each bucket
    std::vector<uint> nums = { bucketBounds[0] };
    uint numBuckts = bucketBounds.size();
    for( uint i = 1 ; i < numBuckts ; i++ ){  nums.push_back( bucketBounds[i] - bucketBounds[i-1] );  }
    return nums;
}

void image_filter_cb( const sensor_msgs::Image& msg ){
    // Filter and store the depth data in a format that the controller will understand
    uint curRow     = 0 , 
         currSample = 0 ;
    // 0. Zero out sums
    vec_assign_all_same( sums , 0.0 );
    
    // 1. For every sampled row
    for( uint i = 0 ; i < numLin ; i++ ){
        // 2. Assign row  &&  Reset sample
        curRow = scanRows[i];
        currSample = 0;
        // 3. For every column index in the sample
        for( uint j = 0 ; j < imageWidth ; j++ ){  
            // 4. If we have reached the bounds of the last sample, then increment sample
            if( j >= sampleBounds[ currSample ] ){  currSample++;  }
            // 5. Accumulate the distance measurement at this pixel
            sums[ currSample ] += msg.data[ rowmajor_flat_index( imageWidth , curRow , j ) ];  
        }
    }
    
    // 6. Calculate averages
    for( int i = 0 ; i < numPts ; i++ ){  distance_arr[i] = sums[i] / blockLen[i];  }
}

// ___ End Functions & Classes ___

/// ___ END INIT ___________________________________________________________________________________________________________________________


// === main ================================================================================================================================

int main( int argc , char** argv ){ // Main takes the terminal command and flags that called it as arguments
	srand( time( 0 ) ); // Random seed based on the current clock time
	
	/// === Preliminary { Setup , Instantiation , Planning  } ==============================================================================

    // NONE?

	/// ___ End Preliminary ________________________________________________________________________________________________________________
	
	// 0. Init ROS  &&  Register node
	ros::init( argc , argv , NODE_NAME );
	
	// 1. Fetch handle to this node
	ros::NodeHandle nodeHandle;
    
    // 1.5. Fetch params
    // A. Get the refresh rate
    assign_param_or_default( nodeHandle , "/publ_rate" , RATE_HZ     ,  100 );
    // B. Get the number of output points
    assign_param_or_default( nodeHandle , "/numpoints" , numPts      ,  100 );
    // C. Get the number of rows to process
    assign_param_or_default( nodeHandle , "/scanlines" , numLin      ,    3 );
    // D. Get indices of rows to process
    assign_param_or_default( nodeHandle , "/scan_rows" , scanRows    , dfltRows );
    // E. Get the source of camera data imageTopic
    assign_param_or_default( nodeHandle , "/imagtopic" , imageTopic  , defaultCamTopic );
    // F. Get the image width
    assign_param_or_default( nodeHandle , "/imagewdth" , imageWidth  , 1280 );
    // G. Get the image height
    assign_param_or_default( nodeHandle , "/imagehght" , imageHeight ,  720 );
    
	// 2. Init node rate
	ros::Rate heartbeat( RATE_HZ );
	
	// 3. Set up subscribers and publishers
	
	// ~ Publishers ~
	ros::Publisher arr_pub    = nodeHandle.advertise<std_msgs::Float64MultiArray>( "/filtered_distance" , QUEUE_LEN );
	
	// ~ Subscribers ~
	ros::Subscriber image_sub = nodeHandle.subscribe( imageTopic , QUEUE_LEN , image_filter_cb ); 
	
	// ~ Service Servers ~
	// ros::ServiceServer SERVER_OBJ = nodeHandle.advertiseService( "SERVICE_NAME" , SERV_CALLBACK );
	
	// ~ Service Clients ~
	// ros::ServiceClient CLIENT_OBJ = nodeHandle.serviceClient<CATEGORY::SERVICE_TYPE>( "SERVICE_NAME" );
	
	
	// N-2. Animation Init 
	//~ srand( ( time( 0 ) % 1 ) * 1000.0 + getpid() ); // Random seed based on the current clock time
	//~ RViz_MarkerManager mrkrMngr{};
	//~ visualization_msgs::MarkerArray& markerArr = mrkrMngr.get_arr();
	
	/// == PRE-LOOP WORK ===================================================================================================================
		
    // A. Set up the output array(s)
    distance_arr = vec_dbbl_zeros( numPts ); // Pre-allocate an array
    sums         = vec_dbbl_zeros( numPts ); // Pre-allocate an array
    blockLen     = vec_dbbl_zeros( numPts ); // Pre-allocate an array
    sampleBounds = even_index_bounds( imageWidth , numPts );
    std::vector<uint> widths = elem_count_bounds( sampleBounds );
    for( uint i = 0 ; i < numPts ; i++ ){  blockLen[i] = widths[i] * 1.0 * numLin;  }
    	
    // B. Instantiate message
    std_msgs::Float64MultiArray flt_dist_msg;
        
	/// __ END PRE-LOOP ____________________________________________________________________________________________________________________
	
	// N-1. Notify
	ros_log( "[" + NODE_NAME + "] Init OK and about to run ..." , INFO );
	
	// N. Main loop
	while( ros::ok() ){ // While neither node nor ROS has been shut down
		
		/// == LOOP WORK ===================================================================================================================
		
		// Every loop, send whatever is stored in the distance array
        flt_dist_msg.data.clear();
        flt_dist_msg.data = vec_copy( distance_arr );
        arr_pub.publish( flt_dist_msg );
		
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
