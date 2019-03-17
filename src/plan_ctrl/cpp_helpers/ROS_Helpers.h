#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
ROS_Helpers.h
James Watson , 2018 June
Common ROS Functions & Tasks

Template Version: 2017-09-23
***********/

#ifndef ROS_HELP_H // This pattern is to prevent symbols to be loaded multiple times
#define ROS_HELP_H // from multiple imports

// ~~ Includes ~~
// ~ ROS ~
#include <ros/ros.h> // ROS , Publishers , Subscribers
// ~ Local ~
#include <Cpp_Helpers.h> // Utilities and Shortcuts


// ~~ Shortcuts and Aliases ~~


// === Classes and Structs =================================================================================================================

enum LogLevel{ INFO , WARN , ERROR };

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

void ros_log( string msg , LogLevel level );

string to_string( std::__cxx11::basic_string<char>& inStr );

template<typename T>
bool assign_param_or_default( ros::NodeHandle& nh , string paramName , T& paramVar , const T& defaultVal ){
    string msg;
    if ( nh.getParam( paramName , paramVar ) ){
        T val = paramVar;
        msg = "Got param '" + paramName + "' with value " + to_string( val );
        ROS_INFO( "%s" , msg.c_str() );
        return true;
    }else{
        msg = ( "Failed to get param '" + string( paramName ) + "'" );
        ROS_ERROR( "%s" , msg.c_str() );
        paramVar = defaultVal;
        return false;
    }
}

size_t rowmajor_flat_index( size_t cols , size_t i , size_t j ); // Return the array index of a flattened 2D array

// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

