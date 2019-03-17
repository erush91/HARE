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

// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

