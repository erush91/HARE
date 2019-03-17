/***********  
ROS_Helpers.cpp
James Watson , 2018 June
Common ROS Functions & Tasks

Template Version: 2017-09-23
***********/

#include "ROS_Helpers.h"

// === Classes and Structs =================================================================================================================



// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

void ros_log( string msg , LogLevel level ){
  switch( level )
  {
	  
    case WARN :
    {
      ROS_WARN_STREAM( msg );
      break;
    }
    
    case ERROR :
    {
      ROS_ERROR_STREAM( msg );
      break;    
    }
    
    default:
    {
      ROS_INFO_STREAM( msg );
      break;    
    }
    
  }
}

string to_string( std::__cxx11::basic_string<char>& inStr ){  return string( inStr );  }

size_t rowmajor_flat_index( size_t cols , size_t i , size_t j ){  return cols * i + j;  } // Return the array index of a flattened 2D array
    

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

