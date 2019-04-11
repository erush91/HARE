# SUBLEVEL CMAKELISTS

# ~ Project Setup ~
cmake_minimum_required( VERSION 2.6 )
project( cpp_helpers )

# ~ Build Settings ~
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # This will prevent g++ from getting upset about default values

# ~ Source Dirs ~
#~ add_subdirectory( src ) # Or where you keep all the source files
#~ add_subdirectory( cpp_helpers ) # Or where you keep all the source files
#~ set( CPP_HELP_CPP cpp_helpers/Cpp_Helpers.cpp )

# ~~~ EXEC_TITLE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#~ add_executable( EXEC_NAME MAIN.cpp SUPPORTING_ETC.cpp )

#~ target_link_libraries( EXEC_NAME ${LIBPATH1} ${LIBPATH2} )
