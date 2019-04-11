#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSenseImShowExample)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

## EXISTING CODE FROM rs-imshow
#add_executable(rs-imshow rs-imshow.cpp)
#target_link_libraries(rs-imshow ${DEPENDENCIES})
#set_target_properties (rs-imshow PROPERTIES
#	FOLDER "Examples/OpenCV"
#)

set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV 3.4.1 EXACT REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers
add_executable(rs-imshow rs-imshow.cpp)
target_link_libraries( rs-imshow ${OpenCV_LIBS} /usr/local/lib/librealsense2.so )
# #target_link_libraries( rs-imshow ${OpenCV_LIBS} /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0) # link OpenCV libraries , hightgui.so not found by cmake so this hack
MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS} )  #display opencv libs found

install(
	TARGETS

	rs-imshow

	RUNTIME DESTINATION
	${CMAKE_INSTALL_PREFIX}/bin
)
