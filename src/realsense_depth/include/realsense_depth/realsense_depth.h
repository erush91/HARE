#include <ros/ros.h>

#include "std_msgs/String.h"

// Used rs-imshow as a go-by
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <iostream>

#include <cv_bridge/cv_bridge.h> // Bridge between OpenCV and ROS

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

//cv::Mat mat_depth_RGB_8bit
// bool DEPTH_FLAG = 0;
// int DEPTH_WIDTH = 848;
// int DEPTH_HEIGHT = 480;

// bool INFRARED_FLAG = 0;
// int INFRARED_WIDTH = 848;
// int INFRARED_HEIGHT = 480;

// bool COLOR_FLAG = 0;
// int COLOR_WIDTH = 848;
// int COLOR_HEIGHT = 480;
    
// cv::Mat mat_depth_RGB_8bit = cv::Mat::zeros(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3);
// cv::Mat mat_infrared_left = cv::Mat::zeros(cv::Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1);
// cv::Mat mat_infrared_right = cv::Mat::zeros(cv::Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1);
  
