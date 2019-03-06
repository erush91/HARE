// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <ros/ros.h>

// Used rs-imshow as a go-by
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <iostream>

#include <cv_bridge/cv_bridge.h> // Bridge between OpenCV and ROS

// https://stackoverflow.com/questions/19331575/cout-and-endl-errors
using std::cout;
using std::endl;

int main(int argc, char * argv[]) try
{
    /////////////////////////////
    // REALSENSE CONFIGURATION //
    /////////////////////////////

    // Declare counter
    unsigned int cnt;
    
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Configured depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);

    // Configured left infrared stream
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 60);
    
    // Configured right infrared stream
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 60);

    //Instruct pipeline to start streaming with the requested configuration
    //pipe.start(cfg);

    // Start streaming with default recommended configuration
    rs2::pipeline_profile profile = pipe.start(cfg);
    
    // Find device
    rs2::device dev = profile.get_device();
    
    // Find depth sensor 
    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    rs2::depth_sensor ds = dev.first<rs2::depth_sensor>();//front().as<depth_sensor>();
    
    // Find depth sensor scaling factor
    float scale = ds.get_depth_scale();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    ///////////////////////
    // ROS CONFIGURATION //
    ///////////////////////

    // Publish sensor_msgs::Image from cv::Mat
    // https://answers.ros.org/question/99831/publish-file-to-image-topic/
    ros::init(argc, argv, "depth_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1);
    ros::Rate loop_rate(60); // currently RealSense defaults to 30 FPS

    /////////////////////
    // FRAME RATE LOOP //
    /////////////////////

    while (nh.ok() && waitKey(1) < 0 && cvGetWindowHandle(window_name)) 
    {
        //////////////////////////////
        // WAIT FOR REALSENSE FRAME //
        //////////////////////////////

        // Wait for next set of frames from the camera
        rs2::frameset data = pipe.wait_for_frames();

        /////////////////////
        // GET DEPTH IMAGE //
        /////////////////////

        // Get depth image data
        rs2::frame depth = data.get_depth_frame();

        // Convert to depth image data to RGB colormap
        rs2::frame depth_RGB = depth.apply_filter(color_map);
        
        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix for imshow (RGB visualization)
        cv::Mat depth_image_8bit_RGB(Size(w, h), CV_8UC3, (void*)depth_RGB.get_data(), Mat::AUTO_STEP);

        // Obtain scaled 16-bit matrix for calculations
        // https://stackoverflow.com/questions/6909464/convert-16-bit-depth-cvmat-to-8-bit-depth
        //cv::Mat depth_image_16bit(Size(w, h), CV_16U, (void*)(depth.get_data()), Mat::AUTO_STEP);

        // https://stackoverflow.com/questions/17892840/opencv-multiply-scalar-and-matrix
        // depth_image_16bit *= scale;

        // Obtain depth image (meters) for calculations
        //https://stackoverflow.com/questions/6302171/convert-uchar-mat-to-float-mat-in-opencv
        //cv::Mat depth_image_float_m;
        //depth_image_16bit.convertTo(depth_image_float_m, CV_32F, scale);//0.00390625); // Note: 1/256 = 0.00390625

        // Copy one row of depth image to a new matrix
        //cv::Mat depth_vector_float_m(Size(w, 0), CV_32F);
        //depth_vector_float_m.push_back(depth_image_float_m.row(h-1));
            
        // Print out data
        if (cnt == 100)
        {	
            //cout << "scale = "<< endl << " "  << scale << endl << endl;
            //cout << "w = "<< endl << " "  << w << endl << endl;
            //cout << "h = "<< endl << " "  << h << endl << endl;
            
            // https://stackoverflow.com/questions/7970988/print-out-the-values-of-a-mat-matrix-in-opencv-c
            //cout << "\n\n\n\n\n\n\n\n\n\n depth image [m] = "<< endl << " "  << depth_image_float_m << endl << endl;
            //cout << "\n\n\n\n\n\n\n\n\n\n depth vector [m] = "<< endl << " "  << depth_vector_float_m << endl << endl;

            cnt = 0;
        }

        // cv::Mat --> sensor_msgs::Image
        cv_bridge::CvImage cv_image;
        cv_image.image = depth_image_8bit_RGB;
        cv_image.encoding = "bgr8";
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);

        /////////////////////////
        // GET INFRARED IMAGES //
        /////////////////////////



        ///////////////////
        // ROS PUBLISHER //
        ///////////////////

        pub.publish(ros_image);

        ///////////////////////////
        // ROS LOOP RATE CONTROL //
        ///////////////////////////
        loop_rate.sleep();
        
    }

    cnt++;
        
        // Update the window with new data
        //imshow(window_name, depth_image_8bit_RGB);
    
    /*
    int main(int argc, char **argv) 
    {
        const int num_points = 5;
        const int vec_length = 3;
        cv::Mat A(num_points, vec_length, CV_32FC1);
        cv::RNG rng(0); // Fill A with random values
        rng.fill(A, cv::RNG::UNIFORM, 0, 1);
        cv::Mat B = cv::Mat(0,vec_length, CV_32FC1);
        B.push_back(A.row(0));
        B.push_back(A.row(2));
        B.push_back(A.row(4));
        std::cout << "A: " << A << std::endl;
        std::cout << "B: " << B << std::endl;
        return 0;
    }    
    */
    
    return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}

catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
