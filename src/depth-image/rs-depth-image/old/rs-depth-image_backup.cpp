// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// Used rs-imshow as a go-by
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <iostream>

// https://stackoverflow.com/questions/19331575/cout-and-endl-errors
using std::cout;
using std::endl;

int main(int argc, char * argv[]) try
{

    // Declare counter
    unsigned int cnt;
    
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    rs2::pipeline_profile profile = pipe.start();
    
    // Start streaming with default recommended configuration
    rs2::device dev = profile.get_device();
    
    // Find depth sensor 
    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    rs2::depth_sensor ds = dev.first<rs2::depth_sensor>();//front().as<depth_sensor>();

    // Find depth sensor scaling factor
    float scale = ds.get_depth_scale();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && cvGetWindowHandle(window_name))
    {
        // Wait for next set of frames from the camera
        rs2::frameset data = pipe.wait_for_frames();
	
	// Get depth image data
	rs2::frame depth = data.get_depth_frame();
	
	// Convert to depth image data to RGB colormap
	rs2::frame depth_RGB = depth.apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix for imshow (RGB visualization)
        cv::Mat image_8bit_RGB(Size(w, h), CV_8UC3, (void*)depth_RGB.get_data(), Mat::AUTO_STEP);

        // Obtain scaled 16-bit matrix (metersdd) for printing/calculations
	// https://stackoverflow.com/questions/6909464/convert-16-bit-depth-cvmat-to-8-bit-depth
        cv::Mat image_16bit_RGB(Size(w, h), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);
	cv::Mat image_16bit(Size(w, h), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);
	//image_16bit = image_16bit * scale;
	//cv::Mat image_16bit_m;
	//image_16bit_m.convertTo(image_16bit, CV_16U, 1);//0.00390625); // Note: 1/256 = 0.00390625

	// Obtains 8-bit matrix from 8-bit matrix for imshow (grayscale visualation)
	cv::Mat image_8bit;
	image_8bit.convertTo(image_16bit, CV_8U, 0.00390625); // Note: 1/256 = 0.00390625 
	
	// Print out data
	if (cnt == 100)
	{	
                cout << "scale = "<< endl << " "  << scale << endl << endl;
		cout << "w = "<< endl << " "  << w << endl << endl;
        	cout << "h = "<< endl << " "  << h << endl << endl;
        	
		// https://stackoverflow.com/questions/7970988/print-out-the-values-of-a-mat-matrix-in-opencv-c
        	cout << "image = "<< endl << " "  << image_16bit_RGB<< endl << endl;

		cnt = 0;
	}
	
	cnt++;

        // Update the window with new data
        imshow(window_name, image_8bit_RGB);

    }
    
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



