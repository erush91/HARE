///////////////////////////////////////////////////////////////////////////////
// Function Name : depth
// Purpose : ROS publisher for depth, infra1, and infra2 images
///////////////////////////////////////////////////////////////////////////////
// Changelog :
// Date           % Name       %   Reason
// 03 / 04 / 2019 % Gene Rush  %   Created code
// 03 / 06 / 2019 % Gene Rush  %   Added this comment block
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include "std_msgs/String.h"

// Used rs-imshow as a go-by
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <iostream>

#include <cv_bridge/cv_bridge.h> // Bridge between OpenCV and ROS

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include "rs_custom_wrapper/rs_custom_wrapper.h"

// https://stackoverflow.com/questions/19331575/cout-and-endl-errors
using std::cout;
using std::endl;

int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "rs_custom_node");
    ros::NodeHandle nh_("~"); // the "~" is required to get parameters

    ////////////////////
    // GET PARAMETERS //
    ////////////////////

    bool CV_IMSHOW_VISUALIZER_FLAG = 0;

    bool DEPTH_RGB_FLAG = 0;

    bool LASER_FLAG = 0;

    bool DEPTH_FLAG = 0;
    int DEPTH_WIDTH = 848;
    int DEPTH_HEIGHT = 480;
    int DEPTH_FPS = 30;

    bool INFRARED_FLAG = 0;
    int INFRARED_WIDTH = 848;
    int INFRARED_HEIGHT = 480;
    int INFRARED_FPS = 30;

    bool COLOR_FLAG = 0;
    int COLOR_WIDTH = 848;
    int COLOR_HEIGHT = 480;
    int COLOR_FPS = 30;

    nh_.param("CV_IMSHOW_VISUALIZER_FLAG", CV_IMSHOW_VISUALIZER_FLAG, CV_IMSHOW_VISUALIZER_FLAG);

    nh_.param("DEPTH_RGB_FLAG", DEPTH_RGB_FLAG, DEPTH_RGB_FLAG);
    
    nh_.param("LASER_FLAG", LASER_FLAG, CV_IMSHOW_VISUALIZER_FLAG);
    
    nh_.param("DEPTH_FLAG", DEPTH_FLAG, DEPTH_FLAG);
    nh_.param("DEPTH_WIDTH", DEPTH_WIDTH, DEPTH_WIDTH);
    nh_.param("DEPTH_HEIGHT", DEPTH_HEIGHT, DEPTH_HEIGHT);
    nh_.param("DEPTH_FPS", DEPTH_FPS, DEPTH_FPS);

    nh_.param("INFRARED_FLAG", INFRARED_FLAG, INFRARED_FLAG);
    nh_.param("INFRARED_WIDTH", INFRARED_WIDTH, INFRARED_WIDTH);
    nh_.param("INFRARED_HEIGHT", INFRARED_HEIGHT, INFRARED_HEIGHT);
    nh_.param("INFRARED_FPS", INFRARED_FPS, INFRARED_FPS);

    nh_.param("COLOR_FLAG", COLOR_FLAG, COLOR_FLAG);
    nh_.param("COLOR_WIDTH", COLOR_WIDTH, COLOR_WIDTH);
    nh_.param("COLOR_HEIGTH", COLOR_HEIGHT, COLOR_HEIGHT);
    nh_.param("COLOR_FPS", COLOR_FPS, COLOR_FPS);

    //////////////////////////
    // INITIALIZE REALSENSE //
    //////////////////////////

    // Declare first loop flag
    bool first_loop_flag = 1;

    // Declare counter
    unsigned int cnt = 0;

    // Declare depth colorizer for pretty visualization of depth frame
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //////////////////////////////
    // SET REALSENSE FRAME SIZE //
    //////////////////////////////
    
    // D435: Use 848x480 resolution @30fps, with auto-exposure. Use post processing with downsample 2.
    // https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets
   
    // WE WANT 848 x 480 RESOLUTION AT 30 FPS

    // Configured depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, DEPTH_FPS);

    // Configured left infrared stream
    // https://github.com/IntelRealSense/librealsense/issues/1140
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, INFRARED_WIDTH, INFRARED_HEIGHT, RS2_FORMAT_Y8, INFRARED_FPS);
    
    // Configured right infrared stream
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, INFRARED_WIDTH, INFRARED_HEIGHT, RS2_FORMAT_Y8, COLOR_FPS);

    // Configured right infrared stream
    //cfg.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_BGR8, COLOR_FPS);

    // Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile profile = pipe.start(cfg);

    /////////////////////////////////
    // GET REALSENSE DEPTH SCALING //
    /////////////////////////////////

    // Find device
    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    auto dev = profile.get_device();

    // Find first depth sensor (device_list can have zero or more then one)
    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    auto depth_sensor = dev.first<rs2::depth_sensor>();

    // Find depth scale (to meters)
    auto scale =  depth_sensor.get_depth_scale();

    ////////////////////////////////////////
    // GET REALSENSE INTRINSIC PARAMETERS //
    ////////////////////////////////////////
    // DOES NOT WORK!!!!!!
    //auto const intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

    ////////////////////////////////////////
    // GET REALSENSE EXTRINSIC PARAMETERS //
    ////////////////////////////////////////
    //pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_extrinsics_to(depth_sensor& to);

    //////////////////////////
    // TURN LASER ON OR OFF //
    //////////////////////////

    // https://github.com/IntelRealSense/librealsense/wiki/API-How-To
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        if(LASER_FLAG)
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        }
        else
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
        }
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        if(LASER_FLAG)
        {
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        }
        else
        {
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
        }
    }

    //////////////////////////////////
    // CONFIGURE REALSENSE SETTINGS //
    //////////////////////////////////

    // How to load json files
    // https://github.com/IntelRealSense/librealsense/issues/1021

    // Provides json files for different applications
    // https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets
    // https://github.com/IntelRealSense/librealsense/issues/1235

    // Miscellaneous forums
    // https://github.com/IntelRealSense/librealsense/issues/2193
    // https://github.com/IntelRealSense/librealsense/issues/1171
    // https://github.com/IntelRealSense/librealsense/issues/1235

    // White paper
    // https://www.intel.com/content/www/us/en/support/articles/000027833/emerging-technologies/intel-realsense-technology.html
    // https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/BKMs_Tuning_RealSense_D4xx_Cam.pdf

    // https://stackoverflow.com/questions/53520301/how-to-load-json-file-to-realsense-d435-camera-using-c-and-intel-api
    // Obtain a list of device_list currently present on the system

    // Load camera settings (.json file)
    // https://stackoverflow.com/questions/53520301/how-to-load-json-file-to-realsense-d435-camera-using-c-and-intel-api
    // https://github.com/IntelRealSense/librealsense/issues/1229

    ///////////////////////////////////////
    // LOAD .JSON FILE (REALSEN SETTINGS //
    ///////////////////////////////////////
    
    // rs2::context ctx;
    // auto device_list = ctx.query_devices();
    // size_t device_count = device_list.size();
    // if (!device_count)
    // {
    //     cout <<"No device detected. Is it plugged in?\n";
    //     return EXIT_SUCCESS;
    // }

    // // Get the first connected device
    // auto dev0 = device_list[0];

    // // Enter advanced mode
    // if (dev0.is<rs400::advanced_mode>())
    // {
    //     // Get the advanced mode functionality
    //     auto advanced_mode_dev = dev0.as<rs400::advanced_mode>();

    //     // Load and configure .json file to device
    //     std::ifstream t("./src/depth/json/DefaultPreset_D435.json"); //HighResHighAccuracyPreset.json
    //     std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    //     advanced_mode_dev.load_json(str);
    // }
    // else
    // {
    //     cout << "Current device doesn't support advanced-mode!\n";
    //     return EXIT_FAILURE;
    // }

    //////////////////////////////////////
    // CONFIGURE AUTO EXPOSURE SETTINGS //
    //////////////////////////////////////
    
    // TO DO

    // WE WANT TO DOWNSAMPLE 2
    // https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets

    // WE WANT TO TURN ON AUTO EXPOSURE
    // https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets
    
    // NOT SURE HOW TO DO THIS, FORUMS SUGGEST POINTS
    // https://software.intel.com/sites/landingpage/realsense/camera-sdk/v1.1/documentation/html/member_functions_neutral_device_pxccapture.html
    // https://github.com/IntelRealSense/librealsense/issues/1542
    //dev->EnableAutoExposure(1.0f);
    //static rs2_option get_sensor_option(const dev& AutoExposure);

    //////////////////////////////////////
    // CONFIGURE DOWN SAMPLING SETTINGS //
    //////////////////////////////////////

    // TO DO
    
    ////////////////////////////////////////////
    // CONFIGURE IMSHOW VISUALIZATION WINDOWS //
    ////////////////////////////////////////////
        
    using namespace cv;
    const auto window_name_depth = "Depth";
    namedWindow(window_name_depth, WINDOW_AUTOSIZE);
    
    const auto window_name_infrared_left = "IR (Left)";
    namedWindow(window_name_infrared_left, WINDOW_AUTOSIZE);

    const auto window_name_infrared_right = "IR (Right)";
    namedWindow(window_name_infrared_right, WINDOW_AUTOSIZE);
    
    //////////////////////////////
    // CONFIGURE ROS PUBLISHERS //
    //////////////////////////////

    // Publish sensor_msgs::Image
    // https://answers.ros.org/question/99831/publish-file-to-image-topic/

    ros::Publisher pub_image_depth_8b = nh_.advertise<sensor_msgs::Image>("/camera/depth_8b/image_rect_raw", 1);
    ros::Publisher pub_image_depth_RGB = nh_.advertise<sensor_msgs::Image>("/camera/depth_RGB/image_rect_raw", 1);
    ros::Publisher pub_image_depth_meters = nh_.advertise<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1);
    ros::Publisher pub_image_infrared_left = nh_.advertise<sensor_msgs::Image>("/camera/infra1/image_rect_raw", 1);
    ros::Publisher pub_image_infrared_right = nh_.advertise<sensor_msgs::Image>("/camera/infra2/image_rect_raw", 1);
    
    //////////////////////////
    // GET REALSENSE FRAMES //
    //////////////////////////
    //rs2::pipeline_profile selection = pipe.start();
    //auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH);
    //auto color_stream = profile.get_stream(RS2_STREAM_COLOR);
    //rs2::rs2_extrinsics e = depth_stream.get_extrinsics_to(color_stream);
    
    cv::Mat mat_depth_8b(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8U);

    while (nh_.ok() && waitKey(1) < 0) 
    {
        cnt++;

        //////////////////////////////////
        // PRINT PARAMETERS TO TERMINAL //
        //////////////////////////////////

        if(first_loop_flag == 1)
        {
            /////////////////////////////////////////
            // PRINT DEPTH FRAME SIZE TO TERIMINAL //
            /////////////////////////////////////////

            cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
            cout << "CV_IMSHOW_VISUALIZER_FLAG: " << CV_IMSHOW_VISUALIZER_FLAG << endl;
            cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
            cout << "DEPTH_RGB_FLAG: " << DEPTH_RGB_FLAG << endl;
            cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
            cout << "LASER_FLAG: " << LASER_FLAG << endl;
            cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
            cout << "DEPTH_FLAG: " << DEPTH_FLAG << endl;
            cout << "DEPTH FRAME SIZE: [" << DEPTH_WIDTH << ", " << DEPTH_HEIGHT << "]" << endl;
            cout << "DEPTH_FPS: " << DEPTH_FPS << endl;
            cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
            cout << "INFRARED_FLAG: " << INFRARED_FLAG << endl;
            cout << "INFRARED FRAME SIZE: [" << INFRARED_WIDTH << ", " << INFRARED_HEIGHT << "]" << endl;
            cout << "INFRARED_FPS: " << INFRARED_FPS << endl;
            cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
            cout << "COLOR_FLAG: " << COLOR_FLAG << endl;
            cout << "COLOR FRAME SIZE: [" << COLOR_WIDTH << ", " << COLOR_HEIGHT << "]" << endl;
            cout << "COLOR_FPS: " << COLOR_FPS << endl;
            cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
            //ROS_INFO("%d", rs2_extrinsics); 
            /////////////////////////
            // INITIALIZE MATRICES //
            /////////////////////////

            // http://answers.opencv.org/question/113449/how-to-initialize-mat-size-and-fill-it-with-zeros-in-a-class/

            // cv::Mat mat_depth_RGB(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_64FC1);
            // cv::Mat mat_infrared_left = cv::Mat::zeros(cv::Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1);
            // cv::Mat mat_infrared_right = cv::Mat::zeros(cv::Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1);

            first_loop_flag = 0;
        }

        ///////////////////////////////
        // PRINT FRAME # TO TERMINAL //
        ///////////////////////////////

        cout << "Frame # " << cnt << endl;

        //////////////////////////////
        // WAIT FOR REALSENSE FRAME //
        //////////////////////////////
        
        // Wait for next set of frames from the camera
        rs2::frameset frame_set = pipe.wait_for_frames();

        if(DEPTH_FLAG)
        {
            //////////////////////////////
            // GET IMAGES FROM FRAMESET //
            //////////////////////////////
            rs2::frame frame_depth = frame_set.get_depth_frame();
 
            ////////////////////////////////////////
            // SEND FRAME DATA TO OPENCV MATRICES //
            ////////////////////////////////////////

            // Obtain 16-bit depth matrix
            // https://stackoverflow.com/questions/6909464/convert-16-bit-depth-cvmat-to-8-bit-depth
            cv::Mat mat_depth_16b(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16U, (void*)(frame_depth.get_data()), Mat::AUTO_STEP);
      
            // Convert to 8-bit depth matrix (for sensor_msgs::Image)
            // https://stackoverflow.com/questions/6909464/convert-16-bit-depth-cvmat-to-8-bit-depth
            mat_depth_16b.convertTo(mat_depth_8b, CV_8U, 0.00390625); // Note: 1/256 = 0.00390625

            // Obtain depth image (meters) for calculations
            //https://stackoverflow.com/questions/6302171/convert-uchar-mat-to-float-mat-in-opencv
            cv::Mat mat_depth_meters;
            mat_depth_16b.convertTo(mat_depth_meters, CV_32F, scale); //0.00390625); // Note: 1/256 = 0.00390625

            // Copy one row of depth image to a new matrix
            cv::Mat vec_depth_meters(Size(DEPTH_WIDTH, 0), CV_32F);
            vec_depth_meters.push_back(mat_depth_meters.row(int(DEPTH_HEIGHT-1)));

            // CURRENTLY, DOES NOT WORK BECAUSE SENSOR_MSGS CAN ONLY ACCEPT 8BIT IMAGES...
            
            ////////////////////////////////////////////////
            // CONVERT 8-BIT CV:MAT to SENSOR_MSGS::IMAGE //
            ////////////////////////////////////////////////
            cv_bridge::CvImage cv_image_depth_8b;
            cv_image_depth_8b.image = mat_depth_8b;
            cv_image_depth_8b.encoding = "mono8";
            sensor_msgs::Image ros_image_depth_8b;
            cv_image_depth_8b.toImageMsg(ros_image_depth_8b);

            //////////////////////////////////////////
            // CONVERT CV:MAT to SENSOR_MSGS::IMAGE //
            //////////////////////////////////////////
            cv_bridge::CvImage cv_image_depth_meters;
            cv_image_depth_meters.image = mat_depth_meters;
            cv_image_depth_meters.encoding = "mono16";
            sensor_msgs::Image ros_image_depth_meters;
            cv_image_depth_meters.toImageMsg(ros_image_depth_meters);

            // PRINT MATRIX ONCE
            if (cnt == 10)
            {	                
                // https://stackoverflow.com/questions/7970988/print-out-the-values-of-a-mat-matrix-in-opencv-c
                cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n depth image [m] = "<< endl << " "  << mat_depth_8b << endl << endl;
                cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n depth vector [m] = "<< endl << " "  << vec_depth_meters << endl << endl;
            }

            ////////////////////////////////////////
            // PUBLISH ROS MESSAGES TO ROS TOPICS //
            ////////////////////////////////////////
            pub_image_depth_8b.publish(ros_image_depth_8b);
            pub_image_depth_meters.publish(ros_image_depth_meters);

            if(DEPTH_RGB_FLAG)
            {
                ////////////////////////
                // APPLY RGB COLORMAP //
                ////////////////////////
                rs2::frame frame_depth_RGB = frame_depth.apply_filter(color_map); // Convert to depth image frame to RGB colormap////////////////////////////////////////
                
                //////////////////////////////////////////////
                // SEND FRAME DATA TO 8-BIT OPENCV MATRICES //
                //////////////////////////////////////////////
                cv::Mat mat_depth_RGB(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3, (void*)frame_depth_RGB.get_data(), Mat::AUTO_STEP);
    
                //////////////////////////////////////////
                // CONVERT CV:MAT to SENSOR_MSGS::IMAGE //
                //////////////////////////////////////////
                cv_bridge::CvImage cv_image_depth_RGB;
                cv_image_depth_RGB.image = mat_depth_RGB;
                cv_image_depth_RGB.encoding = "bgr8";
                sensor_msgs::Image ros_image_depth_RGB;
                cv_image_depth_RGB.toImageMsg(ros_image_depth_RGB);

                if(CV_IMSHOW_VISUALIZER_FLAG)
                {
                    /////////////////////////////////////////
                    // UPDATE IMSHOW VISUALIZATION WINDOWS //
                    /////////////////////////////////////////
                    cv::imshow(window_name_depth, mat_depth_RGB);
                }

                ////////////////////////////////////////
                // PUBLISH ROS MESSAGES TO ROS TOPICS //
                ////////////////////////////////////////
                pub_image_depth_RGB.publish(ros_image_depth_RGB);
            }

        }

        if(INFRARED_FLAG)
        {
            //////////////////////////////
            // GET IMAGES FROM FRAMESET //
            //////////////////////////////
            rs2::video_frame frame_infrared_left = frame_set.get_infrared_frame(1);
            rs2::video_frame frame_infrared_right = frame_set.get_infrared_frame(2);

            ////////////////////////////////////////
            // SEND FRAME DATA TO OPENCV MATRICES //
            ////////////////////////////////////////
            cv::Mat mat_infrared_left(Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1, (void*)frame_infrared_left.get_data(), Mat::AUTO_STEP);
            cv::Mat mat_infrared_right(Size(INFRARED_WIDTH, INFRARED_HEIGHT), CV_8UC1, (void*)frame_infrared_right.get_data(), Mat::AUTO_STEP); 
            
            //////////////////////////////////////////
            // CONVERT CV:MAT to SENSOR_MSGS::IMAGE //
            //////////////////////////////////////////
            cv_bridge::CvImage cv_image_infrared_left;
            cv_image_infrared_left.image = mat_infrared_left;
            cv_image_infrared_left.encoding = "mono8";
            sensor_msgs::Image ros_image_infrared_left;

            cv_bridge::CvImage cv_image_infrared_right;
            cv_image_infrared_right.image = mat_infrared_right;
            cv_image_infrared_right.encoding = "mono8";
            sensor_msgs::Image ros_image_infrared_right;

            cv_image_infrared_left.toImageMsg(ros_image_infrared_left);
            cv_image_infrared_right.toImageMsg(ros_image_infrared_right);

            if( CV_IMSHOW_VISUALIZER_FLAG)
            {
                /////////////////////////////////////////
                // UPDATE IMSHOW VISUALIZATION WINDOWS //
                /////////////////////////////////////////
                cv::imshow(window_name_infrared_left, mat_infrared_left);
                cv::imshow(window_name_infrared_right, mat_infrared_right);
            }
            
            ////////////////////////////////////////
            // PUBLISH ROS MESSAGES TO ROS TOPICS //
            ////////////////////////////////////////
            pub_image_infrared_left.publish(ros_image_infrared_left);
            pub_image_infrared_right.publish(ros_image_infrared_right);

            // POINT CLOUD CREATION: HAVE NOT IMPLEMENTED
            // https://stackoverflow.com/questions/32521043/how-to-convert-cvmat-to-pclpointcloud
            // {
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat depthMat)
            // {
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::PointCLoud
            // // calibration parameters
            //     float const fx_d = 5.9421434211923247e+02;
            //     float const fy_d = 5.9104053696870778e+02;
            //     float const cx_d = 3.3930780975300314e+02;
            //     float const cy_d = 2.4273913761751615e+02;

            //     unsigned char* p = depthMat.data;
            //     for (int i = 0; i<depthMat.rows; i++)
            //     {
            //         for (int j = 0; j < depthMat.cols; j++)
            //         {
            //             float z = static_cast<float>(*p);
            //             pcl::PointXYZ point;
            //             point.z = 0.001 * z;
            //             point.x = point.z*(j - cx_d)  / fx_d;
            //             point.y = point.z *(cy_d - i) / fy_d;
            //             ptCloud->points.push_back(point);
            //             ++p;
            //         }
            //     }
            //     ptCloud->width = (int)depthMat.cols; 
            //     ptCloud->height = (int)depthMat.rows; 

            //     return ptCloud;

            // }
            // }

        }

        ////////////////////////////////////
        // EXTRACT ROW DATA FROM MATRICES //
        ////////////////////////////////////

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

        
		char c = cv::waitKey(1);
		if (c == 's')
		{

		}
		else if (c == 'q')
		{
            break;
        }
    }
      
    return EXIT_SUCCESS;
}
/////////////////
// END OF MAIN //
/////////////////

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
