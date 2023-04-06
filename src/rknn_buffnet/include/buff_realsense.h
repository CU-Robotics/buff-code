/*-------------------------------------------
                Includes
-------------------------------------------*/
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

// #include "opencv2/core/core.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc.hpp"
// #include "rknn_api.h"

// #include <stdint.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <sys/time.h>


#ifndef BUFF_REALSENSE_H
#define BUFF_REALSENSE_H

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEPTH_STREAM    RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define COLOR_STREAM    RS2_STREAM_COLOR  // rs2_stream is a types of data provided by RealSense device           //
#define DEPTH_FORMAT    RS2_FORMAT_Z16    // rs2_format identifies how binary data is encoded within a frame      //
#define COLOR_FORMAT    RS2_FORMAT_RGB8   // rs2_format identifies how binary data is encoded within a frame      //
#define RS_WIDTH        640               // Defines the number of columns for each frame                         //
#define RS_HEIGHT       480               // Defines the number of lines for each frame                           //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Buff_RealSense {
private:
    rs2::config cfg;                // sets the above macros
    rs2::pipeline pipeline;         // the actual data pipeline
    rs2::frameset frameset;         // object to store output of the pipeline
    rs2::frame color_image;         // color image
    rs2::frame depth_image;         // depth image

public:
    Buff_RealSense();

    void init_ros_publishers(image_transport::ImageTransport*);
    void read_frames();
    
    void publish_frames();

    double get_fps();
    rs2::frame get_depth();
    rs2::frame get_color();

    // Convert RealSense frame to OpenCV matrix:
    cv::Mat get_depth_cv();
    cv::Mat get_color_cv();
};

#endif