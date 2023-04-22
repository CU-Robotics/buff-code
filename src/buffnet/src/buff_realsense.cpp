#include "buff_realsense.h"


// Convert rs2::frame to cv::Mat
static cv::Mat frame_to_mat(const rs2::frame& f)
{
    auto vf = f.as<rs2::video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        cv::Mat r_rgb = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat r_bgr;
        cv::cvtColor(r_rgb, r_bgr, cv::COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return cv::Mat(cv::Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
static cv::Mat depth_frame_to_meters( const rs2::depth_frame & f )
{
    cv::Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    dm = dm * f.get_units();
    return dm;
}

Buff_RealSense::Buff_RealSense() {
  	// Use a configuration object to request depth and color from the pipeline
  	ROS_INFO("Configuring Realsense");
  	cfg.enable_stream(DEPTH_STREAM, RS_WIDTH, RS_HEIGHT, DEPTH_FORMAT, FPS);
  	cfg.enable_stream(COLOR_STREAM, RS_WIDTH, RS_HEIGHT, COLOR_FORMAT, FPS);
  	pipeline.start(cfg);
  	frameset = pipeline.wait_for_frames();
  	ROS_INFO("Hello Realsense!");
}


void Buff_RealSense::read_frames() {
	frameset = pipeline.wait_for_frames();
    depth_image = frameset.get_depth_frame();
    if (!depth_image) {
     	ROS_ERROR("Missed depth image");
    }

    color_image = frameset.get_color_frame();
    if (!color_image) {
     	ROS_ERROR("Missed color image");
    }
}


double Buff_RealSense::get_fps() {
	return double(FPS);
}

rs2::frame Buff_RealSense::get_depth() {
	return depth_image;
}

rs2::frame Buff_RealSense::get_color() {
	return color_image;
}

cv::Mat Buff_RealSense::get_depth_cv() {
	return depth_frame_to_meters(depth_image);
}

cv::Mat Buff_RealSense::get_color_cv() {
	return frame_to_mat(color_image);
}

cv::Mat Buff_RealSense::get_color_cv_resized(int w, int h) {
    cv::Mat resized;
    cv::resize(frame_to_mat(color_image), resized, cv::Size(w, h), 0, 0, CV_INTER_LINEAR);
    return resized;
}
