// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
								Includes
-------------------------------------------*/
#include "buffnet.h"
#include "buff_realsense.h"

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

/*-------------------------------------------
									Main Function
-------------------------------------------*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Publisher depth_pub = it.advertise("/realsense/depth", 1);
    image_transport::Publisher color_pub = it.advertise("/realsense/color", 1);

	Buffnet buffnet;

	ros::Rate loop_rate(FPS);

	while (ros::ok()) {
		buffnet.spin_realsense();
	    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", buffnet.rs.get_depth_cv()).toImageMsg();
	    sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "BGR8", buffnet.rs.get_color_cv()).toImageMsg();
	    depth_pub.publish(depth_msg);
	    color_pub.publish(color_msg);
		loop_rate.sleep();
	}

	return 0;
}
