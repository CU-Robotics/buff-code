/*-------------------------------------------
                Includes
-------------------------------------------*/
#include "buff_realsense.h"

// #include "opencv2/core/core.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc.hpp"
#include "rknn_api.h"

// #include <stdint.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <sys/time.h>

#include "postprocess.h"
#include "buff_realsense.h"


#ifndef BUFFNET_H
#define BUFFNET_H

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const float nms_threshold           = NMS_THRESH;
const float box_conf_threshold      = BOX_THRESH;
const int MODEL_IN_WIDTH            = 640;
const int MODEL_IN_HEIGHT           = 640;
const int MODEL_IN_CHANNELS         = 3;
const int NPU_USE_FLOATS            = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void dump_tensor_attr(rknn_tensor_attr*);
// unsigned char* load_model(const char*, int*);

class Buffnet {
private:
    rknn_context            ctx;
    size_t                  actual_size;
    int                     img_width;
    int                     img_height;
    int                     img_channel;
    int                     channel;
    int                     width;
    int                     height;
    int                     ret;
    int                     model_len;
    int                     model_active;
    unsigned char*          model;
    const char*             model_path;
    rknn_input_output_num   io_num;
    rknn_tensor_attr        input_attrs[1];
    rknn_tensor_attr        output_attrs[3];
    rknn_input              inputs[1];
    rknn_output             outputs[3];
    struct timeval          start_time;
    struct timeval          stop_time;
    cv::Mat                 annotated_img;

    Buff_RealSense rs;

public:
    Buffnet();
    ~Buffnet();
    void model_info();
    void init_model();
    int  spin_model();
    void spin_realsense();
    cv::Mat get_depth();
    cv::Mat get_color();
    cv::Mat get_annot();
};

#endif