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



#ifndef BUFFNET_H
#define BUFFNET_H

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int MODEL_IN_WIDTH =      640;
const int MODEL_IN_HEIGHT =     640;
const int MODEL_IN_CHANNELS =   3;
const int NPU_USE_FLOATS =      1;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void dump_tensor_attr(rknn_tensor_attr*);
// unsigned char* load_model(const char*, int*);

class Buffnet {
private:
    rknn_context            ctx;
    int                     ret;
    int                     model_len;
    int                     model_active;
    unsigned char*          model;
    const char*             model_path;
    rknn_input_output_num   io_num;
    rknn_input              inputs[1];
    rknn_output             outputs[1];

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
};

#endif